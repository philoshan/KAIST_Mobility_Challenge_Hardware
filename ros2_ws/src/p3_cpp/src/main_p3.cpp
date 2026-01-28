#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp> 
#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <map>
#include <deque> 

namespace Geo {
    struct Vec2 {
        double x, y;
        Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
        Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
        double dot(const Vec2& o) const { return x * o.x + y * o.y; }
        double dist_Sq() const { return x*x + y*y; }
    };

    bool check_obb_intersection(const std::vector<Vec2>& box1, const std::vector<Vec2>& box2) {
        auto get_axes = [](const std::vector<Vec2>& b) {
            return std::vector<Vec2>{
                {b[1].x - b[0].x, b[1].y - b[0].y},
                {b[1].x - b[2].x, b[1].y - b[2].y}
            };
        };
        std::vector<Vec2> axes = get_axes(box1);
        auto axes2 = get_axes(box2);
        axes.insert(axes.end(), axes2.begin(), axes2.end());

        for (const auto& axis : axes) {
            double min1 = 1e9, max1 = -1e9;
            double min2 = 1e9, max2 = -1e9;
            for (const auto& p : box1) {
                double proj = p.dot(axis);
                min1 = std::min(min1, proj); max1 = std::max(max1, proj);
            }
            for (const auto& p : box2) {
                double proj = p.dot(axis);
                min2 = std::min(min2, proj); max2 = std::max(max2, proj);
            }
            if (max1 < min2 || max2 < min1) return false;
        }
        return true;
    }
}

struct Vehicle {
    std::string id;
    bool is_cav;
    Geo::Vec2 pos{0.0, 0.0};
    double z = 0.0;
    bool active = false;
    bool is_stopped = false; 
    bool forced_stop = false; 
    std::string stop_cause = ""; 
    bool has_entered_roundabout = false; 

    // hv 속도
    Geo::Vec2 last_pos{0.0, 0.0};
    rclcpp::Time last_time;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_stop;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_change_way;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_is_roundabout;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_hv_vel;
};

class MainTrafficController : public rclcpp::Node {
public:
    MainTrafficController() : Node("main_traffic_controller") {
        car_info = {0.17, 0.16, 0.075, 0.075};
        front_padding_ = 0.8;
        side_padding_ = 0.8;
        approach_range_sq_ = 2.5 * 2.5;

        // 사지교차로
        fourway_center_ = {-2.333, 0.0};
        fourway_len_ = 2.0;
        fourway_app_r_sq_ = 1.5 * 1.5;
        fourway_box_half_len_ = fourway_len_ / 2.0;

        // 회전교차로
        round_center_ = {1.667, 0.0};
        round_app_r_sq_ = 1.8 * 1.8;
        round_radius_ = 1.4;

        // 삼지교차로
        threeway_box_x_min_ = -3.7; threeway_box_x_max_ = -1.2;
        threeway_1_y_min_ = 1.0;    threeway_1_y_max_ = 3.0;
        threeway_2_y_min_ = -3.0;   threeway_2_y_max_ = -1.0;

        tmr_discovery_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MainTrafficController::discover_vehicles, this));
        tmr_control_ = create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&MainTrafficController::control_loop, this));

        RCLCPP_INFO(get_logger(), "Controller Started: IDs Swapped (2->4, 3->6, 4->7)");
    }

private:
    rclcpp::TimerBase::SharedPtr tmr_discovery_, tmr_control_;
    std::unordered_map<std::string, Vehicle> vehicles_;
    std::map<std::string, std::string> conflict_info_;

    std::deque<double> hv_vel_buffer_;
    const int vel_buffer_size_ = 10;

    std::vector<double> car_info;
    double front_padding_, side_padding_, approach_range_sq_;
    Geo::Vec2 fourway_center_;
    double fourway_len_, fourway_app_r_sq_, fourway_box_half_len_;
    Geo::Vec2 round_center_;
    double round_app_r_sq_, round_radius_;
    double threeway_box_x_min_, threeway_box_x_max_, threeway_1_y_min_, threeway_1_y_max_, threeway_2_y_min_, threeway_2_y_max_;

    void update_conflict_status(const std::string& stopped_cav, const std::string& cause_vehicle) {
        if (conflict_info_.count(stopped_cav) && conflict_info_[stopped_cav] == cause_vehicle) return; 
        conflict_info_[stopped_cav] = cause_vehicle;
    }

    void clear_conflict_status(const std::string& stopped_cav) {
        if (conflict_info_.count(stopped_cav)) {
            conflict_info_.erase(stopped_cav);
        }
    }

    bool is_approaching(const Vehicle& spot, const Geo::Vec2& target_pos) {
        Geo::Vec2 vec = target_pos - spot.pos;
        return vec.dot({std::cos(spot.z), std::sin(spot.z)}) > 0;
    }

    std::vector<Geo::Vec2> get_vehicle_corners(const Vehicle& v, double front_ext, double side_pad) {
        double front = car_info[0] + front_ext;
        double rear = car_info[1];
        double left = car_info[2] + side_pad;
        double right = car_info[3] + side_pad;
        std::vector<Geo::Vec2> locals = {{front, left}, {front, -right}, {-rear, -right}, {-rear, left}};
        std::vector<Geo::Vec2> world_corners;
        double c = std::cos(v.z), s = std::sin(v.z);
        for (const auto& p : locals) {
            world_corners.push_back({
                v.pos.x + (p.x * c - p.y * s),
                v.pos.y + (p.x * s + p.y * c)
            });
        }
        return world_corners;
    }

    // [수정] Discovery Logic: 길이 제한 제거 및 CAV_06 등 인식 가능하게 변경
    void discover_vehicles() {
        auto topic_map = this->get_topic_names_and_types();
        for (const auto& [name, types] : topic_map) {
            if (name.empty() || name[0] != '/') continue;
            
            // 길이 제한 제거 (필수)
            
            std::string id = "";
            bool is_cav = false;

            size_t cav_pos = name.find("CAV_");
            size_t hv_pos = name.find("HV_");

            if (cav_pos != std::string::npos) {
                // CAV_XX 추출
                if (cav_pos + 6 <= name.size()) {
                    id = name.substr(cav_pos, 6);
                    is_cav = true;
                }
            } 
            else if (hv_pos != std::string::npos) {
                // HV_XX 추출
                if (hv_pos + 5 <= name.size()) {
                    id = name.substr(hv_pos, 5);
                    is_cav = false;
                }
            }

            if (!id.empty() && !vehicles_.count(id)) {
                // PoseStamped 확인 (안전장치)
                bool is_pose = false;
                for(const auto& t : types) {
                    if(t.find("PoseStamped") != std::string::npos) { is_pose = true; break; }
                }
                if(is_pose) register_vehicle(id, name, is_cav);
            }
        }
    }

    void register_vehicle(const std::string& id, const std::string& topic, bool is_cav) {
        Vehicle v; v.id = id; v.is_cav = is_cav;
        v.last_time = this->now();

        v.sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            topic, rclcpp::SensorDataQoS(),
            [this, id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (vehicles_.count(id)) {
                    Vehicle& veh = vehicles_[id];
                    double prev_x = veh.last_pos.x;
                    double prev_y = veh.last_pos.y;
                    rclcpp::Time prev_time = veh.last_time;

                    veh.pos = {msg->pose.position.x, msg->pose.position.y};
                    veh.z = msg->pose.orientation.z; 
                    veh.active = true;
                    veh.last_pos = veh.pos;
                    veh.last_time = msg->header.stamp;

                    if (id == "HV_19") { 
                        double dt = (veh.last_time - prev_time).seconds();
                        if (dt > 0.001) { 
                            double dx = veh.pos.x - prev_x;
                            double dy = veh.pos.y - prev_y;
                            double dist = std::hypot(dx, dy);
                            double vel = dist / dt;
                            hv_vel_buffer_.push_back(vel);
                            if (hv_vel_buffer_.size() > vel_buffer_size_) hv_vel_buffer_.pop_front();
                            double sum = 0.0;
                            for (double val : hv_vel_buffer_) sum += val;
                            double avg_vel = sum / hv_vel_buffer_.size();
                            std_msgs::msg::Float32 vel_msg;
                            vel_msg.data = static_cast<float>(std::max(0.0, avg_vel - 0.05)); 
                            for (auto& [target_id, target_v] : vehicles_) {
                                if (target_v.is_cav && target_v.active && target_v.pub_hv_vel) {
                                    target_v.pub_hv_vel->publish(vel_msg);
                                }
                            }
                        }
                    }
                }
            }
        );

        if (is_cav) {
            std::string prefix = "/" + id;
            v.pub_stop = create_publisher<std_msgs::msg::Bool>(prefix + "/cmd_stop", rclcpp::SensorDataQoS());
            v.pub_change_way = create_publisher<std_msgs::msg::Bool>(prefix + "/change_waypoint", rclcpp::SensorDataQoS());
            v.pub_is_roundabout = create_publisher<std_msgs::msg::Bool>(prefix + "/is_roundabout", rclcpp::SensorDataQoS());
            v.pub_hv_vel = create_publisher<std_msgs::msg::Float32>(prefix + "/hv_vel", rclcpp::SensorDataQoS());
        }
        vehicles_[id] = std::move(v);
        RCLCPP_INFO(get_logger(), "Registered Vehicle: %s", id.c_str());
    }

    bool is_in_threeway_zone(const Geo::Vec2& pos) {
        return (pos.x >= threeway_box_x_min_ && pos.x <= threeway_box_x_max_) &&
               ((pos.y >= threeway_1_y_min_ && pos.y <= threeway_1_y_max_) ||
                (pos.y >= threeway_2_y_min_ && pos.y <= threeway_2_y_max_));
    }

    // [수정] ID Swap: 3->6, 4->7, 2->4
    bool check_threeway_conflict(const Vehicle& my_cav, const std::string& tid, bool is_next_lane, bool is_full, std::string& temp_log) {
        if (tid == "CAV_06" || tid == "CAV_07") { // (구: 03, 04) -> (신: 06, 07)
            if (my_cav.id == "CAV_01" && is_next_lane) {
                temp_log = "threeway_NEXT_LANE";
                return true;
            } else if (my_cav.id == "CAV_04" && is_full && !is_next_lane) { // (구: 02) -> (신: 04)
                temp_log = "threeway_VERTICAL";
                return true;
            }
        }
        return false;
    }

    bool is_in_fourway_zone(const Vehicle& v){ return (v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_; }

    bool check_fourway_conflict(bool is_front, bool is_next_lane, std::string& temp_log) {
        if (is_front && !is_next_lane) {
            temp_log = "4WAY_FRONT";
            return true;
        }
        return false;
    }

    // [수정] ID Swap: 2->4, 3->6, 4->7
    bool check_fourway_rect_split(const Vehicle& my_cav) {
        double x_min = fourway_center_.x - fourway_box_half_len_;
        double x_max = fourway_center_.x + fourway_box_half_len_;
        double y_min = fourway_center_.y - fourway_box_half_len_;
        double y_max = fourway_center_.y + fourway_box_half_len_;

        bool i_am_inside = (my_cav.pos.x >= x_min && my_cav.pos.x <= x_max && my_cav.pos.y >= y_min && my_cav.pos.y <= y_max);
        if (i_am_inside) return false;
        if ((my_cav.pos - fourway_center_).dist_Sq() > fourway_app_r_sq_) return false;
        if (!is_approaching(my_cav, fourway_center_)) return false;
        
        bool am_i_top = (my_cav.pos.y > fourway_center_.y); 
        bool am_i_right = (my_cav.pos.x > fourway_center_.x); 

        if (my_cav.id == "CAV_01" || my_cav.id == "CAV_04") { // (구: 02 -> 신: 04)
            std::string tid = (my_cav.id == "CAV_01") ? "CAV_04" : "CAV_01"; // (구: 02 -> 신: 04)
            if (vehicles_.count(tid) && vehicles_.at(tid).active) {
                const auto& target = vehicles_.at(tid);
                bool target_inside = (target.pos.x >= x_min && target.pos.x <= x_max && target.pos.y >= y_min && target.pos.y <= y_max);
                if (target_inside) {
                    bool target_is_right = (target.pos.x > fourway_center_.x);
                    bool target_approaching = is_approaching(target, fourway_center_);
                    
                    if (my_cav.id == "CAV_01"){
                        if (am_i_top && !target_is_right && target_approaching) return true;
                        else if (!am_i_top && target_is_right && target_approaching) return true;
                    }
                    else{
                        bool target_is_top = (target.pos.y > fourway_center_.y);
                        if(!am_i_right && !target_is_top && target_approaching) return true;
                        else if (am_i_right && target_is_right && target_approaching) return true;
                    }
                }
            }
            auto is_target_in_box = [&](const std::string& tid) {
                if (vehicles_.count(tid) && vehicles_.at(tid).active) {
                    const auto& tv = vehicles_.at(tid);
                    return (tv.pos.x >= x_min && tv.pos.x <= x_max && tv.pos.y >= y_min && tv.pos.y <= y_max);
                }
                return false;
            };
            
            // (구: 03 -> 신: 06), (구: 04 -> 신: 07)
            if (is_target_in_box("CAV_06")) {
                if (my_cav.id == "CAV_01" && am_i_top) return true;
                if (my_cav.id == "CAV_04" && !am_i_right) return true;
            }
            if (is_target_in_box("CAV_07")) {
                if (my_cav.id == "CAV_01" && !am_i_top) return true;
                if (my_cav.id == "CAV_04" && am_i_right) return true;
            }
        }

        int count_top = 0, count_bottom = 0;
        for (const auto& [id, v] : vehicles_) {
            if (!v.active) continue;
            if (v.pos.x >= x_min && v.pos.x <= x_max && v.pos.y >= y_min && v.pos.y <= y_max) {
                if (v.pos.y > fourway_center_.y) count_top++;
                else count_bottom++;
            }
        }
        return am_i_top ? (count_top >= 2) : (count_bottom >= 2);
    }

    bool is_in_round_zone(const Vehicle& v){ return (v.pos - round_center_).dist_Sq() <= round_app_r_sq_; }

    int count_cavs_in_roundabout() {
        int count = 0;
        double r_sq = round_radius_ * round_radius_;
        for (const auto& [id, v] : vehicles_) {
            if (v.is_cav && v.active && (v.pos - round_center_).dist_Sq() <= r_sq) count++;
        }
        return count;
    }

    // [수정] ID Swap 적용
    bool check_roundabout(const Vehicle& my_cav) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        double r_sq = round_radius_ * round_radius_;
        if (dist_sq > round_app_r_sq_ || dist_sq <= r_sq || !is_approaching(my_cav, round_center_)) return false;

        int cavs_inside = count_cavs_in_roundabout();
        if (cavs_inside >= 2) return true;

        auto is_blocked_by_hv_check = [&](const Vehicle& v) -> bool {
            double default_check_left = 2.1 / round_radius_;
            double default_check_right = 0.4 / round_radius_;
            if (v.is_stopped && v.stop_cause == "ROUND_YIELD") default_check_left += 0.5;
            double v_angle = std::atan2(v.pos.y - round_center_.y, v.pos.x - round_center_.x);

            for (const auto& [tid, target] : vehicles_) {
                if (tid == v.id || !target.active || target.is_cav) continue;
                if ((target.pos - round_center_).dist_Sq() <= r_sq){
                    double t_angle = std::atan2(target.pos.y - round_center_.y, target.pos.x - round_center_.x);
                    double diff = t_angle - v_angle;
                    while (diff > M_PI) diff -= 2.0 * M_PI;
                    while (diff < -M_PI) diff += 2.0 * M_PI;
                    if ((diff > 0 && diff < default_check_right) || (diff <= 0 && diff > -default_check_left)) return true;
                }
            }
            return false;
        };

        if (is_blocked_by_hv_check(my_cav)) return true;

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.is_cav || !target.active) continue;
            // [수정] Pair Logic: 2->4, 3->6, 4->7
            bool is_pair = ((my_cav.id == "CAV_01" && tid == "CAV_07") ||
                            (my_cav.id == "CAV_07" && tid == "CAV_01") ||
                            (my_cav.id == "CAV_04" && tid == "CAV_06") ||
                            (my_cav.id == "CAV_06" && tid == "CAV_04"));
            double t_dist_sq = (target.pos - round_center_).dist_Sq();
            if (t_dist_sq > round_app_r_sq_ || t_dist_sq <= r_sq || !is_approaching(target, round_center_)) continue;
            if (is_blocked_by_hv_check(target)) continue;
            if (is_pair) continue;
            if (t_dist_sq < dist_sq - 0.01) return true;
            if (std::abs(t_dist_sq - dist_sq) <= 0.01 && tid < my_cav.id) return true;
        }
        return false;
    }

    // [수정] ID Swap 적용
    void process_roundabout_path_decision(Vehicle& my_cav, const std::string& zone, const std::string& my_id) {
        double dist_sq = (my_cav.pos - round_center_).dist_Sq();
        if (dist_sq <= round_app_r_sq_ * 1.1 && dist_sq > (round_radius_ * round_radius_) && !my_cav.has_entered_roundabout) {
            bool go_inside = false;
            // 2->4, 3->6, 4->7
            if (my_cav.id == "CAV_01" &&
                vehicles_.count("CAV_07") &&
                vehicles_["CAV_07"].active &&
                (vehicles_["CAV_07"].pos - round_center_).dist_Sq() <= 2.5*2.5) {
                go_inside = true;
            }
            else if (my_cav.id == "CAV_04" &&
                     vehicles_.count("CAV_06") &&
                     vehicles_["CAV_06"].active &&
                     (vehicles_["CAV_06"].pos - round_center_).dist_Sq() <= 2.5*2.5) {
                go_inside = true;
            }
            
            std_msgs::msg::Bool way_msg;
            way_msg.data = go_inside;
            my_cav.pub_change_way->publish(way_msg);
            my_cav.has_entered_roundabout = true;
        }
    }

    bool is_in_danger_zone(const Vehicle& v) {
        if ((v.pos - round_center_).dist_Sq() <= round_app_r_sq_) return true;
        if ((v.pos - fourway_center_).dist_Sq() <= fourway_app_r_sq_) return true;
        if (is_in_threeway_zone(v.pos)) return true;
        return false;
    }

    bool is_parallel(const Vehicle& my_cav, const Vehicle& target){
        double dz = my_cav.z - target.z;
        while (dz > M_PI) dz -= 2.0 * M_PI;
        while (dz < -M_PI) dz += 2.0 * M_PI;
        dz = std::abs(dz);
        return (dz < 0.40) || (dz > (M_PI - 0.40));
    }

    bool is_conflict(const Vehicle& my_cav, std::string& conflict_id, std::string& log_msg, bool& deadlock_override) {
        double dist_to_round = (my_cav.pos - round_center_).dist_Sq(); 
        if (dist_to_round <= (round_radius_* round_radius_) && !is_approaching(my_cav, round_center_)) return false;

        for (const auto& [tid, target] : vehicles_) {
            if (tid == my_cav.id || !target.active) continue;
            if ((my_cav.pos - target.pos).dist_Sq() > approach_range_sq_) continue;

            auto target_box = get_vehicle_corners(target, 0.0, 0.0);
            auto full_box = get_vehicle_corners(my_cav, front_padding_, side_padding_);
            auto front_box = get_vehicle_corners(my_cav, front_padding_, 0.1);

            bool is_full = Geo::check_obb_intersection(full_box, target_box);
            bool is_front = Geo::check_obb_intersection(front_box, target_box);

            double vehicle_width = car_info[2] + car_info[3];
            double dx = target.pos.x - my_cav.pos.x;
            double dy = target.pos.y - my_cav.pos.y;
            double side_dist = std::abs(-std::sin(my_cav.z) * dx + std::cos(my_cav.z) * dy);
            bool is_parallel_result = is_parallel(my_cav, target);
            bool is_next_lane = is_parallel_result && (side_dist > vehicle_width * 1.2) && (side_dist < vehicle_width * 5.0);

            bool collision_detected = false;
            std::string temp_log = "";

            if (is_in_threeway_zone(my_cav.pos)) {
                collision_detected = check_threeway_conflict(my_cav, tid, is_next_lane, is_full, temp_log);
            }   
            else if (is_in_fourway_zone(my_cav)) {
                collision_detected = check_fourway_conflict(is_front, is_next_lane, temp_log);
            }

            if (collision_detected) {
                if (conflict_info_.count(tid) && conflict_info_[tid] == my_cav.id) {
                    deadlock_override = true;
                    continue;
                } else {
                    conflict_id = tid;
                    log_msg = temp_log;
                    return true;
                }
            }
        }
        return false;
    }

    void control_loop() {
        if (vehicles_.empty()) return;

        for (auto& [my_id, my_cav] : vehicles_) {
            if (!my_cav.is_cav || !my_cav.active) continue;

            double dist_from_center = std::sqrt((my_cav.pos - round_center_).dist_Sq());
            bool inside_roundabout = (dist_from_center <= round_radius_);
            if (my_cav.pub_is_roundabout) {
                std_msgs::msg::Bool round_msg;
                round_msg.data = inside_roundabout;
                my_cav.pub_is_roundabout->publish(round_msg);
            }

            if (!is_in_danger_zone(my_cav)) {
                handle_safe_zone_recovery(my_cav, my_id);
                continue;
            }

            bool my_cav_stop = false;
            std::string cause_id = "NONE";
            std::string log_msg = "";
            bool deadlock_active = false;
            std::string zone = is_in_threeway_zone(my_cav.pos) ? "threeway" :
                               (is_in_fourway_zone(my_cav) ? "4WAY" : "ROUND");

            if (my_cav.forced_stop) {
                my_cav_stop = true; cause_id = my_cav.stop_cause; log_msg = "FORCED";
            }
            else {
                if (is_conflict(my_cav, cause_id, log_msg, deadlock_active)) {
                    my_cav_stop = true;
                }
                else {
                    if (zone == "4WAY") {
                        if (check_fourway_rect_split(my_cav)) {
                            my_cav_stop = true; cause_id = "4WAY_BLOCK"; log_msg = "4WAY_FULL";
                        }
                    }
                    else if (zone == "ROUND") {
                        if (check_roundabout(my_cav)) {
                            my_cav_stop = true; cause_id = "ROUND_YIELD"; log_msg = "ROUND_YIELD";
                            my_cav.has_entered_roundabout = false;
                        } 
                        else {
                            process_roundabout_path_decision(my_cav, zone, my_id);
                        }
                    } else {
                        my_cav.has_entered_roundabout = false;
                    }
                }
            }

            if (!my_cav_stop) clear_conflict_status(my_id);

            if (my_cav.is_stopped != my_cav_stop) {
                if (my_cav_stop) update_conflict_status(my_id, cause_id);
            } else if (my_cav_stop) {
                update_conflict_status(my_id, cause_id);
            }

            my_cav.is_stopped = my_cav_stop;
            if (my_cav_stop && my_cav.stop_cause.empty()) my_cav.stop_cause = cause_id;
            else if (!my_cav_stop) my_cav.stop_cause.clear();

            if (my_cav.pub_stop) {
                std_msgs::msg::Bool msg; msg.data = my_cav_stop; my_cav.pub_stop->publish(msg);
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainTrafficController>());
    rclcpp::shutdown();
    return 0;
}