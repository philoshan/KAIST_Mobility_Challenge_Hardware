#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp> // [변경] Accel -> Twist
#include <std_msgs/msg/bool.hpp> 
#include <std_msgs/msg/float32.hpp> 

#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <algorithm>
#include <limits>

using std::placeholders::_1;

struct Point {
    double x;
    double y;
};

class RealStanleyDriver : public rclcpp::Node
{
public:
    RealStanleyDriver()
    : Node("real_stanley_driver")
    {
        // QoS 설정: SensorData (Best Effort, Volatile)
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_profile.best_effort();
        qos_profile.durability_volatile();

        // [파라미터 설정] 요청하신 코드 원본 값 그대로 유지
        this->declare_parameter("original_way_path", "tool/cav1p3.csv");
        this->declare_parameter("inside_way_path", "tool/cav1p3_inside.csv");
        
        this->declare_parameter("k_gain", 2.0);          // 원본 유지
        this->declare_parameter("max_steer", 0.7);       // 원본 유지
        this->declare_parameter("target_speed", 2.0);    // 원본 유지
        this->declare_parameter("center_to_front", 0.17);// 원본 유지
        this->declare_parameter("wheelbase", 0.33);      // 원본 유지
        this->declare_parameter("steer_gain", 1.0);      // 원본 유지
        this->declare_parameter("forward_step", 15);     // 원본 유지
        
        this->declare_parameter("warmup_steps", 10);     // 원본 유지

        // 파라미터 로드
        original_csv_path_ = this->get_parameter("original_way_path").as_string();
        inside_csv_path_ = this->get_parameter("inside_way_path").as_string();
        
        k_gain_ = this->get_parameter("k_gain").as_double();
        max_steer_ = this->get_parameter("max_steer").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        center_to_front_ = this->get_parameter("center_to_front").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        steer_gain_ = this->get_parameter("steer_gain").as_double();
        forward_step_ = this->get_parameter("forward_step").as_int(); 
        warmup_steps_target_ = this->get_parameter("warmup_steps").as_int();

        // 경로 파일 로딩
        load_waypoints(original_csv_path_, waypoints_original_);
        load_waypoints(inside_csv_path_, waypoints_inside_);
        
        current_waypoints_ = &waypoints_original_;
        is_inside_path_active_ = false;

        // Subscriber: /Ego_pose
        sub_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/Ego_pose", qos_profile, std::bind(&RealStanleyDriver::pose_callback, this, _1));
        
        // Publisher: [변경] /Accel -> /cmd_vel (Twist)
        // 하드웨어 제어를 위해 메시지 타입만 변경
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 기타 제어 토픽들 (원본 유지)
        sub_stop_cmd_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_stop", qos_profile, 
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                this->stop_signal_ = msg->data; 
                if (this->stop_signal_) this->publish_stop_command();
            });

        sub_change_way_ = this->create_subscription<std_msgs::msg::Bool>(
            "change_waypoint", qos_profile,
            std::bind(&RealStanleyDriver::callback_change_waypoint, this, _1));

        sub_hv_vel_ = this->create_subscription<std_msgs::msg::Float32>(
            "hv_vel", qos_profile,
            [this](const std_msgs::msg::Float32::SharedPtr msg) { this->hv_vel_ = msg->data; });

        sub_is_roundabout_ = this->create_subscription<std_msgs::msg::Bool>(
            "is_roundabout", qos_profile,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { this->is_roundabout_ = msg->data; });
            
        RCLCPP_INFO(this->get_logger(), "Real Driver Started. Speed: %.2f, WB: %.2f", target_speed_, wheelbase_);
    }

private:
    void load_waypoints(const std::string& path, std::vector<Point>& target_vector) {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", path.c_str());
            return;
        }
        std::string line;
        std::getline(file, line); 
        target_vector.clear();
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;
            while (std::getline(ss, cell, ',')) row.push_back(cell);
            if (row.size() >= 2) {
                try {
                    Point p = {std::stod(row[0]), std::stod(row[1])};
                    target_vector.push_back(p);
                } catch (...) { continue; }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", target_vector.size(), path.c_str());
    }

    void callback_change_waypoint(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !is_inside_path_active_ && !waypoints_inside_.empty()) {
            is_inside_path_active_ = true;
            current_waypoints_ = &waypoints_inside_;
        } 
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // [변경] Twist 메시지로 정지
    void publish_stop_command() {
        auto stop_msg = geometry_msgs::msg::Twist();
        stop_msg.linear.x = 0.0;  
        stop_msg.angular.z = 0.0; 
        pub_cmd_vel_->publish(stop_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 1. 초기 웜업
        if (current_warmup_count_ < warmup_steps_target_) {
            current_warmup_count_++;
            publish_stop_command();
            return;
        }

        // 2. 비상 정지
        if (stop_signal_) {
            publish_stop_command();
            return; 
        }

        if (current_waypoints_->empty()) return;

        // --- Stanley Logic ---

        // 1. 차량 상태 추출 (z값 직접 사용 - 파이썬 테스트 결과 반영)
        double center_x = msg->pose.position.x;
        double center_y = msg->pose.position.y;
        double current_yaw = msg->pose.orientation.z;

        // 2. 전륜 위치 계산
        double front_x = center_x + center_to_front_ * std::cos(current_yaw);
        double front_y = center_y + center_to_front_ * std::sin(current_yaw);

        // 3. Nearest Waypoint Search
        int nearest_idx = -1;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_waypoints_->size(); ++i) {
            double dx = front_x - (*current_waypoints_)[i].x;
            double dy = front_y - (*current_waypoints_)[i].y;
            double dist = std::hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // 4. 경로 복귀 로직
        if (is_inside_path_active_) {
            if (nearest_idx >= (int)current_waypoints_->size() - 5) { 
                is_inside_path_active_ = false;
                current_waypoints_ = &waypoints_original_;
                return; 
            }
        }

        // 5. CTE 계산
        int next_nearest_idx = (nearest_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_nearest_idx == 0) next_nearest_idx = nearest_idx;

        double map_x = (*current_waypoints_)[nearest_idx].x;
        double map_y = (*current_waypoints_)[nearest_idx].y;
        double next_map_x = (*current_waypoints_)[next_nearest_idx].x;
        double next_map_y = (*current_waypoints_)[next_nearest_idx].y;

        double path_dx = next_map_x - map_x;
        double path_dy = next_map_y - map_y;
        double path_len = std::hypot(path_dx, path_dy);
        if (path_len < 1e-6) path_len = 1e-6;

        double dx = front_x - map_x;
        double dy = front_y - map_y;
        double cross_product = dx * path_dy - dy * path_dx; 
        double cte = cross_product / path_len;

        // 6. Heading Error 계산
        int target_idx = (nearest_idx + forward_step_) % current_waypoints_->size();
        
        if (is_inside_path_active_ && (nearest_idx + forward_step_) >= (int)current_waypoints_->size()) {
            target_idx = current_waypoints_->size() - 1;
        }

        int next_target_idx = (target_idx + 1) % current_waypoints_->size();
        if (is_inside_path_active_ && next_target_idx == 0) next_target_idx = target_idx; 
        
        double target_dx = (*current_waypoints_)[next_target_idx].x - (*current_waypoints_)[target_idx].x;
        double target_dy = (*current_waypoints_)[next_target_idx].y - (*current_waypoints_)[target_idx].y;
        
        if (std::hypot(target_dx, target_dy) < 1e-6 && target_idx > 0) {
             target_dx = (*current_waypoints_)[target_idx].x - (*current_waypoints_)[target_idx-1].x;
             target_dy = (*current_waypoints_)[target_idx].y - (*current_waypoints_)[target_idx-1].y;
        }

        double path_yaw = std::atan2(target_dy, target_dx);
        double heading_error = normalize_angle(path_yaw - current_yaw);

        // 7. 속도 결정
        double final_speed = target_speed_;
        if (is_roundabout_) final_speed = std::max((double)hv_vel_, 0.0);

        // 8. Stanley Control Law
        double v_clamped = std::max(final_speed, 0.1); 
        double cte_correction = std::atan2(k_gain_ * cte, v_clamped);

        double steer_angle = heading_error + cte_correction; 
        steer_angle = normalize_angle(steer_angle);
        steer_angle *= steer_gain_;
        steer_angle = std::max(-max_steer_, std::min(max_steer_, steer_angle));

        // 9. 최종 발행 [변경] Accel -> Twist
        // Hardware interface expects Twist msg
        auto msg_out = geometry_msgs::msg::Twist();
        
        // 각속도 계산 (omega = v / L * tan(delta))
        double yaw_rate = (final_speed / wheelbase_) * std::tan(steer_angle);

        msg_out.linear.x = final_speed;
        msg_out.angular.z = yaw_rate;

        pub_cmd_vel_->publish(msg_out);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; // [변경]
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stop_cmd_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_change_way_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_hv_vel_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_is_roundabout_;

    bool stop_signal_ = false;
    
    std::vector<Point> waypoints_original_;
    std::vector<Point> waypoints_inside_;
    std::vector<Point>* current_waypoints_; 

    std::string original_csv_path_;
    std::string inside_csv_path_;
    
    double k_gain_;
    double max_steer_;
    double target_speed_;
    double center_to_front_;
    double wheelbase_;
    double steer_gain_;
    int forward_step_;

    int warmup_steps_target_;
    int current_warmup_count_ = 0;

    bool is_inside_path_active_ = false; 

    float hv_vel_ = 0.0f;
    bool is_roundabout_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RealStanleyDriver>());
    rclcpp::shutdown();
    return 0;
}