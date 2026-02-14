#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
//#include <tf2/LinearMath/Quaternion.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class TurtleVisualizerWithTwoPath : public rclcpp::Node
{
public:
    TurtleVisualizerWithTwoPath() : Node("turtle_visualizer_with_two_path")
    {
        // 파라미터 선언
        this->declare_parameter("window_size", 500);
        this->declare_parameter("world_size", 11.088889);
        this->declare_parameter("background_color_r", 245);
        this->declare_parameter("background_color_g", 245);
        this->declare_parameter("background_color_b", 245);
        this->declare_parameter("turtle_color_r", 69);
        this->declare_parameter("turtle_color_g", 255);
        this->declare_parameter("turtle_color_b", 50);
        this->declare_parameter("path_color_r", 255);
        this->declare_parameter("path_color_g", 0);
        this->declare_parameter("path_color_b", 0);
        this->declare_parameter("plan1_color_r", 255);
        this->declare_parameter("plan1_color_g", 0);
        this->declare_parameter("plan1_color_b", 0);
        this->declare_parameter("plan2_color_r", 0);
        this->declare_parameter("plan2_color_g", 0);
        this->declare_parameter("plan2_color_b", 255);
        this->declare_parameter("turtle_size", 10);
        this->declare_parameter("path_thickness", 2);
        this->declare_parameter("trail_length", 500);
        
        // 파라미터 가져오기
        window_size_ = this->get_parameter("window_size").as_int();
        world_size_ = this->get_parameter("world_size").as_double();
        bg_color_ = cv::Scalar(
            this->get_parameter("background_color_b").as_int(),
            this->get_parameter("background_color_g").as_int(),
            this->get_parameter("background_color_r").as_int()
        );
        turtle_color_ = cv::Scalar(
            this->get_parameter("turtle_color_b").as_int(),
            this->get_parameter("turtle_color_g").as_int(),
            this->get_parameter("turtle_color_r").as_int()
        );
        path_color_ = cv::Scalar(
            this->get_parameter("path_color_b").as_int(),
            this->get_parameter("path_color_g").as_int(),
            this->get_parameter("path_color_r").as_int()
        );
        plan1_color_ = cv::Scalar(
            this->get_parameter("plan1_color_b").as_int(),
            this->get_parameter("plan1_color_g").as_int(),
            this->get_parameter("plan1_color_r").as_int()
        );
        plan2_color_ = cv::Scalar(
            this->get_parameter("plan2_color_b").as_int(),
            this->get_parameter("plan2_color_g").as_int(),
            this->get_parameter("plan2_color_r").as_int()
        );
        turtle_size_ = this->get_parameter("turtle_size").as_int();
        path_thickness_ = this->get_parameter("path_thickness").as_int();
        trail_length_ = this->get_parameter("trail_length").as_int();
        
        // 이미지 초기화
        canvas_ = cv::Mat(window_size_, window_size_, CV_8UC3, bg_color_);
        
        // Subscribers
        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleVisualizerWithTwoPath::pose_callback, this, std::placeholders::_1));
        
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/desired_path", 10,
            std::bind(&TurtleVisualizerWithTwoPath::path_callback, this, std::placeholders::_1));
        
        // 추가: Plan1 경로 구독
        plan1_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan1/desired_path", 10,
            std::bind(&TurtleVisualizerWithTwoPath::plan1_callback, this, std::placeholders::_1));
        
        // 추가: Plan2 경로 구독
        plan2_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan2/desired_path", 10,
            std::bind(&TurtleVisualizerWithTwoPath::plan2_callback, this, std::placeholders::_1));
        
        // Timer for visualization update
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&TurtleVisualizerWithTwoPath::update_visualization, this));
        
        RCLCPP_INFO(this->get_logger(), "Turtle Visualizer with Two Paths Started");
        RCLCPP_INFO(this->get_logger(), "  Window size: %d", window_size_);
        RCLCPP_INFO(this->get_logger(), "  World size: %.2f", world_size_);
        RCLCPP_INFO(this->get_logger(), "  Plan1 color: RGB(%d, %d, %d)", 
                    this->get_parameter("plan1_color_r").as_int(),
                    this->get_parameter("plan1_color_g").as_int(),
                    this->get_parameter("plan1_color_b").as_int());
        RCLCPP_INFO(this->get_logger(), "  Plan2 color: RGB(%d, %d, %d)",
                    this->get_parameter("plan2_color_r").as_int(),
                    this->get_parameter("plan2_color_g").as_int(),
                    this->get_parameter("plan2_color_b").as_int());
    }

private:
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
        has_pose_ = true;
        
        // 거북이 궤적 저장
        cv::Point pt = world_to_pixel(msg->x, msg->y);
        turtle_trail_.push_back(pt);
        
        // 궤적 길이 제한
        if (turtle_trail_.size() > static_cast<size_t>(trail_length_))
        {
            turtle_trail_.erase(turtle_trail_.begin());
        }
    }
    
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        desired_path_ = *msg;
        has_path_ = true;
    }
    
    // 추가: Plan1 경로 콜백
    void plan1_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        plan1_path_ = *msg;
        has_plan1_ = true;
    }
    
    // 추가: Plan2 경로 콜백
    void plan2_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        plan2_path_ = *msg;
        has_plan2_ = true;
    }
    
    cv::Point world_to_pixel(double x, double y)
    {
        int px = static_cast<int>((x / world_size_) * window_size_);
        int py = static_cast<int>(((world_size_ - y) / world_size_) * window_size_);
        return cv::Point(px, py);
    }
    
    void draw_turtle(cv::Mat& img, const turtlesim::msg::Pose& pose)
    {
        cv::Point center = world_to_pixel(pose.x, pose.y);
        
        // 거북이 몸통 (원)
        //cv::circle(img, center, turtle_size_, turtle_color_, -1);
        
        // 거북이 몸체 (원)
        cv::circle(img, center, 8, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        cv::circle(img, center, 8, cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
        
        // 거북이 방향 표시 (화살표)
        /*double arrow_length = turtle_size_ * 1.5;
        cv::Point arrow_end(
            center.x + arrow_length * std::cos(pose.theta),
            center.y - arrow_length * std::sin(pose.theta)
        );
        cv::arrowedLine(img, center, arrow_end, cv::Scalar(0, 0, 0), 2);*/
        
        // 방향 화살표 그리기
        draw_direction_arrow(img, center, pose.theta);
    }
    
    void draw_direction_arrow(cv::Mat& image, const cv::Point2f& position, float theta)
    {
        // 화살표 길이
        const float arrow_length = 25.0f;
        const float arrow_head_length = 10.0f;
        const float arrow_head_angle = M_PI / 6.0f;  // 30도
        
        // OpenCV 좌표계에서는 Y축이 반대이므로 각도 조정
        // turtlesim: 0도는 오른쪽(동쪽), 반시계방향 증가
        // OpenCV: Y축이 아래쪽이므로 각도 반전 필요
        float opencv_theta = -theta;
        
        // 화살표 끝점 계산
        cv::Point2f arrow_end(
            position.x + arrow_length * cos(opencv_theta),
            position.y + arrow_length * sin(opencv_theta)
        );
        
        // 메인 화살표 선 그리기
        cv::arrowedLine(image, position, arrow_end, 
                       cv::Scalar(255, 0, 0), 1, cv::LINE_AA, 0, 0.3);
        
        // 더 명확한 화살표 머리 그리기 (삼각형)
        cv::Point2f arrow_head_point1(
            arrow_end.x - arrow_head_length * cos(opencv_theta - arrow_head_angle),
            arrow_end.y - arrow_head_length * sin(opencv_theta - arrow_head_angle)
        );
        
        cv::Point2f arrow_head_point2(
            arrow_end.x - arrow_head_length * cos(opencv_theta + arrow_head_angle),
            arrow_end.y - arrow_head_length * sin(opencv_theta + arrow_head_angle)
        );
        
        // 화살표 머리 채우기
        /*std::vector<cv::Point> arrow_head_points = {
            cv::Point(static_cast<int>(arrow_end.x), static_cast<int>(arrow_end.y)),
            cv::Point(static_cast<int>(arrow_head_point1.x), static_cast<int>(arrow_head_point1.y)),
            cv::Point(static_cast<int>(arrow_head_point2.x), static_cast<int>(arrow_head_point2.y))
        };
        
        cv::fillConvexPoly(image, arrow_head_points, cv::Scalar(255, 0, 0), cv::LINE_AA);*/
    }

    void draw_path(cv::Mat& img, const nav_msgs::msg::Path& path, const cv::Scalar& color)
    {
        if (path.poses.size() < 2) return;
        
        for (size_t i = 0; i < path.poses.size() - 1; ++i)
        {
            cv::Point p1 = world_to_pixel(
                path.poses[i].pose.position.x,
                path.poses[i].pose.position.y
            );
            cv::Point p2 = world_to_pixel(
                path.poses[i + 1].pose.position.x,
                path.poses[i + 1].pose.position.y
            );
            
            cv::line(img, p1, p2, color, path_thickness_);
        }
    }
    
    void draw_trail(cv::Mat& img)
    {
        if (turtle_trail_.size() < 2) return;
        
        for (size_t i = 0; i < turtle_trail_.size() - 1; ++i)
        {
            // 투명도 효과 (오래된 궤적일수록 연하게)
            double alpha = static_cast<double>(i) / turtle_trail_.size();
            cv::Scalar color(
                turtle_color_[0] * alpha,
                turtle_color_[1] * alpha,
                turtle_color_[2] * alpha
            );
            
            cv::line(img, turtle_trail_[i], turtle_trail_[i + 1], color, 2);
        }
    }
    
    void update_visualization()
    {
        // 캔버스 초기화
        canvas_ = cv::Mat(window_size_, window_size_, CV_8UC3, bg_color_);
        
        // 격자 그리기
        //draw_grid(canvas_);
        
        // 추가: Plan1 경로 그리기 (빨간색)
        if (has_plan1_)
        {
            draw_path(canvas_, plan1_path_, plan1_color_);
        }
        
        // 추가: Plan2 경로 그리기 (파란색)
        if (has_plan2_)
        {
            draw_path(canvas_, plan2_path_, plan2_color_);
        }
        
        // 기존: Desired path 그리기 (원래 색상)
        if (has_path_)
        {
            draw_path(canvas_, desired_path_, path_color_);
        }
        
        // 거북이 궤적 그리기
        draw_trail(canvas_);
        
        // 거북이 그리기
        if (has_pose_)
        {
            draw_turtle(canvas_, current_pose_);
            
            // 정보 표시
            /*
            std::string info = "Turtle: (" + 
                             std::to_string(static_cast<int>(current_pose_.x * 10) / 10.0) + ", " +
                             std::to_string(static_cast<int>(current_pose_.y * 10) / 10.0) + ")";
            cv::putText(canvas_, info, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
            
            // 추가: 경로 정보 표시
            if (has_plan1_)
            {
                cv::putText(canvas_, "Plan1 (Red)", cv::Point(10, 60),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, plan1_color_, 2);
            }
            if (has_plan2_)
            {
                cv::putText(canvas_, "Plan2 (Blue)", cv::Point(10, 85),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, plan2_color_, 2);
            }*/
        }
        
        // 이미지 표시
        cv::imshow("Turtle Visualizer with Two Paths", canvas_);
        cv::waitKey(1);
    }
    
    void draw_grid(cv::Mat& img)
    {
        // 격자선 그리기
        cv::Scalar grid_color(200, 200, 200);
        int grid_spacing = window_size_ / 11;
        
        for (int i = 0; i <= 11; ++i)
        {
            int pos = i * grid_spacing;
            
            // 세로선
            cv::line(img, cv::Point(pos, 0), cv::Point(pos, window_size_), 
                    grid_color, 1);
            
            // 가로선
            cv::line(img, cv::Point(0, pos), cv::Point(window_size_, pos), 
                    grid_color, 1);
        }
    }
    
    // 멤버 변수
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan1_sub_;  // 추가
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan2_sub_;  // 추가
    rclcpp::TimerBase::SharedPtr timer_;
    
    turtlesim::msg::Pose current_pose_;
    nav_msgs::msg::Path desired_path_;
    nav_msgs::msg::Path plan1_path_;  // 추가
    nav_msgs::msg::Path plan2_path_;  // 추가
    
    bool has_pose_ = false;
    bool has_path_ = false;
    bool has_plan1_ = false;  // 추가
    bool has_plan2_ = false;  // 추가
    
    std::vector<cv::Point> turtle_trail_;
    
    cv::Mat canvas_;
    
    int window_size_;
    double world_size_;
    cv::Scalar bg_color_;
    cv::Scalar turtle_color_;
    cv::Scalar path_color_;
    cv::Scalar plan1_color_;  // 추가
    cv::Scalar plan2_color_;  // 추가
    int turtle_size_;
    int path_thickness_;
    int trail_length_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleVisualizerWithTwoPath>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    cv::destroyAllWindows();
    
    return 0;
}