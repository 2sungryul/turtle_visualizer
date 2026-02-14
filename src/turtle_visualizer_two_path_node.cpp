#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
//#include <tf2/LinearMath/Quaternion.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

class TurtleVisualizerTwoPathNode2 : public rclcpp::Node
{
public:
    TurtleVisualizerTwoPathNode2() : Node("turtle_visualizer_two_path_node2")
    {
        // 파라미터 선언
        this->declare_parameter("window_size", 500);
        this->declare_parameter("world_size", 11.088889);
        this->declare_parameter("background_color_r", 255);
        this->declare_parameter("background_color_g", 255);
        this->declare_parameter("background_color_b", 255);
        this->declare_parameter("turtle1_color_r", 69);
        this->declare_parameter("turtle1_color_g", 255);
        this->declare_parameter("turtle1_color_b", 50);
        this->declare_parameter("turtle2_color_r", 255);
        this->declare_parameter("turtle2_color_g", 165);
        this->declare_parameter("turtle2_color_b", 0);
        this->declare_parameter("plan1_color_r", 255);
        this->declare_parameter("plan1_color_g", 0);
        this->declare_parameter("plan1_color_b", 0);
        this->declare_parameter("plan2_color_r", 0);
        this->declare_parameter("plan2_color_g", 0);
        this->declare_parameter("plan2_color_b", 255);
        this->declare_parameter("turtle_size", 10);
        this->declare_parameter("path_thickness", 2);
        this->declare_parameter("trail_length", 100);
        
        // 파라미터 가져오기
        window_size_ = this->get_parameter("window_size").as_int();
        world_size_ = this->get_parameter("world_size").as_double();
        bg_color_ = cv::Scalar(
            this->get_parameter("background_color_b").as_int(),
            this->get_parameter("background_color_g").as_int(),
            this->get_parameter("background_color_r").as_int()
        );
        turtle1_color_ = cv::Scalar(
            this->get_parameter("turtle1_color_b").as_int(),
            this->get_parameter("turtle1_color_g").as_int(),
            this->get_parameter("turtle1_color_r").as_int()
        );
        turtle2_color_ = cv::Scalar(
            this->get_parameter("turtle2_color_b").as_int(),
            this->get_parameter("turtle2_color_g").as_int(),
            this->get_parameter("turtle2_color_r").as_int()
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
        turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleVisualizerTwoPathNode2::turtle1_pose_callback, this, std::placeholders::_1));
        
        // 추가: Turtle2 pose 구독
        turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose", 10,
            std::bind(&TurtleVisualizerTwoPathNode2::turtle2_pose_callback, this, std::placeholders::_1));
        
        // Plan1 경로 구독
        plan1_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan1/desired_path", 10,
            std::bind(&TurtleVisualizerTwoPathNode2::plan1_callback, this, std::placeholders::_1));
        
        // Plan2 경로 구독
        plan2_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan2/desired_path", 10,
            std::bind(&TurtleVisualizerTwoPathNode2::plan2_callback, this, std::placeholders::_1));
        
        // Timer for visualization update
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&TurtleVisualizerTwoPathNode2::update_visualization, this));
        
        RCLCPP_INFO(this->get_logger(), "Turtle Visualizer Two Path Node2 Started");
        RCLCPP_INFO(this->get_logger(), "  Window size: %d", window_size_);
        RCLCPP_INFO(this->get_logger(), "  World size: %.2f", world_size_);
        RCLCPP_INFO(this->get_logger(), "  Turtle1 color: RGB(%d, %d, %d)", 
                    this->get_parameter("turtle1_color_r").as_int(),
                    this->get_parameter("turtle1_color_g").as_int(),
                    this->get_parameter("turtle1_color_b").as_int());
        RCLCPP_INFO(this->get_logger(), "  Turtle2 color: RGB(%d, %d, %d)", 
                    this->get_parameter("turtle2_color_r").as_int(),
                    this->get_parameter("turtle2_color_g").as_int(),
                    this->get_parameter("turtle2_color_b").as_int());
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
    void turtle1_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle1_pose_ = *msg;
        has_turtle1_pose_ = true;
        
        // 거북이1 궤적 저장
        cv::Point pt = world_to_pixel(msg->x, msg->y);
        turtle1_trail_.push_back(pt);
        
        // 궤적 길이 제한
        if (turtle1_trail_.size() > static_cast<size_t>(trail_length_))
        {
            turtle1_trail_.erase(turtle1_trail_.begin());
        }
    }
    
    // 추가: Turtle2 pose 콜백
    void turtle2_pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        turtle2_pose_ = *msg;
        has_turtle2_pose_ = true;
        
        // 거북이2 궤적 저장
        cv::Point pt = world_to_pixel(msg->x, msg->y);
        turtle2_trail_.push_back(pt);
        
        // 궤적 길이 제한
        if (turtle2_trail_.size() > static_cast<size_t>(2*trail_length_))
        {
            turtle2_trail_.erase(turtle2_trail_.begin());
        }
    }
    
    void plan1_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        plan1_path_ = *msg;
        has_plan1_ = true;
    }
    
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
    
    /*
    void draw_turtle(cv::Mat& img, const turtlesim::msg::Pose& pose, const cv::Scalar& color)
    {
        cv::Point center = world_to_pixel(pose.x, pose.y);
        
        // 거북이 몸통 (원)
        cv::circle(img, center, turtle_size_, color, -1);
        
        // 거북이 방향 표시 (화살표)
        double arrow_length = turtle_size_ * 1.5;
        cv::Point arrow_end(
            center.x + arrow_length * std::cos(pose.theta),
            center.y - arrow_length * std::sin(pose.theta)
        );
        cv::arrowedLine(img, center, arrow_end, cv::Scalar(0, 0, 0), 2);
    }*/

    void draw_turtle(cv::Mat& img, const turtlesim::msg::Pose& pose, const cv::Scalar& color)
    {
        cv::Point center = world_to_pixel(pose.x, pose.y);
        
        // 거북이 몸통 (원)
        //cv::circle(img, center, turtle_size_, turtle_color_, -1);
        
        // 거북이 몸체 (원)
        cv::circle(img, center, 8, color, -1, cv::LINE_AA);
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
    
    void draw_trail(cv::Mat& img, const std::vector<cv::Point>& trail, const cv::Scalar& color)
    {
        if (trail.size() < 2) return;
        
        for (size_t i = 0; i < trail.size() - 1; ++i)
        {
            // 투명도 효과 (오래된 궤적일수록 연하게)
            double alpha = static_cast<double>(i) / trail.size();
            cv::Scalar trail_color(
                color[0] * alpha,
                color[1] * alpha,
                color[2] * alpha
            );
            
            cv::line(img, trail[i], trail[i + 1], trail_color, 2);
        }
    }
    
    void update_visualization()
    {
        // 캔버스 초기화
        canvas_ = cv::Mat(window_size_, window_size_, CV_8UC3, bg_color_);
        
        // 격자 그리기
        //draw_grid(canvas_);
        
        // Plan1 경로 그리기 (빨간색)
        if (has_plan1_)
        {
            draw_path(canvas_, plan1_path_, plan1_color_);
        }
        
        // Plan2 경로 그리기 (파란색)
        if (has_plan2_)
        {
            draw_path(canvas_, plan2_path_, plan2_color_);
        }
        
        // 거북이1 궤적 그리기
        draw_trail(canvas_, turtle1_trail_, turtle1_color_);
        
        // 추가: 거북이2 궤적 그리기
        draw_trail(canvas_, turtle2_trail_, turtle2_color_);
        
        // 거북이1 그리기
        if (has_turtle1_pose_)
        {
            draw_turtle(canvas_, turtle1_pose_, turtle1_color_);
        }
        
        // 추가: 거북이2 그리기
        if (has_turtle2_pose_)
        {
            draw_turtle(canvas_, turtle2_pose_, turtle2_color_);
        }
        
        // 정보 표시
        /*
        int y_offset = 30;
        
        if (has_turtle1_pose_)
        {
            std::string info1 = "Turtle1: (" + 
                               std::to_string(static_cast<int>(turtle1_pose_.x * 10) / 10.0) + ", " +
                               std::to_string(static_cast<int>(turtle1_pose_.y * 10) / 10.0) + ")";
            cv::putText(canvas_, info1, cv::Point(10, y_offset), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, turtle1_color_, 2);
            y_offset += 30;
        }
        
        // 추가: Turtle2 정보 표시
        if (has_turtle2_pose_)
        {
            std::string info2 = "Turtle2: (" + 
                               std::to_string(static_cast<int>(turtle2_pose_.x * 10) / 10.0) + ", " +
                               std::to_string(static_cast<int>(turtle2_pose_.y * 10) / 10.0) + ")";
            cv::putText(canvas_, info2, cv::Point(10, y_offset), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, turtle2_color_, 2);
            y_offset += 30;
        }
        
        // 경로 정보 표시
        if (has_plan1_)
        {
            cv::putText(canvas_, "Plan1 (Red)", cv::Point(10, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, plan1_color_, 2);
            y_offset += 25;
        }
        if (has_plan2_)
        {
            cv::putText(canvas_, "Plan2 (Blue)", cv::Point(10, y_offset),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, plan2_color_, 2);
        }*/
        
        // 이미지 표시
        cv::imshow("Turtle Visualizer Two Path Node2", canvas_);
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
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;  // 추가
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan1_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan2_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    turtlesim::msg::Pose turtle1_pose_;
    turtlesim::msg::Pose turtle2_pose_;  // 추가
    nav_msgs::msg::Path plan1_path_;
    nav_msgs::msg::Path plan2_path_;
    
    bool has_turtle1_pose_ = false;
    bool has_turtle2_pose_ = false;  // 추가
    bool has_plan1_ = false;
    bool has_plan2_ = false;
    
    std::vector<cv::Point> turtle1_trail_;
    std::vector<cv::Point> turtle2_trail_;  // 추가
    
    cv::Mat canvas_;
    
    int window_size_;
    double world_size_;
    cv::Scalar bg_color_;
    cv::Scalar turtle1_color_;
    cv::Scalar turtle2_color_;  // 추가
    cv::Scalar plan1_color_;
    cv::Scalar plan2_color_;
    int turtle_size_;
    int path_thickness_;
    int trail_length_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleVisualizerTwoPathNode2>();
    
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