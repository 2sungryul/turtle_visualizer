#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/opencv.hpp>
#include <deque>
#include <memory>
#include <cmath>
#include <sstream>
#include <iomanip>
#include <vector>

class TurtleVisualizerNode : public rclcpp::Node
{
public:
    TurtleVisualizerNode() : Node("turtle_visualizer_node"), current_theta_(0.0f)
    {
        // turtlesim 윈도우 크기 (기본값: 500x500)
        window_width_ = 500;
        window_height_ = 500;
        
        // 초기 위치 설정
        current_position_ = cv::Point2f(window_width_ / 2.0f, window_height_ / 2.0f);
        
        // 배경 이미지 초기화 (turtlesim과 동일한 배경색: 연한 파란색)
        canvas_ = cv::Mat(window_height_, window_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        
        // OpenCV 윈도우 생성
        //cv::namedWindow("Turtle Visualizer", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Turtle Visualizer");
                
        // /turtle1/pose 토픽 구독
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&TurtleVisualizerNode::pose_callback, this, std::placeholders::_1));

        // /desired_path 토픽 구독
        path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
            "/desired_path",
            10,
            std::bind(&TurtleVisualizerNode::path_callback, this, std::placeholders::_1));
        
        // 타이머 생성 (화면 업데이트용, 30Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&TurtleVisualizerNode::update_display, this));
        
        RCLCPP_INFO(this->get_logger(), "Turtle Tracker Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /turtle1/pose");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /desired_path");
        RCLCPP_INFO(this->get_logger(), "Press 'c' to clear trajectory, 'q' to quit");
    }

private:

    void draw_desired_path(cv::Mat& image)
    {
        if (desired_path_points_.empty())
        {
            return;
        }
        
        // 경로를 선으로 연결하여 그리기 (초록색)
        for (size_t i = 1; i < desired_path_points_.size(); ++i)
        {
            cv::line(image, 
                    desired_path_points_[i - 1], 
                    desired_path_points_[i],
                    cv::Scalar(0, 180, 0),  // 초록색
                    2, 
                    cv::LINE_AA);
        }
        
        // 각 포인트에 작은 원 그리기
        /*for (const auto& point : desired_path_points_)
        {
            cv::circle(image, point, 3, cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
        }*/
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
    
    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        // turtlesim 좌표계를 OpenCV 좌표계로 변환
        // turtlesim: (0,0)은 왼쪽 아래, (11.088889, 11.088889)는 오른쪽 위
        // OpenCV: (0,0)은 왼쪽 위
        
        float scale = window_width_ / 11.088889f;  // turtlesim의 최대 좌표
        
        cv::Point2f current_point(
            msg->x * scale,
            window_height_ - (msg->y * scale)  // Y축 반전
        );
        
        // 현재 위치와 방향각 저장
        current_position_ = current_point;
        current_theta_ = msg->theta;  // 라디안 단위
        
        // 궤적 저장
        if (!trajectory_.empty())
        {
            // 이전 점과 현재 점 사이에 선 그리기
            cv::line(canvas_, trajectory_.back(), current_point, 
                     cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        }
        
        trajectory_.push_back(current_point);
        
        // 궤적이 너무 길어지면 오래된 점 제거 (선택사항)
        if (trajectory_.size() > 10000)
        {
            trajectory_.pop_front();
        }
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        // desired_path 업데이트
        desired_path_points_.clear();
        
        // turtlesim의 좌표 범위 (0~11.088889)를 픽셀 좌표로 변환
        float scale = window_width_ / 11.088889f;
        
        for (const auto& pose_stamped : msg->poses)
        {
            // Path의 좌표를 픽셀 좌표로 변환
            // path_publisher는 픽셀 단위로 발행하므로 직접 사용
            cv::Point2f point(
                pose_stamped.pose.position.x*scale,
                window_height_ - pose_stamped.pose.position.y*scale
            );
            
            desired_path_points_.push_back(point);
        }
        
        if (!desired_path_points_.empty())
        {
            RCLCPP_INFO(this->get_logger(), 
                       "Received desired path with %zu points", 
                       desired_path_points_.size());
        }
    }    
    
    void update_display()
    {
        cv::Mat display = canvas_.clone();

        // Desired path 그리기 (초록색 원)
        draw_desired_path(display);
        
        // 현재 거북이 위치에 원과 화살표 그리기
        if (!trajectory_.empty())
        {
            // 거북이 몸체 (원)
            cv::circle(display, current_position_, 8, 
                      cv::Scalar(0, 255, 0), -1, cv::LINE_AA);
            cv::circle(display, current_position_, 8, 
                      cv::Scalar(0, 0, 0), 2, cv::LINE_AA);
            
            // 방향 화살표 그리기
            draw_direction_arrow(display, current_position_, current_theta_);
        }
        
        // 정보 표시
        /*
        std::string info = "Points: " + std::to_string(trajectory_.size());
        cv::putText(display, info, cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
        cv::putText(display, "Press 'c' to clear, 'q' to quit", cv::Point(10, 60),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);*/
        
        // 방향각 정보 표시
        /*if (!trajectory_.empty())
        {
            std::stringstream ss;
            ss << "Angle: " << std::fixed << std::setprecision(2) 
               << (current_theta_ * 180.0 / M_PI) << " deg";
            cv::putText(display, ss.str(), cv::Point(10, 90),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        }*/
        
        // Desired path 정보 표시
        /*
        if (!desired_path_points_.empty())
        {
            std::string path_info = "Path points: " + std::to_string(desired_path_points_.size());
            cv::putText(display, path_info, cv::Point(10, 120),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 100, 0), 1);
        }*/

         // 범례 표시
        /*cv::putText(display, "Red: Turtle trajectory", cv::Point(10, window_height_ - 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(display, "Green: Desired path", cv::Point(10, window_height_ - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 180, 0), 1);*/
        cv::putText(display, "Red: Turtle trajectory", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
        cv::putText(display, "Green: Desired path", cv::Point(10, 50),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 180, 0), 1);


        // 화면 표시
        cv::imshow("Turtle Visualizer", display);
        
        // 키 입력 처리
        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            RCLCPP_INFO(this->get_logger(), "Quitting...");
            rclcpp::shutdown();
        }
        else if (key == 'c' || key == 'C')
        {
            RCLCPP_INFO(this->get_logger(), "Clearing trajectory");
            trajectory_.clear();
            canvas_ = cv::Mat(window_height_, window_width_, CV_8UC3, 
                            cv::Scalar(255, 200, 150));
        }
    }
    
    // 멤버 변수
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    cv::Mat canvas_;
    std::deque<cv::Point2f> trajectory_;
     std::vector<cv::Point2f> desired_path_points_;
    
    int window_width_;
    int window_height_;
    
    // 현재 거북이 상태
    cv::Point2f current_position_;
    float current_theta_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<TurtleVisualizerNode>();
    
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    
    cv::destroyAllWindows();
    rclcpp::shutdown();
    
    return 0;
}
