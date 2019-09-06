#include <cstdio>
#include <cstring>
#include <inttypes.h>
#include <iostream>
#include <memory>

#include <custom/srv/transform.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros_own/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

#define _USE_MATH_DEFINES

using Transform = custom::srv::Transform;
rclcpp::Node::SharedPtr node = nullptr;

class echoListener {
public:
    tf2_ros_own::Buffer buffer_;
    std::shared_ptr<tf2_ros_own::TransformListener> tfl_;

    //constructor with name
    echoListener(rclcpp::Clock::SharedPtr clock) : buffer_(clock) {
        tfl_ = std::make_shared<tf2_ros_own::TransformListener>(buffer_);
    }

    ~echoListener() {}
};

void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<Transform::Request> request,
    const std::shared_ptr<Transform::Response> response
) {
    std::cout << "aaaaaa\n";
    (void)request_header;
    (void)request;
    (void)response;
    RCLCPP_INFO(
        node->get_logger(),
        "request: received"
    );

    rclcpp::Clock::SharedPtr clock = node->get_clock();
    echoListener echoListener(clock);
    // Wait for up to one second for the first transforms to become avaiable. 
    echoListener.buffer_.canTransform(request->source_frame, request->target_frame, tf2::TimePoint(), tf2::durationFromSec(1.0));

    try {
        geometry_msgs::msg::TransformStamped echo_transform;
        echo_transform = echoListener.buffer_.lookupTransform(request->source_frame, request->target_frame, tf2::TimePoint());
        std::cout << "At time " << echo_transform.header.stamp.sec << "." << echo_transform.header.stamp.nanosec << std::endl;

        auto translation = echo_transform.transform.translation;
        auto rotation = echo_transform.transform.rotation;
        response->x = translation.x;
        response->y = translation.y;
        response->z = translation.z;
        std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << std::endl;
        std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " 
                            << rotation.z << ", " << rotation.w << "]" << std::endl;
    }
    catch(tf2::TransformException& ex) {
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        
    }
}

int main(int argc, char ** argv) {
    std::cout << "elo melo\n";
    //Initialize ROS
    rclcpp::init(argc, argv);

//     // Allow 2 or 3 command line arguments
//     if (argc < 3 || argc > 4) {
//         printf("Usage: tf2_echo source_frame target_frame [echo_rate]\n\n");
//         printf("This will echo the transform from the coordinate frame of the source_frame\n");
//         printf("to the coordinate frame of the target_frame. \n");
//         printf("Note: This is the transform to get data from target_frame into the source_frame.\n");
//         printf("Default echo rate is 1 if echo_rate is not given.\n");
//         return -1;
//     }

    node = rclcpp::Node::make_shared("transform_service");
    auto server = node->create_service<Transform>("transform_srv", handle_service);
    rclcpp::spin(node);
    rclcpp::shutdown();
    node = nullptr;
    return 0;

//     rclcpp::Clock::SharedPtr clock = node->get_clock();
//     //Instantiate a local listener
//     echoListener echoListener(clock);


//     std::string source_frameid = std::string(argv[1]);
//     std::string target_frameid = std::string(argv[2]);

//     }
}
