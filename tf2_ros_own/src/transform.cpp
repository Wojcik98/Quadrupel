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
std::unique_ptr<tf2_ros_own::Buffer> tfBuffer;
std::unique_ptr<tf2_ros_own::TransformListener> tfListener;

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

    try {
        geometry_msgs::msg::TransformStamped echo_transform;
        echo_transform = tfBuffer->lookupTransform(request->source_frame, request->target_frame, tf2::TimePoint());
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

    node = rclcpp::Node::make_shared("transform_service");
    tfBuffer.reset(new tf2_ros_own::Buffer(node->get_clock()));
    tfListener.reset(new tf2_ros_own::TransformListener(*tfBuffer));
    auto server = node->create_service<Transform>("transform_srv", handle_service);
    rclcpp::spin(node);
    rclcpp::shutdown();
    node = nullptr;
    return 0;
}
