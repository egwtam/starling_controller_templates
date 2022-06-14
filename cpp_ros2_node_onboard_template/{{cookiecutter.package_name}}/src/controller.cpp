/*
 * Starling Template
 * Copyright (C) 2021 University of Bristol
 *
 * Author: Mickey Li <mickey.li@bristol.ac.uk> (University of Bristol)
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "controller.hpp"
#include "main.hpp"

UserController::UserController(UAVController *node) {
    // Set node so this controller can refer back to it later
    this->node = node;

    // Initialise publishers
    this->notify_angle_pub = this->node->create_publisher<{{cookiecutter.custom_ros2_msgs_name}}::msg::TargetAngle>("/monitor/notify_angle", 1);

    // Initialise Subscribers
    this->sync_angle_sub =  this->node->create_subscription<{{cookiecutter.custom_ros2_msgs_name}}::msg::TargetAngle>(
        "sync_angle", 10,
        std::bind(&UserController::handleTargetAngle, this, std::placeholders::_1));
    
    this->notify_vehicles_sub = this->node->create_subscription<{{cookiecutter.custom_ros2_msgs_name}}::msg::NotifyVehicles>(
        "notify_vehicles", 10,
        std::bind(&UserController::handleNotifyVehicles, this, std::placeholders::_1));

    // Variables
    this->origin = geometry_msgs::msg::Point();
}

void UserController::reset() {
    this->system_vehicle_id = 0;
}

bool UserController::smGoToStart(const rclcpp::Time& stamp) {
    return false;
}

bool UserController::smExecute(const rclcpp::Time& stamp, const rclcpp::Duration& time_elapsed) {
    return false;
}

void UserController::handleTargetAngle(const {{cookiecutter.custom_ros2_msgs_name}}::msg::TargetAngle::SharedPtr s) {

}

void UserController::handleNotifyVehicles(const {{cookiecutter.custom_ros2_msgs_name}}::msg::NotifyVehicles::SharedPtr s) {

    if(!this->node->start_trajectory_location) {
        this->system_vehicle_id = s->vehicle_id;

        double start_target_theta = 2 * M_PI * s->vehicle_id / s->total_vehicles;
        double startx = this->circle_radius * cos(start_target_theta);
        double starty = this->circle_radius * sin(start_target_theta);
        double startz = this->height;
        double startyaw = start_target_theta;

        auto loc = std::make_shared<trajectory_msgs::msg::JointTrajectoryPoint>();
        std::vector<double> pos = {startx, starty, startz, startyaw};
        loc->positions = pos;
        this->node->start_trajectory_location = loc;
    }

}

rclcpp::Logger UserController::get_logger() {return this->node->get_logger();}
rclcpp::Time UserController::now() {return this->node->now();}

