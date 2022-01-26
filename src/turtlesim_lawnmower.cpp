/**
 * @file turtlesim_lawnmower.cpp
 * @author Mannat Rana (mrana8@asu.edu)
 * @brief Assignment 1 for ASU Course SES 598: Autonomous Exploration Systems
 *        Taught by Professor Jnaneshwar Das during Spring 2022
 * @version 2.0
 * @date 2022-01-20
 */

// Includes - Need ROS/Turtlesim components and some STD C++ libraries
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <turtlesim/Kill.h>
#include <turtlesim/Spawn.h>
#include "turtlesim/Pose.h"
#include <utility>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>

// Easy place to edit default node name
static const std::string node_name{"Lawnmower_Node"};

// Initialize global variables to hold parameter values
std::string turtle_name{};
std::string vel_topic{};
std::string pos_topic{};
double linear_deadband{};
double angular_deadband{};
std::vector<double> start_point{};
double height{};
double width{};
double angular_kp{};
double linear_kp{};
double window_max{};
double window_min{};
double max_lin_vel{};
double pose_hz{};

// Class to contain everything needed for turtle control
class Turtle {
private:
    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber pose_sub;
    double x;
    double y;
    double theta;
    geometry_msgs::Twist vel_msg;
    bool ready;
    ros::Rate msg_rate;

public:
    /**
     * @brief Construct a new Turtle object
     */
    Turtle() 
    : x{0}, y{0}, theta{0}, ready{false}, msg_rate{ros::Rate(pose_hz)} {
        // Kill the existing turtle
       ros::ServiceClient kill_client {nh.serviceClient<turtlesim::Kill>("/kill")};
       turtlesim::Kill::Request kill_req{};
       kill_req.name = "turtle1";
       turtlesim::Kill::Response kill_resp{};
       ros::service::waitForService("/kill");
       bool kill_success {kill_client.call(kill_req, kill_resp)};
       if (kill_success) {
           ROS_INFO_STREAM("Killed " << kill_req.name);
       }
       else {
           ROS_ERROR_STREAM("Could not kill " << kill_req.name);
       }
       
       // Spawn a turtle at the desired location
       ros::ServiceClient spawn_client {nh.serviceClient<turtlesim::Spawn>("/spawn")};
       turtlesim::Spawn::Request spawn_req{};
       spawn_req.x = start_point.at(0); spawn_req.y = start_point.at(1); 
       spawn_req.theta = start_point.at(2); spawn_req.name = turtle_name;
       turtlesim::Spawn::Response spawn_resp{};
       bool spawn_success {spawn_client.call(spawn_req, spawn_resp)};
       if (spawn_success) {
           ROS_INFO_STREAM("Spawned " << spawn_req.name << " at [" << spawn_req.x << ", " 
           << spawn_req.y << "] at " << spawn_req.theta << " radians.");
       }
       else {
           ROS_FATAL_STREAM("Could not spawn " << spawn_req.name);
       }

       // Initialize a publisher to send command velocities
       vel_pub = nh.advertise<geometry_msgs::Twist>(turtle_name + vel_topic, 1000);
       if (!vel_pub) {
           ROS_FATAL_STREAM("Failed to set up publisher on topic: " << vel_pub.getTopic());
       }
       else {
           ROS_DEBUG_STREAM("Set up publisher on topic: " << vel_pub.getTopic());
       }

       // Initialize a subscriber to the turtle's pose topic
       pose_sub = nh.subscribe(turtle_name + pos_topic, 10, &Turtle::pose_callback, this); 
       if (!pose_sub) {
           ROS_FATAL_STREAM("Failed to set up subscriber to topic: " << pose_sub.getTopic());
       }
       else {
           ROS_DEBUG_STREAM("Set up subscriber to topic: " << pose_sub.getTopic());
       }
    }

    /**
     * @brief Callback to update turtle's current pose
     * @param pose_msg 
     */
    void pose_callback(const turtlesim::Pose::ConstPtr& pose_msg) {
        // Update pose
        x = pose_msg->x;
        y = pose_msg->y;
        theta = pose_msg->theta;
        // Let program know that pose info has been recieved
        ready = true;
    }

    /**
     * @brief Return whether pose information has been recieved
     * @return true if pose message HAS been recieved 
     * @return false if pose message has NOT been recieved
     */
    bool is_ready() {
        return ready;
    }

    /**
     * @brief Publish a velocity command to send turtle straight
     *        Caps velocity if needed
     * @param lin_vel The desired command velocity
     */
    void move_straight(double lin_vel) {
        // Check to see if velocity is above cap
        if (lin_vel > max_lin_vel) {
            // Cap velocity
            lin_vel = max_lin_vel;
        }
        vel_msg.linear.x = lin_vel;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg);
    }

    /**
     * @brief Publish a velocity command to rotate turtle 
     * @param ang_vel Desired rotational velocity in rad/s
     */
    void rotate(double ang_vel) {
        vel_msg.linear.x = 0;
        vel_msg.angular.z = ang_vel;
        vel_pub.publish(vel_msg);
    }

    /**
     * @brief Send turtle to a desired setpoint
     * @param goal_x Desired x location 
     * @param goal_y Desired y location
     */
    void go_to_setpoint(double goal_x, double goal_y) {
        static int point_counter {1};
        auto setpoint {std::make_pair(goal_x, goal_y)};
        // Check if X coordinate is within window bounds - cap if necessary
        if (setpoint.first > window_max) {
            setpoint.first = window_max;
            ROS_WARN_STREAM("X Coordinate for point " << point_counter << " is outside the window, reducing it to " << window_max);
        }
        else if (setpoint.first < window_min) {
            setpoint.first = window_min;
            ROS_WARN_STREAM("X Coordinate for point " << point_counter << " is outside the window, raising it to " << window_min);
        }
        // Check if Y coordinate is within window bounds - cap if necessary
        if (setpoint.second > window_max) {
            setpoint.second = window_max;
            ROS_WARN_STREAM("Y Coordinate for point " << point_counter << " is outside the window, reducing it to " << window_max);
        }
        else if (setpoint.second < window_min) {
            setpoint.second = window_min;
            ROS_WARN_STREAM("Y Coordinate for point " << point_counter << " is outside the window, raising it to " << window_min);
        }
        
        ROS_INFO_STREAM("Going to point " << point_counter << ": [" << std::setprecision(2) << setpoint.first 
        << ", " << setpoint.second << "].");
        // Calculate the error for each state variable (x, y, theta)
        double d_x {setpoint.first - x};
        double d_y {setpoint.second - y};
        double theta_error {std::atan2(d_y, d_x) - theta};
        // Rotate while current theta is not correct
        while (std::abs(theta_error) > angular_deadband) {
            // Simple P controller for rotational velocity
            rotate(angular_kp * theta_error);
            // Continue updating theta while rotating
            theta_error = std::atan2(d_y, d_x) - theta;
            msg_rate.sleep();
            ros::spinOnce();
        }
        rotate(0);
        // Calculate distance using Pythagorean Theorem
        double distance_to_target {std::sqrt((d_x * d_x) + (d_y * d_y))};
        while(std::abs(distance_to_target) > linear_deadband) {
            // Simple P controller for linear velocity    
            move_straight(linear_kp * distance_to_target);
            // Continue updating x, y, and net distance
            d_x = setpoint.first - x;
            d_y = setpoint.second - y;
            distance_to_target = std::sqrt((d_x * d_x) + (d_y * d_y));
            msg_rate.sleep();
            ros::spinOnce();
        }
        move_straight(0);
        ROS_DEBUG_STREAM("Reached point " << point_counter << ": [" << std::setprecision(2) << setpoint.first 
        << ", " << setpoint.second << "].");
        ros::spinOnce();
        point_counter++;
    }

    /**
     * @brief Have turtle do desired lawn mowing pattern
     */
    void mow_lawn() {
        ROS_INFO("Checking for turtle pose information...");
        // Make sure turtle pose is known
        while(!is_ready()) {
            ROS_WARN_ONCE("Waiting for turtle pose information!!!");
            ros::spinOnce();
        } 
        ROS_DEBUG("Received information, beginning lawn mowing process");
        // Send setpoints for zigzag pattern
        go_to_setpoint(x + width, y);
        go_to_setpoint(x, y + height);
        go_to_setpoint(x - width, y);
        go_to_setpoint(x, y + height);
        go_to_setpoint(x + width, y);
        go_to_setpoint(start_point.at(0), start_point.at(1));
        ROS_DEBUG("Finished Lawn Mowing Path");
    }
};

int main(int argc, char** argv) {
    // Initialize Node
    ros::init(argc, argv, node_name);

    // Get all global variables from parameter server
    ros::param::get("/turtle_name", turtle_name);
    ros::param::get("/vel_topic", vel_topic);
    ros::param::get("/pos_topic", pos_topic);
    ros::param::get("/linear_deadband", linear_deadband);
    ros::param::get("/angular_deadband", angular_deadband);
    ros::param::get("/start_point", start_point);
    ros::param::get("/height", height);
    ros::param::get("/width", width);
    ros::param::get("/linear_kp", linear_kp);
    ros::param::get("/angular_kp", angular_kp);
    ros::param::get("/window_max", window_max);
    ros::param::get("/window_min", window_min);
    ros::param::get("/max_lin_vel", max_lin_vel);
    ros::param::get("/pose_hz", pose_hz);

    // Initialize turtle and complete lawn mowing process
    Turtle t = Turtle{};
    t.mow_lawn();
    ROS_DEBUG_STREAM(node_name << " has completed. Terminating Node...");
}