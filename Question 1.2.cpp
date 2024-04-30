
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose = *msg;
}

bool is_at_position(const geometry_msgs::PoseStamped& current_pose, const geometry_msgs::PoseStamped& target_pose, double tolerance = 0.1) {
    return (
        std::abs(current_pose.pose.position.x - target_pose.pose.position.x) < tolerance &&
        std::abs(current_pose.pose.position.y - target_pose.pose.position.y) < tolerance &&
        std::abs(current_pose.pose.position.z - target_pose.pose.position.z) < tolerance
    );
}

geometry_msgs::PoseStamped generate_lemniscate_pose(double RX, double RY, double altitude, double t, double T) {
    geometry_msgs::PoseStamped pose;
    double omega = 2 * M_PI / T;
    pose.pose.position.x = RX * sin(omega * t);
    pose.pose.position.y = RY * sin(omega * t) * cos(omega * t);
    pose.pose.position.z = altitude;
    return pose;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Lemniscate trajectory parameters
    double RX = 4; // X radius of lemniscate
    double RY = 4; // Y radius of lemniscate
    double altitude = 2; // Altitude
    double T = 60; // Period in seconds

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate(20.0);

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Initial setpoint for takeoff
    geometry_msgs::PoseStamped pose_takeoff;
    pose_takeoff.pose.position.x = 0;
    pose_takeoff.pose.position.y = 0;
    pose_takeoff.pose.position.z = altitude;

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose_takeoff);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    ros::Time start_time = ros::Time::now();

    enum State { TAKEOFF, FLYING_LEMNISCATE, LAND };
    State state = TAKEOFF;

    while (ros::ok()) {
        switch (state) {
            case TAKEOFF:
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                } else if (current_state.armed && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                        ROS_INFO("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    local_pos_pub.publish(pose_takeoff);
                    if (is_at_position(current_pose, pose_takeoff)) {
                        ROS_INFO("Reached takeoff position, starting lemniscate trajectory");
                        state = FLYING_LEMNISCATE;
                        start_time = ros::Time::now();
                    }
                }
                break;

            case FLYING_LEMNISCATE:
                if ((ros::Time::now() - start_time).toSec() < T) {
                    double t = (ros::Time::now() - start_time).toSec();
                    geometry_msgs::PoseStamped lemniscate_pose = generate_lemniscate_pose(RX, RY, altitude, t, T);
                    local_pos_pub.publish(lemniscate_pose);
                } else {
                    ROS_INFO("Completed lemniscate trajectory");
                    state = LAND;
                }
                break;

            case LAND:
                ROS_INFO("Landing");
                mavros_msgs::CommandTOL land_cmd;
                land_cmd.request.yaw = 0;
                land_cmd.request.latitude = 0;
                land_cmd.request.longitude = 0;
                land_cmd.request.altitude = 0;

                if (land_client.call(land_cmd) && land_cmd.response.success) {
                    ROS_INFO("Landing command sent");
                    return 0; // Exit the program after sending the landing command
                } else {
                    ROS_ERROR("Failed to send landing command");
                }
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}