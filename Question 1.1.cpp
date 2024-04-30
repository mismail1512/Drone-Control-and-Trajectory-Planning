
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/State.h>

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

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose_takeoff;
    pose_takeoff.pose.position.x = 0;
    pose_takeoff.pose.position.y = 0;
    pose_takeoff.pose.position.z = 5;

    geometry_msgs::PoseStamped pose_waypoint1;
    pose_waypoint1.pose.position.x = 2;
    pose_waypoint1.pose.position.y = 2;
    pose_waypoint1.pose.position.z = 3;

    geometry_msgs::PoseStamped pose_waypoint2;
    pose_waypoint2.pose.position.x = 4;
    pose_waypoint2.pose.position.y = 6;
    pose_waypoint2.pose.position.z = 8;

    geometry_msgs::PoseStamped pose_waypoint3;
    pose_waypoint3.pose.position.x = -3;
    pose_waypoint3.pose.position.y = -4;
    pose_waypoint3.pose.position.z = 7;

    geometry_msgs::PoseStamped pose_waypoint4;
    pose_waypoint4.pose.position.x = 2;
    pose_waypoint4.pose.position.y = 1;
    pose_waypoint4.pose.position.z = 3;

    geometry_msgs::PoseStamped pose_waypoint5;
    pose_waypoint5.pose.position.x = 1;
    pose_waypoint5.pose.position.y = 4;
    pose_waypoint5.pose.position.z = 5;

    geometry_msgs::PoseStamped pose_home;
    pose_home.pose.position.x = 0;
    pose_home.pose.position.y = 0;
    pose_home.pose.position.z = 0;

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

    enum State { TAKEOFF, WAYPOINT1, WAYPOINT2, WAYPOINT3, WAYPOINT4, WAYPOINT5, GO_HOME, LAND };
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
                        ROS_INFO("Reached takeoff position");
                        state = WAYPOINT1;
                    }
                }
                break;

            case WAYPOINT1:
                local_pos_pub.publish(pose_waypoint1);
                if (is_at_position(current_pose, pose_waypoint1)) {
                    state = WAYPOINT2;
                    ROS_INFO("Reached Waypoint 1: (2,2,3)");
                }
                break;

            case WAYPOINT2:
                local_pos_pub.publish(pose_waypoint2);
                if (is_at_position(current_pose, pose_waypoint2)) {
                    state = WAYPOINT3;
                    ROS_INFO("Reached Waypoint 2: (4,6,8)");
                }
                break;

            case WAYPOINT3:
                local_pos_pub.publish(pose_waypoint3);
                if (is_at_position(current_pose, pose_waypoint3)) {
                    state = WAYPOINT4;
                    ROS_INFO("Reached Waypoint 3: (-3,-4,7)");
                }
                break;

            case WAYPOINT4:
                local_pos_pub.publish(pose_waypoint4);
                if (is_at_position(current_pose, pose_waypoint4)) {
                    state = WAYPOINT5;
                    ROS_INFO("Reached Waypoint 4: (2,1,3)");
                }
                break;

            case WAYPOINT5:
                local_pos_pub.publish(pose_waypoint5);
                if (is_at_position(current_pose, pose_waypoint5)) {
                    state = GO_HOME;
                    ROS_INFO("Reached Waypoint 5: (1,4,5)");
                }
                break;

            case GO_HOME:
                local_pos_pub.publish(pose_home);
                if (is_at_position(current_pose, pose_home, 1.0)) { // Increased tolerance for ground position
                    state = LAND;
                    ROS_INFO("Reached home position, preparing to land.");
                }
                break;

            case LAND:
                ROS_INFO("Landing at home position.");
                mavros_msgs::CommandTOL land_cmd;
                land_cmd.request.yaw = 0;
                land_cmd.request.latitude = 0;
                land_cmd.request.longitude = 0;
                land_cmd.request.altitude = 0;

                if (land_client.call(land_cmd) and land_cmd.response.success) {
                    ROS_INFO("Landing command sent successfully.");
                    return 0; // Exit after landing
                } else {
                    ROS_ERROR("Failed to send landing command.");
                }
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
