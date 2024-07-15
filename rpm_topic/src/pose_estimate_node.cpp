#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "pose_estimate_node");
    ros::NodeHandle nh;

    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    ros::Rate rate(5);

    int count = 0;

    while(ros::ok())
    {
            printf("first count = %d\n", count);
            rate.sleep();
        // if (count = 1)
        {
            printf("count = 1\n");
            geometry_msgs::PoseWithCovarianceStamped initial_pose_msg;

            initial_pose_msg.header.frame_id = "map";
            initial_pose_msg.pose.pose.position.x = -1.9;
            initial_pose_msg.pose.pose.position.y = -0.37;
            initial_pose_msg.pose.pose.orientation.z = 0.0;
            initial_pose_msg.pose.pose.orientation.w = 1.0;

            initial_pose_pub.publish(initial_pose_msg);
        }
        rate.sleep();
        // if (count = 2)
        {
            // printf("count = 2\n");
            ros::shutdown();
        }
        // count = count + 1;
    }
    return 0;
}

