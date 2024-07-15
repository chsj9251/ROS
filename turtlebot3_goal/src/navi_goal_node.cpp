#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "navi_goal_node");
    ros::NodeHandle nh;

    ros::Publisher navi_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::Rate rate(5);

    int count = 0;

    while(ros::ok())
    {
        printf("first count = %d\n", count);
        rate.sleep();
        if (count = 1)
        {
            printf("count = 1\n");
            geometry_msgs::PoseStamped navi_goal_msg;

            navi_goal_msg.header.frame_id = "map";
            navi_goal_msg.pose.position.x = -1.8;
            navi_goal_msg.pose.position.y = 13.1;
            navi_goal_msg.pose.orientation.z = 1.0;

            navi_goal_pub.publish(navi_goal_msg);
        }
        rate.sleep();
        if (count = 2)
        {
            printf("count = 2\n");
            ros::shutdown();
        }
        count = count + 1;
    }
    return 0;
}