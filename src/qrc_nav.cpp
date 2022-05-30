#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <unistd.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double goalLocate[5][2];
bool firstRun, getHome = false;
std::string zbarCBMsg, zbarLastCBMsg = "";

void getparam(ros::NodeHandle &nh)
{
    nh.getParam("/home/x", goalLocate[0][0]);
    nh.getParam("/home/y", goalLocate[0][1]);
    nh.getParam("/Table1/x", goalLocate[1][0]);
    nh.getParam("/Table1/y", goalLocate[1][1]);
    nh.getParam("/Table2/x", goalLocate[2][0]);
    nh.getParam("/Table2/y", goalLocate[2][1]);
    nh.getParam("/Table3/x", goalLocate[3][0]);
    nh.getParam("/Table3/y", goalLocate[3][1]);
    nh.getParam("/Table4/x", goalLocate[4][0]);
    nh.getParam("/Table4/y", goalLocate[4][1]);

    printf("\n -------------------------\n home %f %f \n table1 %f %f\n table2 %f %f\n table3 %f %f\n table4 %f %f\n -------------------------\n ", goalLocate[0][0], goalLocate[0][1], goalLocate[1][0], goalLocate[1][1], goalLocate[2][0], goalLocate[2][1], goalLocate[3][0], goalLocate[3][1], goalLocate[4][0], goalLocate[4][1]);
}

int setNavTable()
{
    printf("\n---------TIRT2022--------\n|PRESSE A KEY:           |\n|1  Table1                |\n|2  Table2                |\n|3  Table3                |\n|4  Table4                |\n------------------------\n Where to go\n");

    float navTable;
    scanf("%f", &navTable);

    navTable = (int)navTable;

    while (navTable < 1 || navTable > 4)
    {
        printf("\nERR Input: %2.0f \n please Enter 1-4 ", navTable);
        scanf("%f", &navTable);
        navTable = (int)navTable;
    }

    return navTable;
}

void sendnavgoal(double navgoal[2])
{
    if (navgoal[0] == goalLocate[0][0] && navgoal[1] == goalLocate[0][1])
    {
        ROS_INFO("Go Home");
        getHome = true;
    }
    else
    {
        getHome = false;
    }

    // tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = navgoal[0];
    goal.target_pose.pose.position.y = navgoal[1];
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal x %f y %f", navgoal[0], navgoal[1]);
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Hooray, the base moved 1 meter forward");
    }
    else
    {
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    }
}

void zbarCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data != zbarLastCBMsg)
    {
        ROS_INFO("\n\n\n\nzbar callback %s\n\n\n\n", msg->data.c_str());
        zbarCBMsg = msg->data;

        if (zbarCBMsg == "none")
        {
            firstRun = false;
        }
        zbarLastCBMsg = zbarCBMsg;
        sendnavgoal(goalLocate[0]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qrc_nav");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("zbar_opencv_code", 1000, zbarCallback);

    getparam(nh);
    int navtable;

    while (ros::ok())
    {
        if (firstRun == false)
        {
            zbarLastCBMsg = "";
            navtable = setNavTable();
            sendnavgoal(goalLocate[navtable]);
            firstRun = true;
        }

        if (getHome == true)
        {
            char temp[10];
            ROS_INFO("\n\n\n\nEnter Key to continue %s\n\n\n\n", zbarCBMsg.c_str());
            scanf("%s", temp);
            if (zbarCBMsg != "none")
            {
                sendnavgoal(goalLocate[navtable]);
            }
        }
        ros::spinOnce();
    }

    return 0;
}