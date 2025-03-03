#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <locale.h>

int flag = 0;
const float SPEED = 7.0;
const float PAI = 3.14159;
const float Alpha = 25.0; // 俯仰角度
float s1 = 0.0;
float s2 = 0.0;
float a1 = 0.0; // z_rotator
float a2 = 0.0; // y_rotator

// 接收到订阅的消息后，会进入消息回调函数
void iat_text_Callback(const std_msgs::String::ConstPtr &msg) // eg：learning::Person::ConstPtr& msg  turtlesim::pose::ConstPtr& msg
{

    // 将接收到的消息打印出来
    ROS_INFO("\nCommands Received : %s \n", msg->data.c_str());

    std::string dataString = msg->data;
    if (dataString.find("前进") != dataString.npos)
    {
        s1 = SPEED;
        s2 = SPEED;
        flag = 1;
    }
    else if (dataString.find("后退") != dataString.npos)
    {
        s1 = -SPEED;
        s2 = -SPEED;
        flag = 2;
    }
    else if (dataString.find("左转") != dataString.npos)
    {
        s1 = 0.0;
        s2 = SPEED;
        flag = 3;
    }
    else if (dataString.find("右转") != dataString.npos)
    {
        s1 = SPEED;
        s2 = 0.0;
        flag = 4;
    }
    else if (dataString.find("停止") != dataString.npos)
    {
        s1 = 0.0;
        s2 = 0.0;
        flag = 5;
    }
    else if (dataString.find("看") != dataString.npos)
    {
        if (dataString.find("左") != dataString.npos)
        {
            a1 = PAI / 2.0;
            flag = 6;
        }
        else if (dataString.find("右") != dataString.npos)
        {
            a1 = -PAI / 2.0;
            flag = 7;
        }
        else if (dataString.find("前") != dataString.npos)
        {
            a1 = 0.0;
            flag = 8;
        }
        else if (dataString.find("后") != dataString.npos)
        {
            a1 = PAI;
            flag = 9;
        }
        else if (dataString.find("上") != dataString.npos)
        {
            a2 = -Alpha / 180.0 * PAI;
            flag = 10;
        }
        else if (dataString.find("下") != dataString.npos)
        {
            a2 = Alpha / 180.0 * PAI;
            flag = 11;
        }
        else if (dataString.find("中") != dataString.npos)
        {
            a2 = 0.0 ;
            flag = 12;
        }
        else
        {
            ROS_INFO("无效命令：%s! \n", msg->data.c_str());
        }
    }
    else
    {
        ROS_INFO("无效命令：%s! \n", msg->data.c_str());
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "zh_CN.UTF-8");

    ros::init(argc, argv, "mbot_control");
    ros::NodeHandle n;
    
    ros::Subscriber iat_text_sub = n.subscribe("/iat", 1000, iat_text_Callback);

    ros::Publisher s1_cmd_pub = n.advertise<std_msgs::Float64>("/mbot/left_wheel_joint_velocity_controller/command", 1);
    ros::Publisher s2_cmd_pub = n.advertise<std_msgs::Float64>("/mbot/right_wheel_joint_velocity_controller/command", 1);
    ros::Publisher a1_cmd_pub = n.advertise<std_msgs::Float64>("/mbot/z_rotator_joint_position_controller/command", 1);
    ros::Publisher a2_cmd_pub = n.advertise<std_msgs::Float64>("/mbot/y_rotator_joint_position_controller/command", 1);

    std_msgs::Float64 cmd1;
    std_msgs::Float64 cmd2;
    std_msgs::Float64 cmd3;
    std_msgs::Float64 cmd4;

    ros::Rate loop_rate(10);
    
    //初始修正
    cmd4.data = 0.0 ;
    a2_cmd_pub.publish(cmd4);

    while (ros::ok())
    {
        cmd1.data = s1;
        cmd2.data = s2;
        cmd3.data = a1;
        cmd4.data = a2;

        if (flag <= 5 && flag >= 1)
        {
            s1_cmd_pub.publish(cmd1);
            s2_cmd_pub.publish(cmd2);
        }
        else if (flag >= 6 && flag <= 9)
        {
            a1_cmd_pub.publish(cmd3);
        }
        else if (flag >= 10 && flag <= 12)
        {
            a2_cmd_pub.publish(cmd4);
        }
        flag = 0;

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;
}
