#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <vector>
using namespace std;

int counter=1;
ros::Publisher pub;

void toDeg(double &rad){
    rad = rad * 180 / M_PI;
}

struct positions{
    double dof1, dof2, dof3, dof4, dof5=0;

};

/*  positions
{135.0, 76.0, 73.2, 90.8} deg
{135.0, 68.4, 102.0, 69.6} deg
{135.0, 69.1, 123.9, 47.1} deg
{135.0, 48.8, 98.6, 92.6} deg
{135.0, 39.1, 124.9, 76.0} deg
{135.0, 18.6, 125.2, 96.2} deg
*/

void pyramid_pos(positions &p){
    switch(counter){
     case 1:
        p.dof1=135;
        p.dof2=76;
        p.dof3=73.2;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
     case 2:
        p.dof1=135;
        p.dof2=68.4;
        p.dof3=102;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
     case 3:
        p.dof1=135;
        p.dof2=69.1;
        p.dof3=123.9;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
     case 4:
        p.dof1=135;
        p.dof2=48.8;
        p.dof3=98.6;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
     case 5:
        p.dof1=135;
        p.dof2=39.1;
        p.dof3=124.9;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
     case 6:
        p.dof1=135;
        p.dof2=18.6;
        p.dof3=125.2;
        p.dof4= 240 - (p.dof2 + p.dof3);
        p.dof5=100;
        break;
    }

}

void sureHit(positions &p){



}

//rostopic pub -1 /servo_algo std_msgs/Float64MultiArray "{data: [90 ,0 ,170, 20, 100]}"
void default_pos_close(positions &p){
    p.dof1 = 90;
    p.dof2 = 0;
    p.dof3 = 170;
    p.dof4 = 70;
    p.dof5 = 100;
}

void default_pos_open(positions &p){
    p.dof1 = 90;
    p.dof2 = 0;
    p.dof3 = 170;
    p.dof4 = 70;
    p.dof5 = 0;
}

void close(positions &p){
    p.dof5 = 100;
}

void open(positions &p){
    p.dof5 = 0;
}

positions pos;

void callback(const std_msgs::Float64MultiArray::ConstPtr &msg){


    if (msg->data.size() < 4) {
        ROS_WARN("Expected 4 angles, got %zu", msg->data.size());
        return;
    }

//store inverse kinematics positions

    double one = msg->data[0];
    double two = msg->data[1];
    double three = msg->data[2];
    double four = msg->data[3];

    toDeg(one);toDeg(two);toDeg(three);toDeg(four);

    //
    if(one >=0){
        one = 90 + one;
    }
    else if(one <=0){
        one = 90 + one;
    }//

    pos.dof1 = one;
    pos.dof2 = two;
    pos.dof3 = three;
    pos.dof4 = ((240 - (two + three)));
    if(pos.dof4<0) pos.dof4=0;
    pos.dof5 = 0;


    ROS_INFO("going: {%.1f, %.1f, %.1f, %.1f} deg", one, two, three, pos.dof4 );

    //publish
    std_msgs::Float64MultiArray out;
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);

//
    //close
    close(pos);
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);
    ROS_INFO("closing gripper...");


    //default pos close
    default_pos_close(pos);
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);
    ROS_INFO("going: {%.1f, %.1f, %.1f, %.1f} deg (default position)", pos.dof1, pos.dof2, pos.dof3, pos.dof4);

    //go to place
    pyramid_pos(pos);
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);
    ROS_INFO("going: {%.1f, %.1f, %.1f, %.1f} deg", pos.dof1, pos.dof2, pos.dof3, pos.dof4);

    //open
    open(pos);
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);
    ROS_INFO("Opening gripper...");

    //def open
    default_pos_open(pos);
    out.data = {pos.dof1, pos.dof2, pos.dof3, pos.dof4, pos.dof5};
    pub.publish(out);
    ROS_INFO("going: {%.1f, %.1f, %.1f, %.1f} deg (default pos)\n", pos.dof1, pos.dof2, pos.dof3, pos.dof4);

    counter++;
    //
}

int main(int argc, char **argv){

    ros::init(argc, argv, "algorithm_node");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float64MultiArray>("/servo_algo", 10);   // publish the thing
    ros::Subscriber sub = nh.subscribe("/joint_angles", 10, callback);    //subscribe to joint_angles
    ros::spin();
    return 0;

}
