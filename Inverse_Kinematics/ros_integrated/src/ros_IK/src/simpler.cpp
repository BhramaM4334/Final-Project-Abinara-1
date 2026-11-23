// Simple IK node: receives X,Y (mm), publishes 4 joint angles (radians)

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>

// Robot dimensions (mm)
const double L1 = 185.0;           // upper arm
const double L2 = 183.0;           // forearm
const double L3 = 94.0;            // gripper
const double SHOULDER_R = -13.68;  // shoulder is behind the base axis
const double SHOULDER_Z = 142.25;  // shoulder height (70 + 40 + 32.25)
const double Z_FIXED = 30.0;      // height of the tip (mm), change as needed
int counter = 1;
double Z[3] = {15.0, 65.0, 135.0};

struct servo_list{

    double S1;
    double S2;
    double S3;
    double S4;

};

ros::Publisher pub;

void toDeg(double &rad){
    rad = rad * 180 / M_PI;
}

void callback(const geometry_msgs::Point::ConstPtr &msg)
{
 //while (counter <6){
    double x = msg->x;
    double y = msg->y;
    double z = 45;

   /* switch (counter){
        case 1:
            z = Z[0];
            break;
        case 2:
            z = Z[0];
            break;
        case 3:
            z = Z[0];
            break;
        case 4:
            z = Z[1];
            break;
        case 5:
            z = Z[1];
            break;
        case 6:
            z = Z[2];
            break;
    }*/

    // Top view: base rotation and horizontal distance
    double dof1 = atan2(y, x);
    double r = sqrt(x * x + y * y);

    // Side view: shoulder -> wrist triangle
    double dr = (r - L3) - SHOULDER_R;
    double dz = z - SHOULDER_Z;
    double d = sqrt(dr * dr + dz * dz);

//check constraint
    if (d > L1 + L2 || d < fabs(L1 - L2)) {
        ROS_WARN("Unreachable target (%.1f, %.1f)", x, y);
        return;
    }

    // Law of cosinesble R2D = 180.0 / M_PI;
    double gamma = acos((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2));
    double beta  = acos((L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d));

    // Link directions, then joint angles, in pictures
    double satu = atan2(dz, dr) + beta;
    double dua = satu - (M_PI - gamma);

    double dof2 = satu - M_PI / 2.0;
    double dof3 = dua - satu;
    double dof4 = 180 - (satu+dua);f(abs(x))

    /* Joint limits
    if (fabs(dof1) > M_PI / 2 || dof2 < -M_PI / 2 || dof2 > 0 ||
        dof3 < -M_PI || dof3 > 0 || fabs(dof4) > M_PI / 2) {
        ROS_WARN("A joint would exceed its limit");
        return;
    }*/

    if(dof4<0) dof4=0;

    //publish
    std_msgs::Float64MultiArray out;

    //send data as radians, for mujoco and
    out.data = {dof1, abs(dof2), abs(dof3), dof4};

    pub.publish(out);

    ROS_INFO("XYZ = %f %f %f %d", x,y,z,counter);
    ROS_INFO("angles (rad): %.3f %.3f %.3f %.3f", dof1, dof2, dof3, dof4);
    counter++;
 //}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_node");
    ros::NodeHandle nh;
    pub = nh.advertise<std_msgs::Float64MultiArray>("/joint_angles", 10);   //publisher here, publish the float64multiarray
    ros::Subscriber sub = nh.subscribe("/target_xy", 10, callback);         //subscriber here, callback is callback
    ros::spin();
    return 0;
}
