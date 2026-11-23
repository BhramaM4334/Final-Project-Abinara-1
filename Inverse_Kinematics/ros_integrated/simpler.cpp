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
int counter = 0;
double Z[3] = {30.0, 90.0, 150.0};

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
    double x = msg->x;
    double y = msg->y;
    double z;

    switch (counter){
    case 1:
        z = Z[0];
    case 2:
        z = Z[0];
    case 3:
        z = Z[0];
    case 4:
        z = Z[1];
    case 5:
        z = Z[1];
    case 6:
        z = Z[2];
    }

    // Top view: base rotation and horizontal distance
    double dof1 = atan2(y, x);
    double r = sqrt(x * x + y * y);

    // Side view: shoulder -> wrist triangle
    double dr = (r - L3) - SHOULDER_R;
    double dz = z - SHOULDER_Z;
    double d = sqrt(dr * dr + dz * dz);

    if (d > L1 + L2 || d < fabs(L1 - L2)) {
        ROS_WARN("Unreachable target (%.1f, %.1f)", x, y);
        return;
    }

    // Law of cosinesble R2D = 180.0 / M_PI;
    double gamma = acos((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2));
    double beta  = acos((L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d));

    // Link directions, then joint angles
    double phi1 = atan2(dz, dr) + beta;
    double phi2 = phi1 - (M_PI - gamma);

    double dof2 = phi1 - M_PI / 2.0;
    double dof3 = phi2 - phi1;
    double dof4 = -phi2;

    // Joint limits
    if (fabs(dof1) > M_PI / 2 || dof2 < -M_PI / 2 || dof2 > 0 ||
        dof3 < -M_PI || dof3 > 0 || fabs(dof4) > M_PI / 2) {
        ROS_WARN("A joint would exceed its limit");
        return;
    }

    std_msgs::Float64MultiArray out;
    out.data = {dof1, dof2, dof3, dof4};
    pub.publish(out);

    ROS_INFO("angles (rad): %.3f %.3f %.3f %.3f", dof1, dof2, dof3, dof4);
    counter++;
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
