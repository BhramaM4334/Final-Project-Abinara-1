// ik_node.cpp  -  ROS Noetic wrapper for the 5-DoF arm inverse kinematics
//
// Subscribes : target_xy     (geometry_msgs/Point)        x, y in input units (z ignored)
// Publishes  : joint_angles  (std_msgs/Float64MultiArray) [dof1..dof4] in radians (URDF angles)
//              servo_deg     (std_msgs/Float64MultiArray) [dof1..dof4] in servo degrees
//
// Private parameters (~):
//   z            (double)  fixed height of the tip in mm            default 100
//   input_scale  (double)  multiply incoming x,y to get mm          default 1.0
//                          (use 1000.0 if your source node sends meters)
//   servo_zero   (list)    servo command (deg) at URDF angle 0      default [90,90,90,90]
//   servo_dir    (list)    +1 or -1 per joint                       default [1,1,1,1]
//   servo_min    (list)    servo lower limit (deg)                  default [0,0,0,0]
//   servo_max    (list)    servo upper limit (deg)                  default [180,180,180,180]

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64MultiArray.h>

#include <cmath>
#include <vector>

// =====================================================================
// Robot dimensions (mm)
// =====================================================================
static const double PI = 3.14159265358979323846;

static const double BASE_HEIGHT   = 70.0 + 40.0;
static const double SHOULDER_RISE = 32.25;
static const double SHOULDER_BACK = 13.68;

static const double L1 = 185.0;
static const double L2 = 183.0;
static const double L3 = 94.0;

static const double SHOULDER_R = -SHOULDER_BACK;
static const double SHOULDER_Z = BASE_HEIGHT + SHOULDER_RISE;

// =====================================================================
// Joint limits and result codes
// =====================================================================
static const double DOF1_MIN = -PI / 2.0;  static const double DOF1_MAX = PI / 2.0;
static const double DOF2_MIN = -PI / 2.0;  static const double DOF2_MAX = 0.0;
static const double DOF3_MIN = -PI;        static const double DOF3_MAX = 0.0;
static const double DOF4_MIN = -PI / 2.0;  static const double DOF4_MAX = PI / 2.0;

enum IkStatus { IK_OK = 0, IK_TOO_FAR, IK_TOO_CLOSE, IK_OVER_LIMIT };

struct JointAngles {
    double dof1, dof2, dof3, dof4;
};

static double toDeg(double rad) { return rad * 180.0 / PI; }

static double safeAcos(double v)
{
    if (v > 1.0) v = 1.0;
    if (v < -1.0) v = -1.0;
    return std::acos(v);
}

static bool inLimit(double v, double lo, double hi)
{
    const double tol = 1e-9;
    return (v >= lo - tol) && (v <= hi + tol);
}

// =====================================================================
// Inverse kinematics (unchanged logic from ik_arm.cpp)
// =====================================================================
static IkStatus inverseKinematics(double x, double y, double z, JointAngles *out)
{
    double dof1 = std::atan2(y, x);
    double r = std::sqrt(x * x + y * y);

    double wrist_r = r - L3;
    double wrist_z = z;

    double dr = wrist_r - SHOULDER_R;
    double dz = wrist_z - SHOULDER_Z;
    double d  = std::sqrt(dr * dr + dz * dz);

    if (d > L1 + L2) return IK_TOO_FAR;
    if (d < std::fabs(L1 - L2)) return IK_TOO_CLOSE;

    double gamma = safeAcos((L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2));
    double beta  = safeAcos((L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d));

    double phi1 = std::atan2(dz, dr) + beta;
    double phi2 = phi1 - (PI - gamma);

    double dof2 = phi1 - PI / 2.0;
    double dof3 = phi2 - phi1;
    double dof4 = -phi2;

    if (!inLimit(dof1, DOF1_MIN, DOF1_MAX)) return IK_OVER_LIMIT;
    if (!inLimit(dof2, DOF2_MIN, DOF2_MAX)) return IK_OVER_LIMIT;
    if (!inLimit(dof3, DOF3_MIN, DOF3_MAX)) return IK_OVER_LIMIT;
    if (!inLimit(dof4, DOF4_MIN, DOF4_MAX)) return IK_OVER_LIMIT;

    out->dof1 = dof1;
    out->dof2 = dof2;
    out->dof3 = dof3;
    out->dof4 = dof4;
    return IK_OK;
}

// =====================================================================
// ROS node
// =====================================================================
class IkNode
{
public:
    IkNode() : pnh_("~")
    {
        pnh_.param("z", z_mm_, 100.0);
        pnh_.param("input_scale", scale_, 1.0);

        pnh_.param<std::vector<double> >("servo_zero", zero_, std::vector<double>(4, 90.0));
        pnh_.param<std::vector<double> >("servo_dir",  dir_,  std::vector<double>(4, 1.0));
        pnh_.param<std::vector<double> >("servo_min",  min_,  std::vector<double>(4, 0.0));
        pnh_.param<std::vector<double> >("servo_max",  max_,  std::vector<double>(4, 180.0));

        if (zero_.size() != 4 || dir_.size() != 4 || min_.size() != 4 || max_.size() != 4) {
            ROS_FATAL("servo_zero, servo_dir, servo_min and servo_max must each have 4 values");
            ros::shutdown();
            return;
        }

        pub_rad_   = nh_.advertise<std_msgs::Float64MultiArray>("joint_angles", 10);
        pub_servo_ = nh_.advertise<std_msgs::Float64MultiArray>("servo_deg", 10);
        sub_       = nh_.subscribe("target_xy", 10, &IkNode::onTarget, this);

        ROS_INFO("IK node ready. Fixed Z = %.1f mm, input scale = %.3f", z_mm_, scale_);
    }

private:
    void onTarget(const geometry_msgs::Point::ConstPtr &msg)
    {
        double x = msg->x * scale_;
        double y = msg->y * scale_;

        JointAngles q;
        IkStatus status = inverseKinematics(x, y, z_mm_, &q);

        if (status != IK_OK) {
            const char *why =
                (status == IK_TOO_FAR)   ? "target too far" :
                (status == IK_TOO_CLOSE) ? "target too close to shoulder" :
                                           "a joint would exceed its limit";
            ROS_WARN("Unreachable (x=%.1f y=%.1f z=%.1f mm): %s", x, y, z_mm_, why);
            return;  // publish nothing, the servos keep their last position
        }

        double rad[4] = { q.dof1, q.dof2, q.dof3, q.dof4 };

        std_msgs::Float64MultiArray out_rad, out_servo;
        for (int i = 0; i < 4; ++i) {
            out_rad.data.push_back(rad[i]);

            double cmd = zero_[i] + dir_[i] * toDeg(rad[i]);
            if (cmd < min_[i]) cmd = min_[i];
            if (cmd > max_[i]) cmd = max_[i];
            out_servo.data.push_back(cmd);
        }

        pub_rad_.publish(out_rad);
        pub_servo_.publish(out_servo);

        ROS_INFO("target (%.1f, %.1f, %.1f) -> rad [%.3f %.3f %.3f %.3f]  servo deg [%.1f %.1f %.1f %.1f]",
                 x, y, z_mm_, rad[0], rad[1], rad[2], rad[3],
                 out_servo.data[0], out_servo.data[1], out_servo.data[2], out_servo.data[3]);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_;
    ros::Publisher pub_rad_;
    ros::Publisher pub_servo_;

    double z_mm_;
    double scale_;
    std::vector<double> zero_, dir_, min_, max_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ik_node");
    IkNode node;
    ros::spin();
    return 0;
}
