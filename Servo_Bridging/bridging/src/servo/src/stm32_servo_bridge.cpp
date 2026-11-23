#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#include <exception>
#include <string>

#include "servo_robot.hpp"

class JembatanLengan
{
public:
    explicit JembatanLengan(const robot::Konfigurasi &konfig) : lengan_(konfig){
    }

    void sambung(){
        lengan_.sambung();
    }

// if use ultrasonic    ================================================
    void aktifkanJarak(const ros::Publisher &pub){
        pub_jarak_ = pub;
    }

    void jarakCallback(const std_msgs::Empty::ConstPtr &){
        std_msgs::Int32 hasil;
        hasil.data = -1;
        try{
            hasil.data = lengan_.jarakCm();
        }
        catch (const robot::SensorJarakGagal &e){
            ROS_WARN("Sensor jarak: %s", e.what());
        }
        catch (const robot::PerintahDitolak &e){
            ROS_ERROR("STM32 menolak perintah jarak: %s", e.what());
        }
        catch (const robot::KomunikasiGagal &e){
            ROS_FATAL("Komunikasi gagal: %s", e.what());
            ros::shutdown();
            return;
        }
        pub_jarak_.publish(hasil);
    }
//=======================================================================

//gerak sendi
    void sendiCallback(const std_msgs::Float64MultiArray::ConstPtr &msg){
        //make sure 5 servos
        if (msg->data.size() != 5){
            ROS_WARN("lengan_target butuh 5 angka, diterima %zu", msg->data.size());
            return;
        }

        try{
            lengan_.gerakSemua(msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
            if (msg->data[4] >= 0.5)
            {
                lengan_.gripperTutup();
            }
            else
            {
                lengan_.gripperBuka();
            }
            lengan_.tungguSelesai();
        }

        //safety measurements
        catch (const robot::SudutDiLuarBatas &e){
            ROS_WARN("Ditolak, sudut di luar batas: %s", e.what());
        }

        catch (const robot::PerintahDitolak &e){
            ROS_ERROR("STM32 menolak perintah: %s", e.what());
        }

        catch (const robot::KomunikasiGagal &e){
            ROS_FATAL("Komunikasi gagal: %s", e.what());
            ros::shutdown();
        }
    }

private:
    robot::Robot lengan_;
    ros::Publisher pub_jarak_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_servo_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    robot::Konfigurasi konfig;
    pnh.param<std::string>("port", konfig.port, "/dev/ttyACM0");

    try{
        JembatanLengan jembatan(konfig);
        ROS_INFO("Menyambung ke STM32 di %s ...", konfig.port.c_str());
        jembatan.sambung();
//subscriber
        ros::Subscriber sub = nh.subscribe("/servo_algo", 50, &JembatanLengan::sendiCallback, &jembatan);
        ROS_INFO("Connected.");

        ros::Subscriber sub_jarak;

        //ultrasonic
        bool pakai_ultrasonik = false;
        pnh.param<bool>("pakai_ultrasonik", pakai_ultrasonik, false);
        if (pakai_ultrasonik){
            jembatan.aktifkanJarak(nh.advertise<std_msgs::Int32>("jarak_cm", 10));
            sub_jarak = nh.subscribe("ukur_jarak", 12, &JembatanLengan::jarakCallback, &jembatan);
            ROS_INFO("Ultrasonik aktif: kirim Empty ke ukur_jarak, hasil di jarak_cm.");
        }
        ros::spin();
    }

    catch (const std::exception &e){
        ROS_FATAL("Gagal: %s", e.what());
        return 1;
    }
    return 0;
}
