#ifndef SERVO_ROBOT_HPP
#define SERVO_ROBOT_HPP

#include <memory>
#include <stdexcept>
#include <string>

namespace robot
{

    enum class Sendi{
        Base,
        Bahu,
        Siku,
        Pergelangan
    };

    class KomunikasiGagal : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    class PerintahDitolak : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    class SensorJarakGagal : public std::runtime_error
    {
    public:
        using std::runtime_error::runtime_error;
    };

    class SudutDiLuarBatas : public std::out_of_range
    {
    public:
        using std::out_of_range::out_of_range;
    };

    struct KonfigSendi
    {
        double offset;
        double arah;
        double batas_min;
        double batas_maks;
    };

    struct Konfigurasi
    {
        std::string port = "/dev/ttyACM0";

        KonfigSendi base        {180.0, -1.0, 0.0, 180.0};
        KonfigSendi bahu        { 0.0, +1.0,   0.0, 180.0};
        KonfigSendi siku        { 0.0, +1.0,   0.0, 180.0};
        KonfigSendi pergelangan { 0.0, +1.0,   0.0, 180.0};

        double gripper_buka = 0.0;
        double gripper_tutup = 60.0;

        double derajat_per_detik = 20.5;
        double cadangan_detik = 0.7;

        int coba = 3;
        int batas_waktu_ms = 500;
        int tunggu_siap_ms = 4000;
        std::string pesan_siap = "Halo dari STM32";

        double sudut_servo_awal = 45.0;
    };

    class Robot
    {
    public:
        explicit Robot(const Konfigurasi &konfig = Konfigurasi());
        ~Robot();

        Robot(const Robot &) = delete;
        Robot &operator=(const Robot &) = delete;

        void sambung();
        void putus();
        bool tersambung() const;

        void gerakSendi(Sendi sendi, double sudut);
        void gerakSemua(double base, double bahu, double siku, double pergelangan);

        void gripperBuka();
        void gripperTutup();
        void gripperKe(double sudut_servo);

        void tungguSelesai();

        int jarakCm();

        double sudutTerakhirDiminta(Sendi sendi) const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

}

#endif
