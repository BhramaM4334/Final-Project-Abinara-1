// servo_robot.cpp
// Implementasi pustaka middle level. Linux, C++17, tanpa ROS.
// STATUS: BELUM TERUJI di STM32 asli.

#include "servo_robot.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace robot
{

    namespace
    {

        using Jam = std::chrono::steady_clock;

        // ===============================================================
        // LAPISAN 1: PORT. Hanya urusan membuka, menulis, membaca port.
        // ===============================================================
        class PortSerial
        {
        public:
            PortSerial() = default;
            PortSerial(const PortSerial &) = delete;
            PortSerial &operator=(const PortSerial &) = delete;
            ~PortSerial() { tutup(); }

            void buka(const std::string &nama)
            {
                tutup();
                fd_ = ::open(nama.c_str(), O_RDWR | O_NOCTTY);
                if (fd_ < 0)
                {
                    throw KomunikasiGagal("tidak bisa membuka " + nama + ": " +
                    std::strerror(errno) +
                    " (cek kabel, nama port, dan grup uucp)");
                }

                termios tio{};
                if (tcgetattr(fd_, &tio) != 0)
                {
                    const std::string sebab = std::strerror(errno);
                    tutup();
                    throw KomunikasiGagal(nama + " bukan port serial: " + sebab);
                }
                cfmakeraw(&tio);
                cfsetispeed(&tio, B115200);
                cfsetospeed(&tio, B115200);
                tio.c_cflag |= (CLOCAL | CREAD);
                tio.c_cc[VMIN] = 0;
                tio.c_cc[VTIME] = 0;
                if (tcsetattr(fd_, TCSANOW, &tio) != 0)
                {
                    const std::string sebab = std::strerror(errno);
                    tutup();
                    throw KomunikasiGagal("gagal mengatur " + nama + ": " + sebab);
                }
                tcflush(fd_, TCIOFLUSH);
                sisa_.clear();
            }

            void tutup()
            {
                if (fd_ >= 0)
                {
                    ::close(fd_);
                    fd_ = -1;
                }
                sisa_.clear();
            }

            bool terbuka() const { return fd_ >= 0; }

            void kirim(const std::string &teks)
            {
                std::size_t terkirim = 0;
                while (terkirim < teks.size())
                {
                    const ssize_t n = ::write(fd_, teks.data() + terkirim,
                                              teks.size() - terkirim);
                    if (n < 0)
                    {
                        if (errno == EINTR)
                        {
                            continue;
                        }
                        throw KomunikasiGagal(std::string("gagal menulis ke port: ") +
                        std::strerror(errno));
                    }
                    terkirim += static_cast<std::size_t>(n);
                }
            }

            // Baca satu baris (tanpa \r\n). true = dapat baris,
            // false = waktu habis sebelum ada baris utuh.
            bool bacaBaris(int batas_ms, std::string &baris)
            {
                const auto batas = Jam::now() + std::chrono::milliseconds(batas_ms);
                while (true)
                {
                    const std::size_t akhir = sisa_.find('\n');
                    if (akhir != std::string::npos)
                    {
                        baris = sisa_.substr(0, akhir);
                        sisa_.erase(0, akhir + 1);
                        if (!baris.empty() && baris.back() == '\r')
                        {
                            baris.pop_back();
                        }
                        return true;
                    }

                    const auto sisa_waktu =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        batas - Jam::now()).count();
                        if (sisa_waktu <= 0)
                        {
                            return false;
                        }

                        pollfd pf{};
                        pf.fd = fd_;
                        pf.events = POLLIN;
                        const int hasil = ::poll(&pf, 1, static_cast<int>(sisa_waktu));
                        if (hasil < 0)
                        {
                            if (errno == EINTR)
                            {
                                continue;
                            }
                            throw KomunikasiGagal(std::string("poll gagal: ") +
                            std::strerror(errno));
                        }
                        if (hasil == 0)
                        {
                            return false;
                        }
                        if (pf.revents & (POLLERR | POLLHUP | POLLNVAL))
                        {
                            throw KomunikasiGagal("port terputus (kabel USB tercabut?)");
                        }

                        char tmp[256];
                        const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
                        if (n < 0)
                        {
                            if (errno == EINTR || errno == EAGAIN)
                            {
                                continue;
                            }
                            throw KomunikasiGagal(std::string("gagal membaca port: ") +
                            std::strerror(errno));
                        }
                        if (n == 0)
                        {
                            throw KomunikasiGagal("port tertutup oleh STM32");
                        }
                        sisa_.append(tmp, static_cast<std::size_t>(n));
                }
            }

            // Buang semua data masuk yang belum dibaca (balasan basi).
            void buangMasuk()
            {
                if (fd_ >= 0)
                {
                    tcflush(fd_, TCIFLUSH);
                }
                sisa_.clear();
            }

        private:
            int fd_ = -1;
            std::string sisa_;   // potongan baris yang belum lengkap
        };

    } // namespace

    // ===============================================================
    // Impl: semua isi Robot disimpan di sini (disembunyikan dari .hpp)
    // ===============================================================
    struct Robot::Impl
    {
        Konfigurasi k;
        PortSerial port;
        bool tersambung = false;

        // Indeks 0..3 = Base..Pergelangan, 4 = gripper.
        double servo_terakhir[5];            // sudut SERVO yang terakhir diminta
        Jam::time_point selesai[5];          // perkiraan waktu tiba tiap servo

        explicit Impl(const Konfigurasi &konfig) : k(konfig)
        {
            if (k.derajat_per_detik <= 0.0)
            {
                throw std::invalid_argument("derajat_per_detik harus > 0");
            }
            for (int i = 0; i < 5; ++i)
            {
                servo_terakhir[i] = k.sudut_servo_awal;
                selesai[i] = Jam::now();
            }
        }

        const KonfigSendi &konfigSendi(Sendi sendi) const
        {
            switch (sendi)
            {
                case Sendi::Base:        return k.base;
                case Sendi::Bahu:        return k.bahu;
                case Sendi::Siku:        return k.siku;
                case Sendi::Pergelangan: return k.pergelangan;
            }
            throw std::invalid_argument("sendi tidak dikenal");
        }

        static const char *namaSendi(Sendi sendi)
        {
            switch (sendi)
            {
                case Sendi::Base:        return "Base";
                case Sendi::Bahu:        return "Bahu";
                case Sendi::Siku:        return "Siku";
                case Sendi::Pergelangan: return "Pergelangan";
            }
            return "?";
        }

        void pastikanTersambung() const
        {
            if (!tersambung)
            {
                throw KomunikasiGagal("belum tersambung, panggil sambung() dulu");
            }
        }

        // Sudut sendi -> sudut servo (bulat). Melempar SudutDiLuarBatas
        // bila di luar batas sendi atau hasilnya di luar 0..180.
        int keSudutServo(Sendi sendi, double sudut) const
        {
            const KonfigSendi &c = konfigSendi(sendi);
            if (!std::isfinite(sudut) || sudut < c.batas_min || sudut > c.batas_maks)
            {
                throw SudutDiLuarBatas(std::string(namaSendi(sendi)) + ": sudut " +
                std::to_string(sudut) + " di luar batas " +
                std::to_string(c.batas_min) + ".." +
                std::to_string(c.batas_maks));
            }
            const long servo = std::lround(c.offset + c.arah * sudut);
            if (servo < 0 || servo > 180)
            {
                throw SudutDiLuarBatas(std::string(namaSendi(sendi)) +
                ": sudut servo hasil konversi " +
                std::to_string(servo) + " di luar 0..180");
            }
            return static_cast<int>(servo);
        }

        // ===========================================================
        // LAPISAN 2: PESAN. Kirim "S<n> <sudut>\n", tunggu OK / ERR.
        // ===========================================================
        void kirimPerintah(int nomor, int sudut_servo)
        {
            const std::string teks =
            "S" + std::to_string(nomor) + " " + std::to_string(sudut_servo);

            for (int percobaan = 1; percobaan <= k.coba; ++percobaan)
            {
                port.buangMasuk();
                port.kirim(teks + "\n");

                const auto batas =
                Jam::now() + std::chrono::milliseconds(k.batas_waktu_ms);
                std::string baris;
                while (true)
                {
                    const auto sisa =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        batas - Jam::now()).count();
                        if (sisa <= 0 || !port.bacaBaris(static_cast<int>(sisa), baris))
                        {
                            break;   // waktu habis: coba lagi
                        }
                        if (baris == "OK")
                        {
                            return;
                        }
                        if (baris.compare(0, 3, "ERR") == 0)
                        {
                            throw PerintahDitolak("STM32 menolak '" + teks + "': " + baris);
                        }
                        // baris lain (misalnya Halo) diabaikan
                }
            }
            throw KomunikasiGagal("STM32 tidak membalas '" + teks + "' setelah " +
            std::to_string(k.coba) + " percobaan");
        }

        // ===========================================================
        // Ukur jarak (opsional, hanya dipanggil bila ultrasonik dipakai).
        // Kirim "D\n", tunggu "DIST <cm>" atau "ERR JARAK".
        // ===========================================================
        int tanyaJarak()
        {
            for (int percobaan = 1; percobaan <= k.coba; ++percobaan)
            {
                port.buangMasuk();
                port.kirim("D\n");

                const auto batas =
                Jam::now() + std::chrono::milliseconds(k.batas_waktu_ms);
                std::string baris;
                while (true)
                {
                    const auto sisa =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        batas - Jam::now()).count();
                    if (sisa <= 0 || !port.bacaBaris(static_cast<int>(sisa), baris))
                    {
                        break;   // waktu habis: coba lagi
                    }
                    if (baris.compare(0, 5, "DIST ") == 0)
                    {
                        try
                        {
                            return std::stoi(baris.substr(5));
                        }
                        catch (const std::exception &)
                        {
                            throw KomunikasiGagal("balasan jarak rusak: '" + baris + "'");
                        }
                    }
                    if (baris == "ERR JARAK")
                    {
                        throw SensorJarakGagal("sensor jarak tidak memberi hasil "
                        "(tidak terpasang atau tidak ada pantulan)");
                    }
                    if (baris.compare(0, 3, "ERR") == 0)
                    {
                        throw PerintahDitolak("STM32 menolak 'D': " + baris +
                        " (firmware belum punya perintah D?)");
                    }
                    // baris lain (misalnya Halo) diabaikan
                }
            }
            throw KomunikasiGagal("STM32 tidak membalas 'D' setelah " +
            std::to_string(k.coba) + " percobaan");
        }

        // ===========================================================
        // LAPISAN 3: perkiraan waktu tiba (untuk tungguSelesai)
        // ===========================================================
        void catatGerak(int indeks, int sudut_servo)
        {
            const double selisih = std::fabs(sudut_servo - servo_terakhir[indeks]);
            const auto lama = std::chrono::duration_cast<Jam::duration>(
                std::chrono::duration<double>(selisih / k.derajat_per_detik));
            const auto mulai = std::max(Jam::now(), selesai[indeks]);
            selesai[indeks] = mulai + lama;
            servo_terakhir[indeks] = sudut_servo;
        }

        void kirimDanCatat(int indeks, int sudut_servo)
        {
            kirimPerintah(indeks + 1, sudut_servo);
            catatGerak(indeks, sudut_servo);
        }
    };

    // ===============================================================
    // LAPISAN 4: ROBOT (yang dipanggil Programming)
    // ===============================================================
    Robot::Robot(const Konfigurasi &konfig) : impl_(new Impl(konfig)) {}

    Robot::~Robot()
    {
        if (impl_)
        {
            impl_->port.tutup();
        }
    }

    void Robot::sambung()
    {
        Impl &d = *impl_;
        d.tersambung = false;
        d.port.buka(d.k.port);

        // Tunggu heartbeat pertama: tanda STM32 selesai inisialisasi.
        const auto batas =
        Jam::now() + std::chrono::milliseconds(d.k.tunggu_siap_ms);
        std::string baris;
        while (true)
        {
            const auto sisa =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                batas - Jam::now()).count();
                if (sisa <= 0 || !d.port.bacaBaris(static_cast<int>(sisa), baris))
                {
                    d.port.tutup();
                    throw KomunikasiGagal("STM32 tidak mengirim '" + d.k.pesan_siap +
                    "' dalam " + std::to_string(d.k.tunggu_siap_ms) +
                    " ms (firmware jalan? kabel? port benar?)");
                }
                if (baris.find(d.k.pesan_siap) != std::string::npos)
                {
                    break;
                }
        }
        d.tersambung = true;
    }

    void Robot::putus()
    {
        impl_->port.tutup();
        impl_->tersambung = false;
    }

    bool Robot::tersambung() const
    {
        return impl_->tersambung;
    }

    void Robot::gerakSendi(Sendi sendi, double sudut)
    {
        Impl &d = *impl_;
        d.pastikanTersambung();
        const int servo = d.keSudutServo(sendi, sudut);
        d.kirimDanCatat(static_cast<int>(sendi), servo);
    }

    void Robot::gerakSemua(double base, double bahu, double siku, double pergelangan)
    {
        Impl &d = *impl_;
        d.pastikanTersambung();

        // Periksa keempatnya dulu supaya tidak ada gerak setengah jadi.
        const int s1 = d.keSudutServo(Sendi::Base, base);
        const int s2 = d.keSudutServo(Sendi::Bahu, bahu);
        const int s3 = d.keSudutServo(Sendi::Siku, siku);
        const int s4 = d.keSudutServo(Sendi::Pergelangan, pergelangan);

        d.kirimDanCatat(0, s1);
        d.kirimDanCatat(1, s2);
        d.kirimDanCatat(2, s3);
        d.kirimDanCatat(3, s4);
    }

    void Robot::gripperKe(double sudut_servo)
    {
        Impl &d = *impl_;
        d.pastikanTersambung();
        if (!std::isfinite(sudut_servo) || sudut_servo < 0.0 || sudut_servo > 180.0)
        {
            throw SudutDiLuarBatas("gripper: sudut " + std::to_string(sudut_servo) +
            " di luar 0..180");
        }
        d.kirimDanCatat(4, static_cast<int>(std::lround(sudut_servo)));
    }

    void Robot::gripperBuka()
    {
        gripperKe(impl_->k.gripper_buka);
    }

    void Robot::gripperTutup()
    {
        gripperKe(impl_->k.gripper_tutup);
    }

    void Robot::tungguSelesai()
    {
        Impl &d = *impl_;
        const auto paling_akhir = *std::max_element(d.selesai, d.selesai + 5);
        if (paling_akhir <= Jam::now())
        {
            return;   // tidak ada gerakan yang masih diperkirakan berjalan
        }
        const auto cadangan = std::chrono::duration_cast<Jam::duration>(
            std::chrono::duration<double>(d.k.cadangan_detik));
        std::this_thread::sleep_until(paling_akhir + cadangan);
    }

    int Robot::jarakCm()
    {
        Impl &d = *impl_;
        d.pastikanTersambung();
        return d.tanyaJarak();
    }

    double Robot::sudutTerakhirDiminta(Sendi sendi) const
    {
        const Impl &d = *impl_;
        const KonfigSendi &c = d.konfigSendi(sendi);
        return (d.servo_terakhir[static_cast<int>(sendi)] - c.offset) / c.arah;
    }

} // namespace robot
