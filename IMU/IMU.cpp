#include <iostream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <chrono>
#include <thread>

const char* SPI_DEVICE = "/dev/spidev0.0";
const uint32_t SPI_SPEED = 1000000; // 1 MHz (কনফিগারেশনের পর 10 MHz পর্যন্ত বাড়ানো যায়)

// রেজিস্টার অ্যাড্রেস (MPU9250 / MPU6500)
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_USER_CTRL    = 0x6A;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_WHO_AM_I     = 0x75;

class JetsonSPIMU {
private:
    int spi_fd;
    double gx_off = 0, gy_off = 0, gz_off = 0;
    double ax_off = 0, ay_off = 0, az_off = 0;
    double roll = 0, pitch = 0, yaw = 0;

    // SPI ফুল-ডুপ্লেক্স ট্রান্সফার ফাংশন
    void spiTransfer(uint8_t* tx, uint8_t* rx, size_t len) {
        struct spi_ioc_transfer tr;
        std::memset(&tr, 0, sizeof(tr));
        tr.tx_buf = reinterpret_cast<unsigned long>(tx);
        tr.rx_buf = reinterpret_cast<unsigned long>(rx);
        tr.len = len;
        tr.speed_hz = SPI_SPEED;
        tr.bits_per_word = 8;

        if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
            std::cerr << "[ERROR] SPI ট্রান্সফার ব্যর্থ হয়েছে!\n";
        }
    }

    void writeRegister(uint8_t reg, uint8_t val) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val}; // MSB = 0 মানে Write
        uint8_t rx[2] = {0};
        spiTransfer(tx, rx, 2);
    }

    uint8_t readRegister(uint8_t reg) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00}; // MSB = 1 মানে Read
        uint8_t rx[2] = {0};
        spiTransfer(tx, rx, 2);
        return rx[1];
    }

public:
    bool init() {
        spi_fd = open(SPI_DEVICE, O_RDWR);
        if (spi_fd < 0) {
            std::cerr << "[ERROR] SPI ডিভাইস ওপেন করা যায়নি: " << SPI_DEVICE << "\n";
            return false;
        }

        uint8_t mode = SPI_MODE_3; // MPU সিরিজের জন্য SPI Mode 3 (বা Mode 0)
        uint8_t bits = 8;
        ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
        ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &SPI_SPEED);

        // ১. চিপ রিসেট ও I2C ইন্টারফেস ডিজেবল করে পিওর SPI মোড চালু
        writeRegister(REG_PWR_MGMT_1, 0x80);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        writeRegister(REG_PWR_MGMT_1, 0x01); // Auto clock select
        writeRegister(REG_USER_CTRL, 0x10);  // I2C_IF_DIS = 1 (শুধু SPI চালু রাখবে)
        writeRegister(REG_CONFIG, 0x03);     // DLPF ফিল্টার

        // ২. WHO_AM_I ভেরিফিকেশন (MPU9250 = 0x71, MPU6500 = 0x70)
        uint8_t id = readRegister(REG_WHO_AM_I);
        std::cout << "[INFO] SPI WHO_AM_I ID: 0x" << std::hex << (int)id << std::dec << "\n";
        if (id == 0x00 || id == 0xFF) {
            std::cerr << "[ERROR] SPI কানেকশন কাজ করছে না (MOSI/MISO/CS পিন চেক করুন)!\n";
            return false;
        }
        std::cout << "[PASS] হাই-স্পিড SPI কমিউনিকেশন সফল!\n";
        return true;
    }

    // একটাই SPI বার্স্ট ট্রান্সফারে ১৪ বাইট ডেটা পড়া (অত্যন্ত দ্রুত)
    void readBurstData(double &ax, double &ay, double &az, double &gx, double &gy, double &gz) {
        uint8_t tx[15] = {0};
        uint8_t rx[15] = {0};
        tx[0] = REG_ACCEL_XOUT_H | 0x80; // Read কমান্ড

        spiTransfer(tx, rx, 15);

        int16_t raw_ax = (rx[1] << 8) | rx[2];
        int16_t raw_ay = (rx[3] << 8) | rx[4];
        int16_t raw_az = (rx[5] << 8) | rx[6];
        int16_t raw_gx = (rx[9] << 8) | rx[10];
        int16_t raw_gy = (rx[11] << 8) | rx[12];
        int16_t raw_gz = (rx[13] << 8) | rx[14];

        ax = (raw_ax / 16384.0) - ax_off;
        ay = (raw_ay / 16384.0) - ay_off;
        az = (raw_az / 16384.0) - az_off;

        gx = (raw_gx / 131.0) - gx_off;
        gy = (raw_gy / 131.0) - gy_off;
        gz = (raw_gz / 131.0) - gz_off;
    }

    void calibrate(int samples = 500) {
        std::cout << "[INFO] হাই-স্পিড ক্যালিব্রেশন চলছে (সেন্সর স্থির রাখুন)...\n";
        double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;

        for (int i = 0; i < samples; ++i) {
            double ax, ay, az, gx, gy, gz;
            readBurstData(ax, ay, az, gx, gy, gz);
            sax += ax; say += ay; saz += (az - 1.0);
            sgx += gx; sgy += gy; sgz += gz;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        ax_off = sax / samples; ay_off = say / samples; az_off = saz / samples;
        gx_off = sgx / samples; gy_off = sgy / samples; gz_off = sgz / samples;
        std::cout << "[PASS] ক্যালিব্রেশন সম্পন্ন!\n\n";
    }

    void run1000HzLoop() {
        auto prev = std::chrono::steady_clock::now();
        int count = 0;
        double timer = 0.0;

        while (true) {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - prev).count();
            prev = now;

            double ax, ay, az, gx, gy, gz;
            readBurstData(ax, ay, az, gx, gy, gz);

            double acc_roll  = std::atan2(ay, az) * 180.0 / M_PI;
            double acc_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * 180.0 / M_PI;

            roll  = 0.98 * (roll + gx * dt) + 0.02 * acc_roll;
            pitch = 0.98 * (pitch + gy * dt) + 0.02 * acc_pitch;
            yaw  += gz * dt;

            count++;
            timer += dt;

            // প্রতি ১০০ মিলিসেকেন্ডে (10 Hz) টার্মিনালে প্রিন্ট (ভেতরে লুপ চলছে ~1000 Hz-এ)
            if (timer >= 0.1) {
                double hz = count / timer;
                double g_norm = std::sqrt(ax * ax + ay * ay + az * az);
                bool ok = (std::abs(g_norm - 1.0) < 0.05);

                std::cout << std::fixed << std::setprecision(2)
                          << "Rate: " << std::setw(6) << hz << " Hz | "
                          << "Roll: " << std::setw(6) << roll << "° | "
                          << "Pitch: " << std::setw(6) << pitch << "° | "
                          << "Yaw: " << std::setw(6) << yaw << "° | "
                          << "|G|: " << g_norm << "g "
                          << (ok ? "[ACCURATE]" : "[MOTION]")
                          << "\r" << std::flush;
                count = 0;
                timer = 0.0;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(800)); // ~1000 Hz টার্গেট
        }
    }

    ~JetsonSPIMU() {
        if (spi_fd >= 0) close(spi_fd);
    }
};

int main() {
    JetsonSPIMU imu;
    if (!imu.init()) return -1;
    imu.calibrate(500);
    imu.run1000HzLoop();
    return 0;
}