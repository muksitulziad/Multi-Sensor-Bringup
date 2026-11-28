#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;
using Clock = std::chrono::steady_clock;

namespace {
volatile std::sig_atomic_t interrupted = 0;
void interrupt(int) { interrupted = 1; }

constexpr int attempts = 30;
constexpr int required = 27;
constexpr int sectors = 12;
constexpr double minHz = 3.0;
constexpr double maxGap = 1.0;

int sdkError(const char* operation, sl_result result) {
    std::cerr << "[FAIL] " << operation << " (SDK 0x" << std::hex << result << std::dec << ")\n";
    return 1;
}

bool number(const char* text, double& value) {
    char* end;
    value = std::strtod(text, &end);
    return end != text && *end == '\0' && std::isfinite(value);
}

struct Target {
    double metres = 0, degrees = 0, tolerance = 0.05;
};

struct Scan {
    size_t valid = 0;
    std::array<bool, sectors> covered{};
    double nearest = std::numeric_limits<double>::infinity();
    double farthest = 0, nearestAngle = 0;
};

Scan inspect(const sl_lidar_response_measurement_node_hq_t* nodes, size_t count,
             const Target& target, std::vector<double>& hits) {
    Scan scan;
    hits.clear();
    for (size_t i = 0; i < count; ++i) {
        if (!nodes[i].dist_mm_q2 || !nodes[i].quality) continue;
        const double metres = nodes[i].dist_mm_q2 / 4000.0;
        const double degrees = nodes[i].angle_z_q14 * (90.0 / 16384.0);
        ++scan.valid;
        scan.covered[static_cast<size_t>(degrees / 30)] = true;
        if (metres < scan.nearest) {
            scan.nearest = metres;
            scan.nearestAngle = degrees;
        }
        scan.farthest = std::max(scan.farthest, metres);
        if (target.metres && std::abs(std::remainder(degrees - target.degrees, 360.0)) <= 5.0)
            hits.push_back(metres);
    }
    return scan;
}

double median(std::vector<double>& hits) {
    auto mid = hits.begin() + hits.size() / 2;
    std::nth_element(hits.begin(), mid, hits.end());
    if (hits.size() % 2) return *mid;
    return (*mid + *std::max_element(hits.begin(), mid)) / 2;
}

int measure(ILidarDriver& driver, const Target& target, bool healthy) {
    std::array<sl_lidar_response_measurement_node_hq_t, 8192> nodes;
    std::vector<double> hits;
    size_t count = nodes.size();
    auto result = driver.grabScanDataHq(nodes.data(), count, 5000);
    if (interrupted) return 130;
    if (SL_IS_FAIL(result)) return sdkError("No complete warm-up scan within 5 seconds", result);

    std::cout << "Indoor bench criteria: 27/30 scans with >=24 valid returns across >=8/12 sectors; "
                 "delivery >=3 Hz, longest gap <=1 s. Bearings use the sensor frame.\n";
    if (target.metres)
        std::cout << "Target: " << target.metres << " m at " << target.degrees
                  << " deg (+/-5 deg), tolerance " << target.tolerance << " m.\n";
    int delivered = 0, usable = 0, targetPassed = 0, timeouts = 0, errors = 0;
    const auto start = Clock::now();
    auto last = start;
    double longest = 0;
    for (int i = 1; i <= attempts && !interrupted; ++i) {
        count = nodes.size();
        result = driver.grabScanDataHq(nodes.data(), count, 1000);
        const auto now = Clock::now();
        const double gap = std::chrono::duration<double>(now - last).count();
        longest = std::max(longest, gap);
        if (interrupted) break;
        std::cout << "[" << i << "/" << attempts << "] ";
        if (SL_IS_FAIL(result)) {
            if (result == SL_RESULT_OPERATION_TIMEOUT) {
                ++timeouts;
                std::cout << "Scan timeout\n";
            } else {
                ++errors;
                std::cout << "Scan read error 0x" << std::hex << result << std::dec << "\n";
            }
            continue;
        }
        last = now;
        ++delivered;
        const Scan scan = inspect(nodes.data(), count, target, hits);
        const auto coverage = std::count(scan.covered.begin(), scan.covered.end(), true);
        if (scan.valid >= 24 && coverage >= 8) ++usable;
        std::cout << "gap " << gap * 1000 << " ms | returns " << scan.valid << "/" << count
                  << " | sectors " << coverage << "/12";
        if (scan.valid)
            std::cout << " | nearest " << scan.nearest << " m @ " << scan.nearestAngle
                      << " deg | farthest " << scan.farthest << " m";
        if (target.metres) {
            if (hits.size() >= 3) {
                const double distance = median(hits);
                const double error = distance - target.metres;
                if (std::abs(error) <= target.tolerance) ++targetPassed;
                std::cout << " | target " << distance << " m, error " << error << " m";
            } else {
                std::cout << " | target: insufficient returns";
            }
        }
        std::cout << '\n';
    }
    if (interrupted) return 130;
    const double seconds = std::chrono::duration<double>(Clock::now() - start).count();
    const double hz = seconds > 0 ? delivered / seconds : 0;
    const bool stream = delivered >= required && hz >= minHz && longest <= maxGap;
    std::cout << (stream ? "[PASS]" : "[FAIL]") << " Delivery: " << delivered << "/" << attempts
              << ", " << hz << " Hz, longest gap " << longest << " s, timeouts " << timeouts
              << ", other errors " << errors << '\n';
    std::cout << (usable >= required ? "[PASS]" : "[FAIL]") << " Indoor coverage: " << usable
              << "/" << attempts << " usable scans\n";
    if (target.metres)
        std::cout << (targetPassed >= required ? "[PASS]" : "[FAIL]") << " Known target: "
                  << targetPassed << "/" << attempts << " within tolerance\n";
    else
        std::cout << "[UNTESTED] Range and bearing accuracy. Supply a known target to check them.\n";
    return healthy && stream && usable >= required && (!target.metres || targetPassed >= required) ? 0 : 1;
}
} // namespace

int main(int argc, char** argv) {
    const std::string usage = "Usage: lidar_check [PORT [TARGET_METRES BEARING_DEG [TOLERANCE_METRES]]]\n";
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << usage;
        return 0;
    }
    Target target;
    if ((argc != 1 && argc != 2 && argc != 4 && argc != 5) ||
        (argc >= 4 && (!number(argv[2], target.metres) || target.metres <= 0 ||
                       !number(argv[3], target.degrees) || target.degrees < 0 || target.degrees >= 360)) ||
        (argc == 5 && (!number(argv[4], target.tolerance) || target.tolerance <= 0))) {
        std::cerr << usage;
        return 2;
    }
    const char* port = argc > 1 ? argv[1] : "/dev/ttyUSB0";
    std::signal(SIGINT, interrupt);
    std::cout << std::fixed << std::setprecision(2) << "RPLIDAR bench test | " << port << " @ 115200\n";
    auto serial = createSerialPortChannel(port, 115200);
    if (!serial) return sdkError("Create serial channel", serial.err);
    std::unique_ptr<IChannel> channel(*serial);
    auto created = createLidarDriver();
    if (!created) return sdkError("Create driver", created.err);
    std::unique_ptr<ILidarDriver> driver(*created);
    auto result = driver->connect(channel.get());
    if (SL_IS_FAIL(result)) return sdkError("Connect: check port, permissions and other serial readers", result);

    sl_lidar_response_device_info_t info{};
    result = driver->getDeviceInfo(info);
    if (SL_IS_FAIL(result)) return sdkError("Read device information", result);
    std::cout << "[PASS] Model ID " << unsigned(info.model) << ", firmware " << (info.firmware_version >> 8)
              << "." << (info.firmware_version & 255) << ", hardware " << unsigned(info.hardware_version)
              << ", serial " << std::hex << std::setfill('0');
    for (auto byte : info.serialnum) std::cout << std::setw(2) << unsigned(byte);
    std::cout << std::dec << std::setfill(' ') << '\n';

    sl_lidar_response_device_health_t health{};
    result = driver->getHealth(health);
    if (SL_IS_FAIL(result)) return sdkError("Read device health", result);
    const bool healthy = health.status == SL_LIDAR_STATUS_OK;
    if (!healthy && health.status != SL_LIDAR_STATUS_WARNING) {
        std::cerr << "[FAIL] Device health status " << unsigned(health.status)
                  << ", error code " << health.error_code << '\n';
        return 1;
    }
    std::cout << (healthy ? "[PASS] Health OK" : "[WARN] Device health warning")
              << ", device error code " << health.error_code << '\n';
    if (interrupted) return 130;

    int status = 1;
    result = driver->setMotorSpeed();
    if (SL_IS_FAIL(result)) sdkError("Start motor", result);
    else if (!interrupted) {
        result = driver->startScan(false, true);
        if (SL_IS_FAIL(result)) sdkError("Start scan", result);
        else status = measure(*driver, target, healthy);
    }
    result = driver->stop();
    if (SL_IS_FAIL(result)) status = sdkError("Stop scan", result);
    result = driver->setMotorSpeed(0);
    if (SL_IS_FAIL(result)) status = sdkError("Stop motor", result);
    driver->disconnect();
    if (interrupted) {
        std::cout << "[INTERRUPTED] Test cancelled\n";
        return 130;
    }
    std::cout << (status == 0 ? "[PASS] Requested bench checks passed\n" : "[FAIL] Bench test failed\n");
    return status;
}
