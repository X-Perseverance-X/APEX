// Official SLAMTEC SDK to newline-delimited normalized scan bridge.
// stdout is machine data; diagnostics go to stderr.
#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace {
volatile std::sig_atomic_t running = 1;

void request_stop(int) { running = 0; }

long long monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string serial_hex(const sl_lidar_response_device_info_t& info) {
    char value[33]{};
    for (int i = 0; i < 16; ++i) {
        std::snprintf(value + 2 * i, 3, "%02X", info.serialnum[i]);
    }
    return value;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr, "Usage: %s PORT BAUD EXPECTED_SERIAL\n", argv[0]);
        return 2;
    }
    const char* port = argv[1];
    const sl_u32 baud = static_cast<sl_u32>(std::strtoul(argv[2], nullptr, 10));
    const char* expected_serial = argv[3];

    ILidarDriver* lidar = *createLidarDriver();
    IChannel* channel = *createSerialPortChannel(port, baud);
    if (!lidar || !channel || SL_IS_FAIL(lidar->connect(channel))) {
        std::fprintf(stderr, "ERROR: LiDAR SDK cannot connect to %s\n", port);
        delete lidar;
        delete channel;
        return 3;
    }
    sl_lidar_response_device_info_t info{};
    sl_lidar_response_device_health_t health{};
    if (SL_IS_FAIL(lidar->getDeviceInfo(info)) ||
        SL_IS_FAIL(lidar->getHealth(health))) {
        std::fprintf(stderr, "ERROR: device did not respond as SLAMTEC LiDAR\n");
        delete lidar;
        delete channel;
        return 4;
    }
    const std::string serial = serial_hex(info);
    if (serial != expected_serial || health.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "ERROR: serial/health mismatch serial=%s status=%u\n",
                     serial.c_str(), health.status);
        delete lidar;
        delete channel;
        return 5;
    }
    std::fprintf(stderr, "LIDAR serial=%s firmware=%u.%02u health=%u\n",
                 serial.c_str(), info.firmware_version >> 8,
                 info.firmware_version & 0xffU, health.status);
    std::printf("{\"meta\":{\"sensor\":\"rplidar\",\"serial\":\"%s\","
                "\"firmware\":\"%u.%02u\",\"hardware_rev\":%u,\"health_code\":%u}}\n",
                serial.c_str(), info.firmware_version >> 8,
                info.firmware_version & 0xffU, info.hardware_version, health.status);
    std::fflush(stdout);

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    if (SL_IS_FAIL(lidar->setMotorSpeed())) {
        std::fprintf(stderr, "ERROR: cannot start LiDAR motor\n");
        delete lidar;
        delete channel;
        return 6;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (SL_IS_FAIL(lidar->startScan(false, true))) {
        std::fprintf(stderr, "ERROR: cannot start LiDAR scan\n");
        lidar->setMotorSpeed(0);
        delete lidar;
        delete channel;
        return 7;
    }

    unsigned long long sequence = 0;
    while (running) {
        sl_lidar_response_measurement_node_hq_t nodes[8192];
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        const long long start_ns = monotonic_ns();
        const sl_result result = lidar->grabScanDataHq(nodes, count, 2000);
        const long long end_ns = monotonic_ns();
        if (SL_IS_FAIL(result)) {
            std::fprintf(stderr, "WARN: grabScanDataHq result=0x%08X\n", result);
            continue;
        }
        lidar->ascendScanData(nodes, count);
        std::printf("{\"timestamp_start_ns\":%lld,\"timestamp_end_ns\":%lld,"
                    "\"sequence\":%llu,\"points\":[",
                    start_ns, end_ns, sequence++);
        bool first = true;
        for (size_t i = 0; i < count; ++i) {
            const double angle_deg = nodes[i].angle_z_q14 * 90.0 / 16384.0;
            const double angle_rad = angle_deg * 0.01745329251994329577;
            const double range_m = nodes[i].dist_mm_q2 / 4000.0;
            const unsigned quality = nodes[i].quality;
            if (!std::isfinite(range_m) || range_m <= 0.0 || quality == 0) continue;
            if (!first) std::putchar(',');
            std::printf("[%.8f,%.5f,%u]", angle_rad, range_m, quality);
            first = false;
        }
        std::printf("]}\n");
        std::fflush(stdout);
    }
    lidar->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    lidar->setMotorSpeed(0);
    delete lidar;
    delete channel;
    return 0;
}
