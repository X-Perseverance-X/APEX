#include <chrono>
#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <unistd.h>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

namespace {

std::atomic_bool keep_running{true};

void stop_requested(int) {
    keep_running.store(false);
}

void usage(const char* argv0) {
    std::fprintf(stderr,
        "Usage: %s --port DEVICE [--baud 115200] [--hold] {status|on|off}\n",
        argv0);
}

std::string serial_hex(const sl_lidar_response_device_info_t& info) {
    char output[33]{};
    for (int i = 0; i < 16; ++i) {
        std::snprintf(output + i * 2, 3, "%02X", info.serialnum[i]);
    }
    return output;
}

const char* health_name(sl_u8 status) {
    switch (status) {
        case SL_LIDAR_STATUS_OK: return "OK";
        case SL_LIDAR_STATUS_WARNING: return "WARNING";
        case SL_LIDAR_STATUS_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

}  // namespace

int main(int argc, char** argv) {
    const char* port = nullptr;
    sl_u32 baud = 115200;
    const char* action = nullptr;
    bool hold = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            port = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baud = static_cast<sl_u32>(std::strtoul(argv[++i], nullptr, 10));
        } else if (std::strcmp(argv[i], "--hold") == 0) {
            hold = true;
        } else if (!action) {
            action = argv[i];
        } else {
            usage(argv[0]);
            return 2;
        }
    }

    if (!port || !action ||
        (std::strcmp(action, "status") != 0 &&
         std::strcmp(action, "on") != 0 &&
         std::strcmp(action, "off") != 0)) {
        usage(argv[0]);
        return 2;
    }
    if (hold && std::strcmp(action, "off") != 0) {
        std::fprintf(stderr, "ERROR: --hold is valid only with the off action\n");
        return 2;
    }

    ILidarDriver* lidar = *createLidarDriver();
    IChannel* channel = *createSerialPortChannel(port, baud);
    if (!lidar || !channel) {
        std::fprintf(stderr, "ERROR: SDK driver/channel allocation failed\n");
        delete lidar;
        delete channel;
        return 3;
    }

    const sl_result connected = lidar->connect(channel);
    if (SL_IS_FAIL(connected)) {
        std::fprintf(stderr, "ERROR: cannot connect to %s (0x%08X)\n",
                     port, connected);
        delete lidar;
        delete channel;
        return 4;
    }

    sl_lidar_response_device_info_t info{};
    sl_lidar_response_device_health_t health{};
    const sl_result info_result = lidar->getDeviceInfo(info);
    const sl_result health_result = lidar->getHealth(health);
    if (SL_IS_FAIL(info_result) || SL_IS_FAIL(health_result)) {
        std::fprintf(stderr,
            "ERROR: port did not answer as a SLAMTEC LiDAR; no motor command sent\n");
        delete lidar;
        delete channel;
        return 5;
    }

    std::printf("serial=%s firmware=%u.%02u hardware=%u health=%s error=%u\n",
        serial_hex(info).c_str(), info.firmware_version >> 8,
        info.firmware_version & 0xffU, info.hardware_version,
        health_name(health.status), health.error_code);

    if (health.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "ERROR: LiDAR health is ERROR; motor command refused\n");
        delete lidar;
        delete channel;
        return 6;
    }

    sl_result result = SL_RESULT_OK;
    if (std::strcmp(action, "on") == 0) {
        result = lidar->setMotorSpeed();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    } else if (std::strcmp(action, "off") == 0) {
        lidar->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        result = lidar->setMotorSpeed(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    if (SL_IS_FAIL(result)) {
        std::fprintf(stderr, "ERROR: motor action '%s' failed (0x%08X)\n",
                     action, result);
        delete lidar;
        delete channel;
        return 7;
    }

    std::printf("motor_action=%s result=OK\n", action);
    std::fflush(stdout);

    // RPLIDAR A1 motor control uses DTR. On some CP2102/Linux combinations,
    // closing the serial descriptor clears DTR and the motor starts again.
    // Holding the verified SDK connection open preserves the OFF state.
    if (hold) {
        std::signal(SIGINT, stop_requested);
        std::signal(SIGTERM, stop_requested);
        std::printf("motor_off_hold=ACTIVE pid=%ld\n",
                    static_cast<long>(::getpid()));
        std::fflush(stdout);
        while (keep_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        lidar->stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        lidar->setMotorSpeed(0);
        std::printf("motor_off_hold=STOPPING\n");
        std::fflush(stdout);
    }

    delete lidar;
    delete channel;
    return 0;
}
