/*
 * Copyright (C) 2026 Viture Inc. All rights reserved.
 */

#include <vector>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#if defined(_WIN32)
#include <windows.h>
#include <cfgmgr32.h>
#include <iomanip>
#include <iostream>
#include <setupapi.h>
#if defined(_MSC_VER)
#pragma comment(lib, "setupapi.lib")
#endif
#elif defined(__APPLE__)
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>
#include <iomanip>
#include <iostream>
#else
#include <dirent.h>
#endif

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define VITURE_GLASSES_SUCCESS 0

#include <hidapi/hidapi.h>
#include "viture_device.h"
#include "viture_device_carina.h"
//include "viture_glasses_constants.h"
#include "viture_glasses_provider.h"
#include "viture_protocol_public.h"


// For this CLI tool, print logs directly to stderr so they appear in the shell
#undef LOGI
#undef LOGE
#define LOGI(...)                          \
    do {                                   \
        std::fprintf(stdout, __VA_ARGS__); \
        std::fprintf(stdout, "\n");        \
    } while (0)
#define LOGE(...)                          \
    do {                                   \
        std::fprintf(stderr, __VA_ARGS__); \
        std::fprintf(stderr, "\n");        \
    } while (0)

volatile std::sig_atomic_t g_should_exit = 0;

std::atomic<bool> gl_pose_thread_running {false};
std::thread gl_pose_thread;
XRDeviceProviderHandle gl_carina_handle = nullptr;

// Global flags for image saving
std::atomic<bool> g_save_next_frame {false};
bool g_stitch_images = true;  // Whether to stitch left/right images

void signal_handler(int signum) {
    if (signum == SIGINT) {
        g_should_exit = 1;
    }
}




void pose_polling_thread() {
    LOGI("Starting pose polling thread");
    float pose[7] = {0};
    const int interval_ms = 10; // 100Hz = 10ms interval
    auto next_time = std::chrono::steady_clock::now();

    while (gl_pose_thread_running && !g_should_exit) {
        // Get current pose with no prediction
        int pose_status = 0;
        int result = xr_device_provider_get_gl_pose_carina(gl_carina_handle, pose, 0.0, &pose_status);

        if (result == VITURE_GLASSES_SUCCESS) {
            LOGI("Pose: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]", pose[0], pose[1], pose[2], pose[3], pose[4],
                 pose[5], pose[6]);
        } else {
            LOGE("Failed to get pose: %d", result);
        }

        // Sleep until next interval
        next_time += std::chrono::milliseconds(interval_ms);
        std::this_thread::sleep_until(next_time);
    }

    LOGI("Pose polling thread stopped");
}

static void ImuRawCallback(float* data, uint64_t ts, uint64_t vsync) {
    LOGI("Imu raw callback - TS: %llu, VSync: %llu, Temp: %.4f, IMU: [gyro: %.4f, "
         "%.4f, %.4f; acc: %.4f, %.4f, %.4f], mag: [%.4f, %.4f, %.4f]",
         ts, vsync, data[9], data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
}

static void ImuPoseCallback(float* data, uint64_t ts) {
    LOGI("Imu pose callback - TS: %llu, euler: [%.4f, %.4f, %.4f] , quat: [%.4f, %.4f, %.4f, %.4f]", ts, data[0],
         data[1], data[2], data[3], data[4], data[5], data[6]);
}

static void PoseCallback(float *pose, double timestamp)
{
    LOGI("Pose callback - TS: %f, point: [%.4f, %.4f, %.4f] , quat: [%.4f, %.4f, %.4f, %.4f] \r\n",
                   timestamp, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]);
}
void VSyncCallback(double timestamp)
{
    LOGI("VSync callback - TS: %f \r\n", timestamp);
}

void ImuCallback(float *imu, double timestamp)
{
    LOGI("Imu callback - TS: %f \r\n", timestamp);
}
void CameraCallback(char *image_left0,
                    char *image_right0,
                    char *image_left1,
                    char *image_right1,
                    double timestamp,
                    int width,
                    int height)
{
    LOGI("Camera callback - TS: %f size: [%df, %df] \r\n", timestamp, width, height);
    static int saved_frame_count = 0;
    //const int MAX_FRAMES = 100;

    // 检查图像尺寸有效性
    if (width <= 0 || height <= 0) {
        LOGE("Invalid image size: %dx%d", width, height);
        return;
    }

    // 只有当按键触时保存  先按下s键,再按下enter键
    if (!g_save_next_frame) {
        return;
    }

    // 为当前帧生成基础文件名（使用帧序号和时间戳，便于识别）
    char base_name[256];
    snprintf(base_name, sizeof(base_name), "/home/soc/frame_%04d_ts_%.3f",
             saved_frame_count, timestamp);

    // 辅助函数：将灰度数据保存为 PNG
    auto save_gray_png = [&](const char* prefix, const char* suffix, char* data) {
        if (!data) {
            LOGE("Null data for %s%s", prefix, suffix);
            return;
        }
        char filename[512];
        snprintf(filename, sizeof(filename), "%s_%s.png", prefix, suffix);
        int success = stbi_write_png(filename, width, height, 1,
                                     (unsigned char*)data, width);
        if (success) {
            LOGI("Saved: %s", filename);
        } else {
            LOGE("Failed to save: %s", filename);
        }
    };

    if (g_stitch_images) {
        // 拼接模式：将左右图像水平拼接
        int combined_width = width * 2;
        int combined_height = height;
        size_t combined_size = combined_width * combined_height;

        std::vector<unsigned char> combined_data(combined_size);

        // 逐行复制：每行先复制 left0，再复制 right0
        for (int y = 0; y < height; ++y) {
            unsigned char* dst_row = combined_data.data() + y * combined_width;
            const unsigned char* src_left_row = reinterpret_cast<const unsigned char*>(image_left0) + y * width;
            const unsigned char* src_right_row = reinterpret_cast<const unsigned char*>(image_right0) + y * width;
            
            memcpy(dst_row, src_left_row, width);
            memcpy(dst_row + width, src_right_row, width);
        }

        char filename[512];
        snprintf(filename, sizeof(filename), "/home/soc/frame_%04d_combined.png", saved_frame_count);

        int success = stbi_write_png(filename, combined_width, combined_height, 1,
                                     combined_data.data(), combined_width);
        if (success) {
            LOGI("Saved combined image: %s (%dx%d)", filename, combined_width, combined_height);
        } else {
            LOGE("Failed to save combined image: %s", filename);
        }
    } else {
        // 非拼接模式：分别保存左右图像
        save_gray_png(base_name, "left", image_left0);
        save_gray_png(base_name, "right", image_right0);
    }

    saved_frame_count++;
    g_save_next_frame = false;  // 重置按键触发标志
}

static void StateCallback(int glass_state_id, int glass_value) {
    LOGI("Received state callback %d, %d", glass_state_id, glass_value);
}

static void DemoLogHook(int level, const char* tag, const char* message) {
    printf("DemoLogHook [%d][%s] %s\n", level, tag, message);
}

#if defined(_WIN32)
#elif defined(__APPLE__)
static bool getUSBProperty(io_service_t service, const char* key, uint16_t* outValue) {
    CFStringRef keyStr = CFStringCreateWithCString(kCFAllocatorDefault, key, kCFStringEncodingUTF8);
    CFTypeRef prop = IORegistryEntryCreateCFProperty(service, keyStr, kCFAllocatorDefault, 0);
    CFRelease(keyStr);
    if (!prop) return false;

    bool success = false;
    if (CFNumberGetTypeID() == CFGetTypeID(prop)) {
        success = CFNumberGetValue((CFNumberRef)prop, kCFNumberShortType, outValue);
    }

    CFRelease(prop);
    return success;
}
#else
// Read integer from sysfs file
static bool read_int_from_file(const char* path, int* out_value) {
    if (!path || !out_value) return false;
    FILE* f = std::fopen(path, "r");
    if (!f) return false;
    char buf[64] {};
    if (!std::fgets(buf, sizeof(buf), f)) {
        std::fclose(f);
        return false;
    }
    std::fclose(f);
    // Trim whitespace/newline
    size_t len = std::strlen(buf);
    while (len > 0 && (buf[len - 1] == '\n' || buf[len - 1] == '\r' || buf[len - 1] == ' ' || buf[len - 1] == '\t')) {
        buf[--len] = '\0';
    }
    char* endptr = nullptr;
    long v = std::strtol(buf, &endptr, 16);
    if (endptr == buf) return false;
    *out_value = static_cast<int>(v);
    return true;
}
#endif

// Try to find a VITURE USB device in sysfs and return fd, bus, addr
static bool autodetect_viture_usb_fallback(std::vector<int>& out_pids) {
#if defined(_WIN32)
    // Use SetupAPI to enumerate present devices and read their hardware IDs
    // (REG_MULTI_SZ) from the registry. Parse strings like "USB\\VID_35CA&PID_1234"
    // and add matching PIDs for vendor 0x35CA.
    HDEVINFO devInfo = SetupDiGetClassDevsA(NULL, "USB", NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    if (devInfo == INVALID_HANDLE_VALUE) {
        return false;
    }

    SP_DEVINFO_DATA devInfoData;
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(devInfo, i, &devInfoData); ++i) {
        DWORD regType = 0;
        BYTE buffer[4096] = {0};
        if (SetupDiGetDeviceRegistryPropertyA(devInfo, &devInfoData, SPDRP_HARDWAREID, &regType, buffer, sizeof(buffer),
                                              NULL)) {
            // buffer is a multi-string (multiple null-terminated strings ending with an extra null)
            const char* p = reinterpret_cast<const char*>(buffer);
            while (*p) {
                const char* vidPos = strstr(p, "VID_");
                const char* pidPos = strstr(p, "PID_");
                if (vidPos && pidPos) {
                    unsigned int vid = 0, pid = 0;
                    if (sscanf(vidPos + 4, "%4x", &vid) == 1 && sscanf(pidPos + 4, "%4x", &pid) == 1) {
                        if (vid == 0x35CA) {
                            if (std::find(out_pids.begin(), out_pids.end(), static_cast<int>(pid)) == out_pids.end()) {
                                out_pids.push_back(static_cast<int>(pid));
                            }
                        }
                    }
                }
                p += std::strlen(p) + 1;
            }
        }
    }

    SetupDiDestroyDeviceInfoList(devInfo);
    return !out_pids.empty();
#elif defined(__APPLE__)
    kern_return_t kr;
    CFMutableDictionaryRef matchingDict = IOServiceMatching(kIOUSBDeviceClassName);
    if (!matchingDict) return false;

    io_iterator_t iterator = 0;
    kr = IOServiceGetMatchingServices(0, matchingDict, &iterator);
    if (kr != KERN_SUCCESS || iterator == 0) {
        return false;
    }

    const uint16_t targetVendorID = 0x35CA;
    io_service_t service;
    int deviceCount = 0;
    int matchedCount = 0;

    while ((service = IOIteratorNext(iterator))) {
        deviceCount++;
        uint16_t vid = 0, pid = 0;
        bool hasVid = getUSBProperty(service, kUSBVendorID, &vid);
        bool hasPid = getUSBProperty(service, kUSBProductID, &pid);

        // Filter by vendor ID
        if (hasVid && vid == targetVendorID && hasPid) {
            if (std::find(out_pids.begin(), out_pids.end(), pid) == out_pids.end()) {
                out_pids.push_back(pid);
                matchedCount++;
            }
        }

        IOObjectRelease(service);
    }
    IOObjectRelease(iterator);
    return !out_pids.empty();
#else
    const char* sys_usb = "/sys/bus/usb/devices";
    DIR* dir = opendir(sys_usb);
    if (!dir) return false;
    struct dirent* ent;
    while ((ent = readdir(dir)) != nullptr) {
        if (ent->d_name[0] == '.') continue;
        // Entries like 1-1, 2-1.2, etc. Filter interfaces (contain ':')
        if (std::strchr(ent->d_name, ':')) continue;
        char path[512];
        std::snprintf(path, sizeof(path), "%s/%s", sys_usb, ent->d_name);

        char idVendorPath[600];
        char idProductPath[600];
        std::snprintf(idVendorPath, sizeof(idVendorPath), "%s/idVendor", path);
        std::snprintf(idProductPath, sizeof(idProductPath), "%s/idProduct", path);

        int vid = 0, pid = 0, bus = 0, addr = 0;
        if (!read_int_from_file(idVendorPath, &vid)) {
            continue;
        }
        if (!read_int_from_file(idProductPath, &pid)) {
            continue;
        }
        // Match VITURE vendor (0x35CA). Accept any PID.
        if (vid != 0x35CA) {
            continue;
        }

        if (std::find(out_pids.begin(), out_pids.end(), pid) == out_pids.end()) {
            out_pids.push_back(pid);
        }
    }
    closedir(dir);
    if (out_pids.empty()) {
        return false;
    }
    return true;
#endif
}

// Try to find all VITURE USB devices using hidapi (cross-platform)
static bool autodetect_viture_usb(std::vector<int>& out_pids) {
    if (hid_init() != 0) {
        LOGE("hid_init failed");
        return false;
    }

    hid_device_info* devices = hid_enumerate(0, 0);
    if (!devices) {
        hid_exit();
        return false;
    }

    for (hid_device_info* cur = devices; cur != nullptr; cur = cur->next) {
        // Ensure vendor matches Viture
        int vid = static_cast<int>(cur->vendor_id);
        if (vid != 0x35CA) continue;
        int pid = static_cast<int>(cur->product_id);
        if (std::find(out_pids.begin(), out_pids.end(), pid) == out_pids.end()) {
            out_pids.push_back(pid);
        }
    }

    hid_free_enumeration(devices);
    hid_exit();

    if (out_pids.empty()) {
        return false;
    }
    return true;
}

static bool read_int_from_stdin(const std::string& prompt, int* out_value) {
    if (!out_value) return false;
    while (true) {
        LOGI("%s", prompt.c_str());
        std::string line;
        if (!std::getline(std::cin, line)) {
            return false;
        }
        
        // Check for special keys first
        if (line == "q" || line == "Q") {
            return false;
        }
        
        // Check for save command
        if (line == "s" || line == "S") {
            g_save_next_frame = true;
            LOGI("Will save the next incoming frame...");
            continue;
        }
        
        if (line.empty()) {
            LOGI("Invalid input, please retry...");
            continue;
        }
        char* end_ptr = nullptr;
        long value = std::strtol(line.c_str(), &end_ptr, 10);
        if (*end_ptr != '\0') {
            LOGI("Invalid character detected, please input integer only");
            continue;
        }
        *out_value = static_cast<int>(value);
        return true;
    }
}

static bool prompt_for_bounded_int(const std::string& prompt, int min_value, int max_value, int* out_value) {
    while (true) {
        int value = 0;
        if (!read_int_from_stdin(prompt, &value)) {
            return false;
        }
        if (value < min_value || value > max_value) {
            LOGI("Please input an integer in range [%d, %d]", min_value, max_value);
            continue;
        }
        *out_value = value;
        return true;
    }
}

static void run_interactive_menu(XRDeviceProviderHandle handle) {
    while (!g_should_exit) {
        LOGI("\n========= Interactive menu =========\n"
             "1. Switch electrochromic film mode\n"
             "2. Switch display mode\n"
             "3. Set brightness\n"
             "4. Set volume\n"
             "5. Open IMU\n"
             "6. Close IMU\n"
             "7. Set log level (0 - 3)\n"
             "8. Save next frame\n");

        int option = 0;
        if (!prompt_for_bounded_int("Please input selection (q for exit): ", 1, 8, &option)) {
            LOGE("Exiting...");
            g_should_exit = 1;
            break;
        }

        switch (option) {
            case 1: {
                int film_choice = 0;
                if (!prompt_for_bounded_int("Please input electrochromic file status, 0 for Off, 1 for On: ", 0, 1,
                                            &film_choice)) {
                    LOGE("Exiting...");
                    g_should_exit = 1;
                    return;
                }
                float voltage = (film_choice == 1) ? 1.0f : 0.0f;
                int result = xr_device_provider_set_film_mode(handle, voltage);
                if (result == VITURE_GLASSES_SUCCESS) {
                    LOGI("Success");
                } else {
                    LOGE("Failed: %d", result);
                }
                break;
            }
            case 2: {
                int mode_choice = 0;
                if (!prompt_for_bounded_int("Please select display mode: 1. 1920x1080  2. 3840x1080\nPlease Input: ", 1,
                                            2, &mode_choice)) {
                    LOGI("Exiting...");
                    g_should_exit = 1;
                    return;
                }
                int target_mode =
                    (mode_choice == 1) ? VITURE_DISPLAY_MODE_1920_1080_60HZ : VITURE_DISPLAY_MODE_3840_1080_60HZ;
                int result = xr_device_provider_set_display_mode(handle, target_mode);
                if (result == VITURE_GLASSES_SUCCESS) {
                    LOGI("Success");
                } else {
                    LOGE("Failed: %d", result);
                }
                break;
            }
            case 3: {
                int device_type = xr_device_provider_get_device_type(handle);
                int brightness = 0;
                if (!prompt_for_bounded_int("Please input brightness level [0, 8]: ", 0, 8, &brightness)) {
                    LOGE("Exiting...\n");
                    g_should_exit = 1;
                    return;
                }
                int result = xr_device_provider_set_brightness_level(handle, brightness);
                if (result == VITURE_GLASSES_SUCCESS) {
                    LOGI("Success");
                } else {
                    LOGE("Failed: %d", result);
                }
                break;
            }
            case 4: {
                int volume = 0;
                if (!prompt_for_bounded_int("Please input volume level [0, 8]: ", 0, 8, &volume)) {
                    LOGE("Exiting...\n");
                    g_should_exit = 1;
                    return;
                }
                int result = xr_device_provider_set_volume_level(handle, volume);
                if (result == VITURE_GLASSES_SUCCESS) {
                    LOGI("Success");
                } else {
                    LOGE("Failed: %d", result);
                }
                break;
            }
            case 5: {
                int device_type = xr_device_provider_get_device_type(handle);
                if (device_type == XR_DEVICE_TYPE_VITURE_GEN1 || device_type == XR_DEVICE_TYPE_VITURE_GEN2) {
                    LOGI("Opening IMU for non-carina device");
                    xr_device_provider_open_imu(handle, VITURE_IMU_MODE_POSE, VITURE_IMU_FREQUENCY_HIGH);
                } else {
                    LOGI("Opening IMU for carina device");
                    // gl_pose_thread_running = true;
                    // gl_pose_thread = std::thread(pose_polling_thread);
                }
                break;
            }
            case 6: {
                int device_type = xr_device_provider_get_device_type(handle);
                if (device_type == XR_DEVICE_TYPE_VITURE_GEN1 || device_type == XR_DEVICE_TYPE_VITURE_GEN2) {
                    LOGI("Closing IMU for non-carina device");
                    xr_device_provider_close_imu(handle, VITURE_IMU_MODE_POSE);
                } else {
                    LOGE("Closing IMU for carina device");
                    if (gl_pose_thread.joinable()) {
                        gl_pose_thread_running = false;
                        gl_pose_thread.join();
                    }
                }
                break;
            }
            case 7: {
                int level = 0;
                if (!prompt_for_bounded_int("Please input log level [0, 3]: ", 0, 3, &level)) {
                    LOGE("Exiting...\n");
                    g_should_exit = 1;
                    return;
                }
                xr_device_provider_set_log_level(level);
                break;
            }
            case 8: {
                g_save_next_frame = true;
                LOGI("Will save the next incoming frame...");
                break;
            }
            default:
                break;
        }
    }
}


int main(int argc, char* argv[]) {
    LOGI("Demo starting...");

    // Parse command line arguments
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--no-stitch" || std::string(argv[i]) == "-n") {
            g_stitch_images = false;
            LOGI("Image stitching disabled - will save left/right images separately");
        } else if (std::string(argv[i]) == "--stitch" || std::string(argv[i]) == "-s") {
            g_stitch_images = true;
            LOGI("Image stitching enabled - will save combined images");
        } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
            LOGI("Usage: %s [options]", argv[0]);
            LOGI("Options:");
            LOGI("  -h, --help    Show this help message");
            LOGI("  -s, --stitch  Enable image stitching (default)");
            LOGI("  -n, --no-stitch  Disable image stitching, save left/right separately");
            return 0;
        }
    }

    // Register signal handler for Ctrl+C
    std::signal(SIGINT, signal_handler);

    std::vector<int> detected_pids;
    if (!autodetect_viture_usb(detected_pids)) {
        LOGI("No Viture devices detected with hidapi");
    }

    if (!autodetect_viture_usb_fallback(detected_pids)) {
        LOGI("No Viture devices detected with native fallback");
    }

    if (detected_pids.empty()) {
        LOGE("No Viture devices detected");
        return 0;
    }

    LOGI("Detected Viture devices:");
    for (size_t i = 0; i < detected_pids.size(); ++i) {
        int is_valid = xr_device_provider_is_product_id_valid(detected_pids[i]);
        LOGI(" %zu. PID = 0x%04X (%s)", (i + 1), detected_pids[i], (is_valid ? "Valid" : "Not valid"));
    }

    int selection = 0;
    if (!prompt_for_bounded_int("Please select a device to connect: ", 1, static_cast<int>(detected_pids.size()),
                                &selection)) {
        LOGE("No device selected");
        return 0;
    }

    int pid = detected_pids[static_cast<size_t>(selection - 1)];
    LOGI("Selected device with PID=0x%04X", pid);

    xr_device_provider_set_log_hook(DemoLogHook);

    XRDeviceProviderHandle handle = xr_device_provider_create(pid);
    if (!handle) {
        LOGE("Failed to create XRDeviceProvider");
        return 3;
    }
    int device_type = xr_device_provider_get_device_type(handle);

    // Register callbacks for Carina device
    if (device_type == XR_DEVICE_TYPE_VITURE_CARINA) {
        gl_carina_handle = handle;
        xr_device_provider_register_callbacks_carina(handle, PoseCallback, VSyncCallback, ImuCallback, CameraCallback);
    } else if (device_type == XR_DEVICE_TYPE_VITURE_GEN1 || device_type == XR_DEVICE_TYPE_VITURE_GEN2) {
        // xr_device_provider_register_imu_raw_callback(handle, ImuRawCallback);
        xr_device_provider_register_imu_pose_callback(handle, ImuPoseCallback);
    } else {
        LOGI("Neither Carina nor Viture device, skipping callback registration");
    }

    if (xr_device_provider_initialize(handle, nullptr, nullptr) != VITURE_GLASSES_SUCCESS) {
        LOGE("Failed to initialize provider");
        xr_device_provider_destroy(handle);
        return 4;
    }

    if (xr_device_provider_start(handle) != VITURE_GLASSES_SUCCESS) {
        LOGE("Failed to start provider");
        xr_device_provider_shutdown(handle);
        xr_device_provider_destroy(handle);
        return 5;
    }

    xr_device_provider_register_state_callback(handle, StateCallback);
    LOGI("Device successfully initialized...");

    char buffer[64] = {0};
    int length = sizeof(buffer);
    int version_result = xr_device_provider_get_glasses_version(handle, buffer, &length);
    LOGI("Glasses firmware version %s", buffer);
    memset(buffer, 0, sizeof(buffer));
    length = sizeof(buffer);
    int name_result = xr_device_provider_get_market_name(pid, buffer, &length);
    LOGI("Glasses market name %s", buffer);

    run_interactive_menu(handle);
    LOGI("Interactive session finished, stopping device...");
    xr_device_provider_register_state_callback(handle, nullptr);

    // Cleanup
    if (xr_device_provider_stop(handle) != VITURE_GLASSES_SUCCESS) {
        LOGE("Stop returned error");
    }
    if (xr_device_provider_shutdown(handle) != VITURE_GLASSES_SUCCESS) {
        LOGE("Shutdown returned error");
    }
    xr_device_provider_destroy(handle);
    LOGI("Bye bye...");
    return 0;
}