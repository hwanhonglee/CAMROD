#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

// Quick workflow (Orbbec):
//  1) Check selected device intrinsics/noise:
//     ./orbbec_sensor_inspector --print_intrinsics [--serial <SN>]
//  2) Validate live stereo+IMU timestamp behavior:
//     ./orbbec_sensor_inspector --run --frames 300 [--disable_global_timestamp]
//  3) Export Kimera sensor YAMLs (camera/imu only):
//     ./orbbec_sensor_inspector --dump_kimera_yaml --output_dir <dir>
//  4) Use that output as sensor section when preparing Kimera params folder.
struct Options {
  enum class Mode {
    kNone,
    kDescribe,
    kPrintIntrinsics,
    kRun,
    kDumpKimeraYaml,
  } mode = Mode::kNone;

  std::string serial_number;
  std::string device_address;
  int device_port = 8090;
  int width = 0;   // 0 means use device default.
  int height = 0;  // 0 means use device default.
  int fps = 0;     // 0 means use device default.
  bool use_global_timestamp = true;
  std::filesystem::path output_dir = "./orbbec_sensor_yaml";
  int frames = 0;  // 0 means run forever.
};

void PrintUsage(const char* argv0) {
  std::cout
      << "Orbbec Sensor Inspector\n"
      << "\n"
      << "Purpose:\n"
      << "  - Check Orbbec stereo/IMU intrinsics quickly\n"
      << "  - Verify stereo image+IMU timestamps\n"
      << "  - Dump Kimera-compatible sensor YAML files\n"
      << "\n"
      << "Modes (pick one):\n"
      << "  --describe\n"
      << "  --print_intrinsics\n"
      << "  --run\n"
      << "  --dump_kimera_yaml\n"
      << "\n"
      << "Options:\n"
      << "  --serial <SN>        select a specific device\n"
      << "  --address <IPv4>     connect to a network device directly\n"
      << "  --port <int>         network device port (default: 8090)\n"
      << "  --width <int>        video width (default: device default)\n"
      << "  --height <int>       video height (default: device default)\n"
      << "  --fps <int>          video fps (default: device default)\n"
      << "  --frames <N>         stop after N grabs (default: 0, infinite)\n"
      << "  --output_dir <path>  dump folder for YAML files\n"
      << "  --disable_global_timestamp\n"
      << "\n"
      << "Examples:\n"
      << "  "
      << argv0
      << " --describe\n"
      << "  "
      << argv0
      << " --print_intrinsics\n"
      << "  "
      << argv0
      << " --run --frames 300\n"
      << "  "
      << argv0
      << " --dump_kimera_yaml --output_dir ./tmp/orbbec_sensor\n";
}

bool ParseInt(const std::string& text, int* out) {
  try {
    *out = std::stoi(text);
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);

    if (arg == "--help" || arg == "-h") {
      return false;
    }

    if (arg == "--describe") {
      options->mode = Options::Mode::kDescribe;
      continue;
    }
    if (arg == "--print_intrinsics" || arg == "--print-intrinsics") {
      options->mode = Options::Mode::kPrintIntrinsics;
      continue;
    }
    if (arg == "--run") {
      options->mode = Options::Mode::kRun;
      continue;
    }
    if (arg == "--dump_kimera_yaml" || arg == "--dump-kimera-yaml") {
      options->mode = Options::Mode::kDumpKimeraYaml;
      continue;
    }

    if (arg == "--disable_global_timestamp") {
      options->use_global_timestamp = false;
      continue;
    }

    if (arg == "--serial" && i + 1 < argc) {
      options->serial_number = argv[++i];
      continue;
    }
    if (arg == "--address" && i + 1 < argc) {
      options->device_address = argv[++i];
      continue;
    }
    if (arg == "--port" && i + 1 < argc) {
      if (!ParseInt(argv[++i], &options->device_port)) {
        std::cerr << "[ERR] Invalid --port value\n";
        return false;
      }
      continue;
    }
    if (arg == "--width" && i + 1 < argc) {
      if (!ParseInt(argv[++i], &options->width)) {
        std::cerr << "[ERR] Invalid --width value\n";
        return false;
      }
      continue;
    }
    if (arg == "--height" && i + 1 < argc) {
      if (!ParseInt(argv[++i], &options->height)) {
        std::cerr << "[ERR] Invalid --height value\n";
        return false;
      }
      continue;
    }
    if (arg == "--fps" && i + 1 < argc) {
      if (!ParseInt(argv[++i], &options->fps)) {
        std::cerr << "[ERR] Invalid --fps value\n";
        return false;
      }
      continue;
    }
    if (arg == "--frames" && i + 1 < argc) {
      if (!ParseInt(argv[++i], &options->frames)) {
        std::cerr << "[ERR] Invalid --frames value\n";
        return false;
      }
      continue;
    }
    if (arg == "--output_dir" && i + 1 < argc) {
      options->output_dir = argv[++i];
      continue;
    }

    std::cerr << "[ERR] Unknown argument: " << arg << "\n";
    return false;
  }

  if (options->mode == Options::Mode::kNone) {
    std::cerr << "[ERR] You must choose one mode.\n";
    return false;
  }
  if (options->width < 0 || options->height < 0 || options->fps < 0 ||
      options->frames < 0) {
    std::cerr << "[ERR] width/height/fps/frames must be >= 0\n";
    return false;
  }
  if (options->device_port <= 0 || options->device_port > 65535) {
    std::cerr << "[ERR] port must be in [1, 65535]\n";
    return false;
  }
  return true;
}

OBFrameType SensorTypeToFrameType(const OBSensorType type) {
  switch (type) {
    case OB_SENSOR_COLOR:
      return OB_FRAME_COLOR;
    case OB_SENSOR_COLOR_LEFT:
      return OB_FRAME_COLOR_LEFT;
    case OB_SENSOR_COLOR_RIGHT:
      return OB_FRAME_COLOR_RIGHT;
    case OB_SENSOR_IR:
      return OB_FRAME_IR;
    case OB_SENSOR_IR_LEFT:
      return OB_FRAME_IR_LEFT;
    case OB_SENSOR_IR_RIGHT:
      return OB_FRAME_IR_RIGHT;
    default:
      return OB_FRAME_UNKNOWN;
  }
}

double ImuSampleRateToHz(const OBIMUSampleRate rate) {
  switch (rate) {
    case OB_SAMPLE_RATE_1_5625_HZ:
      return 1.5625;
    case OB_SAMPLE_RATE_3_125_HZ:
      return 3.125;
    case OB_SAMPLE_RATE_6_25_HZ:
      return 6.25;
    case OB_SAMPLE_RATE_12_5_HZ:
      return 12.5;
    case OB_SAMPLE_RATE_25_HZ:
      return 25.0;
    case OB_SAMPLE_RATE_50_HZ:
      return 50.0;
    case OB_SAMPLE_RATE_100_HZ:
      return 100.0;
    case OB_SAMPLE_RATE_200_HZ:
      return 200.0;
    case OB_SAMPLE_RATE_400_HZ:
      return 400.0;
    case OB_SAMPLE_RATE_500_HZ:
      return 500.0;
    case OB_SAMPLE_RATE_800_HZ:
      return 800.0;
    case OB_SAMPLE_RATE_1_KHZ:
      return 1000.0;
    case OB_SAMPLE_RATE_2_KHZ:
      return 2000.0;
    case OB_SAMPLE_RATE_4_KHZ:
      return 4000.0;
    case OB_SAMPLE_RATE_8_KHZ:
      return 8000.0;
    case OB_SAMPLE_RATE_16_KHZ:
      return 16000.0;
    case OB_SAMPLE_RATE_32_KHZ:
      return 32000.0;
    case OB_SAMPLE_RATE_UNKNOWN:
    default:
      return 0.0;
  }
}

uint64_t FrameTimestampNs(const std::shared_ptr<ob::Frame>& frame,
                          const bool use_global_timestamp) {
  if (!frame) return 0;
  if (use_global_timestamp) {
    const uint64_t ts_global_us = frame->getGlobalTimeStampUs();
    if (ts_global_us > 0) return ts_global_us * 1000ULL;
  }
  const uint64_t ts_system_us = frame->getSystemTimeStampUs();
  if (ts_system_us > 0) return ts_system_us * 1000ULL;
  return frame->getTimeStampUs() * 1000ULL;
}

struct SensorSelection {
  OBSensorType left_sensor = OB_SENSOR_UNKNOWN;
  OBSensorType right_sensor = OB_SENSOR_UNKNOWN;
};

bool IsIrStereoSensor(const OBSensorType type) {
  return type == OB_SENSOR_IR_LEFT || type == OB_SENSOR_IR_RIGHT;
}

bool IsGemini335LeEthernet(const std::shared_ptr<ob::Device>& device) {
  if (!device) return false;
  const auto info = device->getDeviceInfo();
  std::string name = info->getName() ? info->getName() : "";
  std::string connection =
      info->getConnectionType() ? info->getConnectionType() : "";
  std::transform(name.begin(),
                 name.end(),
                 name.begin(),
                 [](const unsigned char c) { return std::tolower(c); });
  std::transform(connection.begin(),
                 connection.end(),
                 connection.begin(),
                 [](const unsigned char c) { return std::tolower(c); });
  const std::string ip = info->getIpAddress() ? info->getIpAddress() : "";
  return name.find("335le") != std::string::npos &&
         (connection.find("ethernet") != std::string::npos ||
          (!ip.empty() && ip != "0.0.0.0"));
}

struct ProfileRequest {
  int width = 0;
  int height = 0;
  int fps = 0;
  OBFormat format = OB_FORMAT_ANY;
  bool using_safe_defaults = false;
};

ProfileRequest ResolveProfileRequest(const std::shared_ptr<ob::Device>& device,
                                     const OBSensorType sensor_type,
                                     const Options& options) {
  ProfileRequest req;
  req.width = options.width;
  req.height = options.height;
  req.fps = options.fps;

  const bool fully_explicit =
      options.width > 0 && options.height > 0 && options.fps > 0;
  if (IsGemini335LeEthernet(device) && !fully_explicit) {
    req.width = 640;
    req.height = 400;
    req.fps = 15;
    req.format = IsIrStereoSensor(sensor_type) ? OB_FORMAT_Y8 : OB_FORMAT_MJPG;
    req.using_safe_defaults = true;
  }

  return req;
}

SensorSelection SelectStereoSensors(const std::shared_ptr<ob::Device>& device) {
  SensorSelection out;
  if (device->getSensor(OB_SENSOR_IR_LEFT) &&
      device->getSensor(OB_SENSOR_IR_RIGHT)) {
    out.left_sensor = OB_SENSOR_IR_LEFT;
    out.right_sensor = OB_SENSOR_IR_RIGHT;
    return out;
  }
  if (device->getSensor(OB_SENSOR_COLOR_LEFT) &&
      device->getSensor(OB_SENSOR_COLOR_RIGHT)) {
    out.left_sensor = OB_SENSOR_COLOR_LEFT;
    out.right_sensor = OB_SENSOR_COLOR_RIGHT;
    return out;
  }
  throw std::runtime_error(
      "No stereo pair found. Need IR_LEFT+IR_RIGHT or COLOR_LEFT+COLOR_RIGHT.");
}

std::shared_ptr<ob::VideoStreamProfile> SelectVideoProfile(
    const std::shared_ptr<ob::Device>& device,
    const std::shared_ptr<ob::Pipeline>& pipeline,
    const OBSensorType sensor_type,
    const Options& options) {
  const auto request = ResolveProfileRequest(device, sensor_type, options);
  auto profiles = pipeline->getStreamProfileList(sensor_type);
  return profiles->getVideoStreamProfile(
      request.width > 0 ? request.width : OB_WIDTH_ANY,
      request.height > 0 ? request.height : OB_HEIGHT_ANY,
      request.format,
      request.fps > 0 ? request.fps : OB_FPS_ANY);
}

std::shared_ptr<ob::AccelStreamProfile> SelectAccelProfile(
    const std::shared_ptr<ob::Device>& device) {
  auto accel_sensor = device->getSensor(OB_SENSOR_ACCEL);
  if (!accel_sensor) return nullptr;
  auto profiles = accel_sensor->getStreamProfileList();
  return profiles->getAccelStreamProfile(OB_ACCEL_FULL_SCALE_RANGE_ANY,
                                         OB_ACCEL_SAMPLE_RATE_ANY);
}

std::shared_ptr<ob::GyroStreamProfile> SelectGyroProfile(
    const std::shared_ptr<ob::Device>& device) {
  auto gyro_sensor = device->getSensor(OB_SENSOR_GYRO);
  if (!gyro_sensor) return nullptr;
  auto profiles = gyro_sensor->getStreamProfileList();
  return profiles->getGyroStreamProfile(OB_GYRO_FULL_SCALE_RANGE_ANY,
                                        OB_GYRO_SAMPLE_RATE_ANY);
}

const char* AccessModeName(const OBDeviceAccessMode mode) {
  switch (mode) {
    case OB_DEVICE_CONTROL_ACCESS:
      return "control";
    case OB_DEVICE_DEFAULT_ACCESS:
      return "default";
    case OB_DEVICE_EXCLUSIVE_ACCESS:
      return "exclusive";
    case OB_DEVICE_MONITOR_ACCESS:
      return "monitor";
    default:
      return "unknown";
  }
}

struct OpenDeviceResult {
  std::shared_ptr<ob::Device> device;
  OBDeviceAccessMode access_mode = OB_DEVICE_DEFAULT_ACCESS;
};

OpenDeviceResult OpenDevice(ob::Context* context, const Options& options) {
  context->enableNetDeviceEnumeration(true);
  const OBDeviceAccessMode access_modes[] = {
      OB_DEVICE_CONTROL_ACCESS,
      OB_DEVICE_DEFAULT_ACCESS,
      OB_DEVICE_EXCLUSIVE_ACCESS,
      OB_DEVICE_MONITOR_ACCESS,
  };

  if (!options.device_address.empty()) {
    std::string last_error;
    for (const auto access_mode : access_modes) {
      try {
        auto device = context->createNetDevice(
            options.device_address.c_str(),
            static_cast<uint16_t>(options.device_port),
            access_mode);
        if (!device) continue;
        std::cout << "[INFO] Orbbec access mode: "
                  << AccessModeName(access_mode) << "\n";
        if (options.use_global_timestamp &&
            device->isGlobalTimestampSupported()) {
          try {
            device->enableGlobalTimestamp(true);
          } catch (const std::exception& e) {
            std::cerr << "[WARN] Failed to enable global timestamp with access="
                      << AccessModeName(access_mode) << ": " << e.what()
                      << "\n";
          }
        }
        return {device, access_mode};
      } catch (const ob::Error& e) {
        last_error = std::string(e.what()) + " (access=" +
                     AccessModeName(access_mode) + ")";
      }
    }
    throw std::runtime_error("Failed to open Orbbec network device by IP. " +
                             last_error);
  }

  auto list = context->queryDeviceList();
  if (!list || list->getCount() == 0) {
    throw std::runtime_error(
        "No Orbbec device found. If this is an Ethernet model such as Gemini "
        "335Le, run the container with host networking and try "
        "--address <device_ip> --port 8090.");
  }

  std::string last_error;
  for (const auto access_mode : access_modes) {
    try {
      std::shared_ptr<ob::Device> device;
      if (options.serial_number.empty()) {
        device = list->getDevice(0, access_mode);
      } else {
        device = list->getDeviceBySN(options.serial_number.c_str(), access_mode);
      }
      if (!device) continue;

      std::cout << "[INFO] Orbbec access mode: "
                << AccessModeName(access_mode) << "\n";

      if (options.use_global_timestamp &&
          device->isGlobalTimestampSupported()) {
        try {
          device->enableGlobalTimestamp(true);
        } catch (const std::exception& e) {
          std::cerr << "[WARN] Failed to enable global timestamp with access="
                    << AccessModeName(access_mode) << ": " << e.what()
                    << "\n";
        }
      }

      if (IsGemini335LeEthernet(device)) {
        try {
          const int bandwidth_type =
              device->getIntProperty(OB_PROP_NETWORK_BANDWIDTH_TYPE_INT);
          std::cout << "[INFO] Network bandwidth type: " << bandwidth_type
                    << "\n";
        } catch (...) {
        }
        try {
          device->setIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT,
                                 HIGH_PERFORMANCE_MODE);
          std::cout << "[INFO] Performance mode: HIGH_PERFORMANCE_MODE\n";
        } catch (const std::exception& e) {
          std::cerr << "[WARN] Failed to set performance mode with access="
                    << AccessModeName(access_mode) << ": " << e.what()
                    << "\n";
        }
      }

      return {device, access_mode};
    } catch (const ob::Error& e) {
      last_error = std::string(e.what()) + " (access=" +
                   AccessModeName(access_mode) + ")";
    }
  }
  throw std::runtime_error("Failed to open Orbbec device. " + last_error);
}

std::string ToLower(std::string value) {
  std::transform(value.begin(),
                 value.end(),
                 value.begin(),
                 [](const unsigned char c) { return std::tolower(c); });
  return value;
}

const char* SensorTypeName(const OBSensorType type) {
  switch (type) {
    case OB_SENSOR_IR_LEFT:
      return "IR_LEFT";
    case OB_SENSOR_IR_RIGHT:
      return "IR_RIGHT";
    case OB_SENSOR_COLOR_LEFT:
      return "COLOR_LEFT";
    case OB_SENSOR_COLOR_RIGHT:
      return "COLOR_RIGHT";
    default:
      return "UNKNOWN";
  }
}

void PrintDeviceSummary(const std::shared_ptr<ob::Device>& device,
                        const SensorSelection& stereo,
                        const Options& options) {
  const auto info = device->getDeviceInfo();
  const std::string name = info->getName() ? info->getName() : "";
  const std::string serial =
      info->getSerialNumber() ? info->getSerialNumber() : "";
  const std::string connection =
      info->getConnectionType() ? info->getConnectionType() : "";
  const std::string ip = info->getIpAddress() ? info->getIpAddress() : "";
  const std::string lower_name = ToLower(name);
  const std::string lower_connection = ToLower(connection);
  const bool is_ethernet =
      lower_connection.find("ethernet") != std::string::npos ||
      lower_name.find("335le") != std::string::npos ||
      lower_name.find("435le") != std::string::npos;

  std::cout << "[INFO] Device: " << name << " SN=" << serial << "\n";
  if (!connection.empty()) {
    std::cout << "[INFO] Connection: " << connection;
    if (!ip.empty() && ip != "0.0.0.0") {
      std::cout << " IP=" << ip;
    }
    std::cout << "\n";
  }
  std::cout << "[INFO] Stereo source: " << SensorTypeName(stereo.left_sensor)
            << " + " << SensorTypeName(stereo.right_sensor)
            << " (auto-selected)\n";
  std::cout << "[INFO] IMU: "
            << ((device->getSensor(OB_SENSOR_ACCEL) &&
                 device->getSensor(OB_SENSOR_GYRO))
                    ? "available"
                    : "not available")
            << "\n";

  if (is_ethernet) {
    std::cout
        << "[INFO] Model hint: Ethernet Orbbec detected. Recommended path is "
           "host-network container + auto discovery, or direct IP with "
           "--address <device_ip> --port "
        << options.device_port << ".\n";
    if (IsGemini335LeEthernet(device) &&
        !(options.width > 0 && options.height > 0 && options.fps > 0)) {
      std::cout
          << "[INFO] Profile hint: Gemini 335Le Ethernet defaults to "
             "640x400@15 for stable live stereo unless width/height/fps are "
             "all set explicitly.\n";
    }
  } else {
    std::cout
        << "[INFO] Model hint: non-Ethernet Orbbec detected. Serial-based "
           "selection should be sufficient.\n";
  }
}

void PrintIntrinsics(const std::shared_ptr<ob::Pipeline>& video_pipeline,
                     const std::shared_ptr<ob::Device>& device,
                     const SensorSelection& stereo,
                     const Options& options) {
  const auto left_profile =
      SelectVideoProfile(device, video_pipeline, stereo.left_sensor, options);
  const auto right_profile =
      SelectVideoProfile(device, video_pipeline, stereo.right_sensor, options);

  const auto print_video = [](const char* name,
                              const std::shared_ptr<ob::VideoStreamProfile>& p) {
    const auto in = p->getIntrinsic();
    const auto d = p->getDistortion();
    std::cout << "===== " << name << " =====\n";
    std::cout << "size: " << in.width << "x" << in.height << "\n";
    std::cout << "fps: " << p->getFps() << "\n";
    std::cout << "fx: " << in.fx << "\n";
    std::cout << "fy: " << in.fy << "\n";
    std::cout << "cx: " << in.cx << "\n";
    std::cout << "cy: " << in.cy << "\n";
    std::cout << "k1: " << d.k1 << "\n";
    std::cout << "k2: " << d.k2 << "\n";
    std::cout << "p1: " << d.p1 << "\n";
    std::cout << "p2: " << d.p2 << "\n";
    std::cout << "k3: " << d.k3 << "\n";
    std::cout << "k4: " << d.k4 << "\n";
  };

  print_video("LEFT CAMERA INTRINSICS", left_profile);
  print_video("RIGHT CAMERA INTRINSICS", right_profile);

  auto accel_profile = SelectAccelProfile(device);
  auto gyro_profile = SelectGyroProfile(device);
  if (accel_profile) {
    const auto in = accel_profile->getIntrinsic();
    std::cout << "===== ACCEL INTRINSIC =====\n";
    std::cout << "sample_rate_hz: "
              << ImuSampleRateToHz(accel_profile->getSampleRate()) << "\n";
    std::cout << "noise_density: " << in.noiseDensity << "\n";
    std::cout << "random_walk: " << in.randomWalk << "\n";
  }
  if (gyro_profile) {
    const auto in = gyro_profile->getIntrinsic();
    std::cout << "===== GYRO INTRINSIC =====\n";
    std::cout << "sample_rate_hz: "
              << ImuSampleRateToHz(gyro_profile->getSampleRate()) << "\n";
    std::cout << "noise_density: " << in.noiseDensity << "\n";
    std::cout << "random_walk: " << in.randomWalk << "\n";
  }
}

void RunStreaming(const std::shared_ptr<ob::Pipeline>& video_pipeline,
                  const std::shared_ptr<ob::Device>& device,
                  const std::shared_ptr<ob::Pipeline>& imu_pipeline,
                  const SensorSelection& stereo,
                  const Options& options,
                  const bool use_global_timestamp,
                  const int max_frames) {
  const auto left_profile =
      SelectVideoProfile(device, video_pipeline, stereo.left_sensor, options);
  const auto right_profile =
      SelectVideoProfile(device, video_pipeline, stereo.right_sensor, options);
  std::shared_ptr<ob::Config> video_config = std::make_shared<ob::Config>();
  video_config->enableStream(left_profile);
  video_config->enableStream(right_profile);
  video_config->setFrameAggregateOutputMode(
      OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
  video_pipeline->start(video_config);

  if (imu_pipeline) {
    std::shared_ptr<ob::Config> imu_config = std::make_shared<ob::Config>();
    imu_config->enableAccelStream();
    imu_config->enableGyroStream();
    imu_config->setFrameAggregateOutputMode(
        OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    imu_pipeline->start(imu_config);
  }

  const OBFrameType left_type = SensorTypeToFrameType(stereo.left_sensor);
  const OBFrameType right_type = SensorTypeToFrameType(stereo.right_sensor);

  int frame_count = 0;
  while (max_frames == 0 || frame_count < max_frames) {
    auto frame_set = video_pipeline->waitForFrameset(100);
    if (frame_set) {
      auto left = frame_set->getFrame(left_type);
      auto right = frame_set->getFrame(right_type);
      if (left && right) {
        auto left_video = left->as<ob::VideoFrame>();
        auto right_video = right->as<ob::VideoFrame>();
        const uint64_t left_ts_ns = FrameTimestampNs(left, use_global_timestamp);
        const uint64_t right_ts_ns =
            FrameTimestampNs(right, use_global_timestamp);
        std::cout << "[LEFT ] ts(ns): " << left_ts_ns << " size: "
                  << left_video->getWidth() << "x" << left_video->getHeight()
                  << "\n";
        std::cout << "[RIGHT] ts(ns): " << right_ts_ns << " size: "
                  << right_video->getWidth() << "x" << right_video->getHeight()
                  << "\n";
        ++frame_count;
      }
    }

    if (imu_pipeline) {
      auto imu_set = imu_pipeline->waitForFrameset(1);
      if (imu_set) {
        auto accel_raw = imu_set->getFrame(OB_FRAME_ACCEL);
        if (accel_raw) {
          auto accel = accel_raw->as<ob::AccelFrame>();
          const auto v = accel->getValue();
          const uint64_t ts_ns = FrameTimestampNs(accel_raw, use_global_timestamp);
          std::cout << "[ACCEL] ts(ns): " << ts_ns << " value:[" << v.x << ", "
                    << v.y << ", " << v.z << "]\n";
        }

        auto gyro_raw = imu_set->getFrame(OB_FRAME_GYRO);
        if (gyro_raw) {
          auto gyro = gyro_raw->as<ob::GyroFrame>();
          const auto v = gyro->getValue();
          const uint64_t ts_ns = FrameTimestampNs(gyro_raw, use_global_timestamp);
          std::cout << "[GYRO ] ts(ns): " << ts_ns << " value:[" << v.x << ", "
                    << v.y << ", " << v.z << "]\n";
        }
      }
    }
  }

  video_pipeline->stop();
  if (imu_pipeline) {
    imu_pipeline->stop();
  }
}

void WriteMatrix4x4(std::ofstream* out, const OBExtrinsic& T_target_source) {
  *out << "  cols: 4\n";
  *out << "  rows: 4\n";
  *out << "  data: [";

  // OBExtrinsic is target<-source with row-major rot[9], trans[3] in mm.
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      *out << T_target_source.rot[r * 3 + c] << ", ";
    }
    *out << (T_target_source.trans[r] / 1000.0) << ", ";
  }
  *out << "0, 0, 0, 1]";
  *out << "\n";
}

OBExtrinsic InvertExtrinsic(const OBExtrinsic& T_target_source) {
  OBExtrinsic T_source_target{};

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      T_source_target.rot[r * 3 + c] = T_target_source.rot[c * 3 + r];
    }
  }

  for (int r = 0; r < 3; ++r) {
    double trans_mm = 0.0;
    for (int c = 0; c < 3; ++c) {
      trans_mm -= T_source_target.rot[r * 3 + c] * T_target_source.trans[c];
    }
    T_source_target.trans[r] = static_cast<float>(trans_mm);
  }

  return T_source_target;
}

void WriteCameraYaml(const std::filesystem::path& path,
                     const std::string& camera_id,
                     const std::shared_ptr<ob::VideoStreamProfile>& profile,
                     const OBExtrinsic& T_imu_cam) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot open " + path.string());
  }

  const auto in = profile->getIntrinsic();
  const auto d = profile->getDistortion();

  out << "%YAML:1.0\n";
  out << "camera_id: " << camera_id << "\n\n";
  out << "T_BS:\n";
  WriteMatrix4x4(&out, T_imu_cam);
  out << "\n";
  out << "rate_hz: " << profile->getFps() << "\n";
  out << "resolution: [" << in.width << ", " << in.height << "]\n";
  out << "camera_model: pinhole\n";
  out << "intrinsics: [" << in.fx << ", " << in.fy << ", " << in.cx << ", "
      << in.cy << "]\n";
  out << "distortion_model: radial-tangential\n";
  out << "distortion_coefficients: [" << d.k1 << ", " << d.k2 << ", "
      << d.p1 << ", " << d.p2 << "]\n";
}

void WriteImuYaml(const std::filesystem::path& path,
                  const std::shared_ptr<ob::AccelStreamProfile>& accel_profile,
                  const std::shared_ptr<ob::GyroStreamProfile>& gyro_profile) {
  const auto accel_intrinsic = accel_profile ? accel_profile->getIntrinsic()
                                             : OBAccelIntrinsic{};
  const auto gyro_intrinsic = gyro_profile ? gyro_profile->getIntrinsic()
                                           : OBGyroIntrinsic{};

  const double accel_hz =
      accel_profile ? ImuSampleRateToHz(accel_profile->getSampleRate()) : 0.0;
  const double gyro_hz =
      gyro_profile ? ImuSampleRateToHz(gyro_profile->getSampleRate()) : 0.0;

  const double rate_hz = gyro_hz > 0.0 ? gyro_hz : (accel_hz > 0.0 ? accel_hz : 200.0);

  const auto safe_or = [](const double v, const double fallback) {
    if (!std::isfinite(v) || v <= 0.0) return fallback;
    return v;
  };

  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot open " + path.string());
  }

  out << "%YAML:1.0\n";
  out << "imu_preintegration_type: 1\n\n";
  out << "T_BS:\n";
  out << "  cols: 4\n";
  out << "  rows: 4\n";
  out << "  data: [1, 0, 0, 0,\n"
      << "         0, 1, 0, 0,\n"
      << "         0, 0, 1, 0,\n"
      << "         0, 0, 0, 1]\n\n";
  out << "rate_hz: " << static_cast<int>(std::round(rate_hz)) << "\n";
  out << "imu_bias_init_sigma: 1e-3\n";
  out << "gyroscope_noise_density: "
      << safe_or(gyro_intrinsic.noiseDensity, 8.7e-5) << "\n";
  out << "gyroscope_random_walk: "
      << safe_or(gyro_intrinsic.randomWalk, 7.4e-4) << "\n";
  out << "accelerometer_noise_density: "
      << safe_or(accel_intrinsic.noiseDensity, 4.4e-4) << "\n";
  out << "accelerometer_random_walk: "
      << safe_or(accel_intrinsic.randomWalk, 2.02e-2) << "\n";
  out << "do_imu_rate_time_alignment: 1\n";
  out << "time_alignment_window_size_s: 10.0\n";
  out << "time_alignment_variance_threshold_scaling: 30.0\n";
  out << "imu_integration_sigma: 1.0e-8\n";
  out << "imu_time_shift: 0.0\n";
  out << "n_gravity: [0.0, 0.0, -9.81]\n";
}

void DumpKimeraYaml(const std::shared_ptr<ob::Pipeline>& video_pipeline,
                    const std::shared_ptr<ob::Device>& device,
                    const SensorSelection& stereo,
                    const Options& options) {
  const auto left_profile =
      SelectVideoProfile(device, video_pipeline, stereo.left_sensor, options);
  const auto right_profile =
      SelectVideoProfile(device, video_pipeline, stereo.right_sensor, options);

  const auto accel_profile = SelectAccelProfile(device);
  const auto gyro_profile = SelectGyroProfile(device);
  if (!accel_profile || !gyro_profile) {
    throw std::runtime_error("IMU profiles not available for dump.");
  }

  const OBExtrinsic T_left_imu = accel_profile->getExtrinsicTo(left_profile);
  const OBExtrinsic T_right_imu = accel_profile->getExtrinsicTo(right_profile);
  const OBExtrinsic T_imu_left = InvertExtrinsic(T_left_imu);
  const OBExtrinsic T_imu_right = InvertExtrinsic(T_right_imu);

  std::filesystem::create_directories(options.output_dir);
  WriteCameraYaml(options.output_dir / "LeftCameraParams.yaml",
                  "left_cam",
                  left_profile,
                  T_imu_left);
  WriteCameraYaml(options.output_dir / "RightCameraParams.yaml",
                  "right_cam",
                  right_profile,
                  T_imu_right);
  WriteImuYaml(options.output_dir / "ImuParams.yaml", accel_profile,
               gyro_profile);

  std::ofstream note(options.output_dir / "README.txt");
  note << "Generated by orbbec_sensor_inspector\n"
       << "Files:\n"
       << "  - LeftCameraParams.yaml\n"
       << "  - RightCameraParams.yaml\n"
       << "  - ImuParams.yaml\n\n"
       << "Usage with Kimera-VIO:\n"
       << "  1) Recommended: run save_sensor_yaml.sh --sensor orbbec --compose_params_dir <params_dir>\n"
       << "  2) Manual: copy these 3 files into your params folder.\n"
       << "  3) Ensure Pipeline/Frontend/Backend/Lcd/Display YAML files also exist.\n"
       << "  4) Run orbbecLiveVIO with --params_folder_path pointing to that folder.\n";

  std::cout << "[OK] Wrote YAML files to: " << options.output_dir << "\n";
}

}  // namespace

int main(int argc, char** argv) try {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  ob::Context context;
  const auto open_result = OpenDevice(&context, options);
  const auto& device = open_result.device;
  const auto stereo = SelectStereoSensors(device);
  PrintDeviceSummary(device, stereo, options);

  auto video_pipeline = std::make_shared<ob::Pipeline>(device);
  std::shared_ptr<ob::Pipeline> imu_pipeline;
  if (device->getSensor(OB_SENSOR_ACCEL) && device->getSensor(OB_SENSOR_GYRO)) {
    imu_pipeline = std::make_shared<ob::Pipeline>(device);
  }

  switch (options.mode) {
    case Options::Mode::kDescribe:
      break;
    case Options::Mode::kPrintIntrinsics:
      PrintIntrinsics(video_pipeline, device, stereo, options);
      break;
    case Options::Mode::kRun:
      RunStreaming(video_pipeline,
                   device,
                   imu_pipeline,
                   stereo,
                   options,
                   options.use_global_timestamp,
                   options.frames);
      break;
    case Options::Mode::kDumpKimeraYaml:
      DumpKimeraYaml(video_pipeline, device, stereo, options);
      break;
    case Options::Mode::kNone:
      PrintUsage(argv[0]);
      return 1;
  }

  return 0;
}
catch (const ob::Error& e) {
  std::cerr << "[ERR] Orbbec SDK failure\n"
            << "  function: " << e.getFunction() << "\n"
            << "  args    : " << e.getArgs() << "\n"
            << "  message : " << e.what() << "\n"
            << "  type    : " << e.getExceptionType() << "\n";
  return 1;
}
catch (const std::exception& e) {
  std::cerr << "[ERR] " << e.what() << "\n";
  return 1;
}
