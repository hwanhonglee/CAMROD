#include <sl/Camera.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {

// Quick workflow (ZED):
//  1) Check raw intrinsics/noise:
//     ./zed_sensor_inspector --print_intrinsics
//  2) Validate live timestamps and IMU stream health:
//     ./zed_sensor_inspector --run --frames 300
//  3) Export Kimera sensor YAMLs (camera/imu only):
//     ./zed_sensor_inspector --dump_kimera_yaml --output_dir <dir>
//  4) Use that output as sensor section when preparing Kimera params folder.
struct Options {
  enum class Mode {
    kNone,
    kPrintIntrinsics,
    kRun,
    kDumpKimeraYaml,
  } mode = Mode::kNone;

  std::string resolution = "HD720";
  int fps = 30;
  bool use_rectified = false;
  bool invert_imu_to_cam = false;
  std::filesystem::path output_dir = "./zed_sensor_yaml";
  int frames = 0;  // 0 means run forever.
};

void PrintUsage(const char* argv0) {
  std::cout
      << "ZED Sensor Inspector\n"
      << "\n"
      << "Purpose:\n"
      << "  - Check ZED intrinsics/IMU quickly\n"
      << "  - Verify image+IMU timestamps\n"
      << "  - Dump Kimera-compatible sensor YAML files\n"
      << "\n"
      << "Modes (pick one):\n"
      << "  --print_intrinsics         Print left/right intrinsics + distortion\n"
      << "  --run                      Stream image/imu lines to stdout\n"
      << "  --dump_kimera_yaml         Write Left/Right/Imu YAML files\n"
      << "\n"
      << "Common options:\n"
      << "  --resolution <HD2K|HD1080|HD720|VGA>   (default: HD720)\n"
      << "  --fps <int>                              (default: 30)\n"
      << "  --rectified                              (use rectified calibration)\n"
      << "  --invert_imu_to_cam                      (invert IMU->cam extrinsic)\n"
      << "\n"
      << "Run options:\n"
      << "  --frames <N>   stop after N grabs (default: 0, infinite)\n"
      << "\n"
      << "Dump options:\n"
      << "  --output_dir <path>  output directory (default: ./zed_sensor_yaml)\n"
      << "\n"
      << "Examples:\n"
      << "  "
      << argv0
      << " --print_intrinsics --resolution HD720 --fps 30\n"
      << "  "
      << argv0
      << " --run --frames 200\n"
      << "  "
      << argv0
      << " --dump_kimera_yaml --output_dir ./tmp/zed_sensor --rectified\n";
}

sl::RESOLUTION ParseResolution(const std::string& resolution) {
  if (resolution == "HD2K") return sl::RESOLUTION::HD2K;
  if (resolution == "HD1080") return sl::RESOLUTION::HD1080;
  if (resolution == "HD720") return sl::RESOLUTION::HD720;
  if (resolution == "VGA") return sl::RESOLUTION::VGA;
  std::cerr << "[WARN] Unknown resolution '" << resolution
            << "', fallback to HD720.\n";
  return sl::RESOLUTION::HD720;
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

    if (arg == "--rectified") {
      options->use_rectified = true;
      continue;
    }
    if (arg == "--invert_imu_to_cam" || arg == "--invert-imu-to-cam") {
      options->invert_imu_to_cam = true;
      continue;
    }

    if (arg == "--resolution" && i + 1 < argc) {
      options->resolution = argv[++i];
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
  if (options->fps <= 0) {
    std::cerr << "[ERR] --fps must be > 0\n";
    return false;
  }
  if (options->frames < 0) {
    std::cerr << "[ERR] --frames must be >= 0\n";
    return false;
  }
  return true;
}

bool OpenCamera(const Options& options, sl::Camera* zed) {
  sl::InitParameters init_params;
  init_params.camera_resolution = ParseResolution(options.resolution);
  init_params.camera_fps = options.fps;
  init_params.depth_mode = sl::DEPTH_MODE::NONE;
  init_params.coordinate_units = sl::UNIT::METER;
  init_params.sdk_verbose = false;

  const sl::ERROR_CODE err = zed->open(init_params);
  if (err != sl::ERROR_CODE::SUCCESS) {
    std::cerr << "[ERR] Failed to open ZED camera: " << sl::toString(err)
              << "\n";
    return false;
  }
  return true;
}

void PrintIntrinsics(sl::Camera& zed, const bool rectified) {
  const auto info = zed.getCameraInformation();
  const auto calib = rectified ? info.camera_configuration.calibration_parameters
                               : info.camera_configuration.calibration_parameters_raw;

  const auto print_cam = [](const char* name, const sl::CameraParameters& c) {
    std::cout << "===== " << name << " =====\n";
    std::cout << "fx: " << c.fx << "\n";
    std::cout << "fy: " << c.fy << "\n";
    std::cout << "cx: " << c.cx << "\n";
    std::cout << "cy: " << c.cy << "\n";
    std::cout << "k1: " << c.disto[0] << "\n";
    std::cout << "k2: " << c.disto[1] << "\n";
    std::cout << "p1: " << c.disto[2] << "\n";
    std::cout << "p2: " << c.disto[3] << "\n";
    std::cout << "k3: " << c.disto[4] << "\n";
  };

  print_cam("LEFT CAMERA INTRINSICS", calib.left_cam);
  print_cam("RIGHT CAMERA INTRINSICS", calib.right_cam);

  const auto& sensors = info.sensors_configuration;
  std::cout << "===== IMU NOISE MODEL (SDK) =====\n";
  std::cout << "gyro sampling_rate: " << sensors.gyroscope_parameters.sampling_rate
            << " Hz\n";
  std::cout << "gyro noise_density: " << sensors.gyroscope_parameters.noise_density
            << "\n";
  std::cout << "gyro random_walk: " << sensors.gyroscope_parameters.random_walk
            << "\n";
  std::cout << "accel sampling_rate: "
            << sensors.accelerometer_parameters.sampling_rate << " Hz\n";
  std::cout << "accel noise_density: "
            << sensors.accelerometer_parameters.noise_density << "\n";
  std::cout << "accel random_walk: "
            << sensors.accelerometer_parameters.random_walk << "\n";
}

void RunStreaming(sl::Camera& zed, const Options& options) {
  sl::RuntimeParameters runtime_params;
  sl::Mat left_img;
  sl::SensorsData sensors_data;

  int frame_count = 0;
  while (options.frames == 0 || frame_count < options.frames) {
    if (zed.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) {
      continue;
    }

    zed.retrieveImage(left_img, sl::VIEW::LEFT);
    const uint64_t img_ts =
        zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();

    std::cout << "[IMAGE] ts(ns): " << img_ts << " size: "
              << left_img.getWidth() << "x" << left_img.getHeight() << "\n";

    if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE) ==
        sl::ERROR_CODE::SUCCESS) {
      const auto& imu = sensors_data.imu;
      const uint64_t imu_ts = imu.timestamp.getNanoseconds();
      std::cout << "[IMU  ] ts(ns): " << imu_ts << " acc:["
                << imu.linear_acceleration.x << ", "
                << imu.linear_acceleration.y << ", "
                << imu.linear_acceleration.z << "] gyro:["
                << imu.angular_velocity.x << ", " << imu.angular_velocity.y
                << ", " << imu.angular_velocity.z << "]\n";
    }

    ++frame_count;
  }
}

void WriteMatrix4x4(std::ofstream* out, const sl::Transform& T) {
  *out << "  cols: 4\n";
  *out << "  rows: 4\n";
  *out << "  data: [";
  for (int i = 0; i < 16; ++i) {
    *out << static_cast<double>(T.m[i]);
    if (i < 15) *out << ", ";
  }
  *out << "]\n";
}

void WriteCameraYaml(const std::filesystem::path& path,
                     const std::string& camera_id,
                     const sl::CameraParameters& camera,
                     const sl::Transform& T_BS,
                     const int fps,
                     const bool rectified) {
  std::ofstream out(path);
  if (!out.is_open()) {
    throw std::runtime_error("Cannot open " + path.string());
  }

  out << "%YAML:1.0\n";
  out << "camera_id: " << camera_id << "\n\n";
  out << "T_BS:\n";
  WriteMatrix4x4(&out, T_BS);
  out << "\n";
  out << "rate_hz: " << fps << "\n";
  out << "resolution: [" << camera.image_size.width << ", "
      << camera.image_size.height << "]\n";
  out << "camera_model: pinhole\n";
  out << "intrinsics: [" << camera.fx << ", " << camera.fy << ", "
      << camera.cx << ", " << camera.cy << "]\n";

  if (rectified) {
    out << "distortion_model: none\n";
    out << "distortion_coefficients: [0, 0, 0, 0]\n";
  } else {
    out << "distortion_model: radial-tangential\n";
    out << "distortion_coefficients: [" << camera.disto[0] << ", "
        << camera.disto[1] << ", " << camera.disto[2] << ", "
        << camera.disto[3] << "]\n";
  }
}

void WriteImuYaml(const std::filesystem::path& path,
                  const sl::SensorsConfiguration& sensors,
                  const int fallback_rate_hz) {
  double rate_hz = sensors.gyroscope_parameters.sampling_rate;
  if (!(std::isfinite(rate_hz) && rate_hz > 0.0)) {
    rate_hz = sensors.accelerometer_parameters.sampling_rate;
  }
  if (!(std::isfinite(rate_hz) && rate_hz > 0.0)) {
    rate_hz = fallback_rate_hz;
  }

  double gyro_noise = sensors.gyroscope_parameters.noise_density;
  double gyro_rw = sensors.gyroscope_parameters.random_walk;
  if (sensors.gyroscope_parameters.sensor_unit == sl::SENSORS_UNIT::DEG_SEC) {
    static constexpr double kDegToRad = 0.017453292519943295;
    gyro_noise *= kDegToRad;
    gyro_rw *= kDegToRad;
  }

  double accel_noise = sensors.accelerometer_parameters.noise_density;
  double accel_rw = sensors.accelerometer_parameters.random_walk;

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
  out << "gyroscope_noise_density: " << safe_or(gyro_noise, 8.7e-5) << "\n";
  out << "gyroscope_random_walk: " << safe_or(gyro_rw, 7.4e-4) << "\n";
  out << "accelerometer_noise_density: " << safe_or(accel_noise, 4.4e-4)
      << "\n";
  out << "accelerometer_random_walk: " << safe_or(accel_rw, 2.02e-2) << "\n";
  out << "do_imu_rate_time_alignment: 1\n";
  out << "time_alignment_window_size_s: 10.0\n";
  out << "time_alignment_variance_threshold_scaling: 30.0\n";
  out << "imu_integration_sigma: 1.0e-8\n";
  out << "imu_time_shift: 0.0\n";
  out << "n_gravity: [0.0, 0.0, -9.81]\n";
}

void DumpKimeraYaml(sl::Camera& zed, const Options& options) {
  const auto info = zed.getCameraInformation();
  const auto calib = options.use_rectified
                         ? info.camera_configuration.calibration_parameters
                         : info.camera_configuration.calibration_parameters_raw;

  // ZED SDK gives Camera<-IMU transform. Kimera expects body(T_BS)->sensor.
  // Here we follow the same convention used by existing ZED dump tool.
  sl::Transform T_BS_left = info.sensors_configuration.camera_imu_transform;
  if (options.invert_imu_to_cam) {
    T_BS_left.inverse();
  }
  sl::Transform T_BS_right = sl::Transform(T_BS_left * calib.stereo_transform);

  std::filesystem::create_directories(options.output_dir);
  WriteCameraYaml(options.output_dir / "LeftCameraParams.yaml",
                  "left_cam",
                  calib.left_cam,
                  T_BS_left,
                  options.fps,
                  options.use_rectified);
  WriteCameraYaml(options.output_dir / "RightCameraParams.yaml",
                  "right_cam",
                  calib.right_cam,
                  T_BS_right,
                  options.fps,
                  options.use_rectified);
  WriteImuYaml(options.output_dir / "ImuParams.yaml",
               info.sensors_configuration,
               options.fps);

  std::ofstream note(options.output_dir / "README.txt");
  note << "Generated by zed_sensor_inspector\n"
       << "Files:\n"
       << "  - LeftCameraParams.yaml\n"
       << "  - RightCameraParams.yaml\n"
       << "  - ImuParams.yaml\n\n"
       << "Usage with Kimera-VIO:\n"
       << "  1) Recommended: run save_sensor_yaml.sh --sensor zed --compose_params_dir <params_dir>\n"
       << "  2) Manual: copy these 3 files into your params folder.\n"
       << "  3) Ensure Pipeline/Frontend/Backend/Lcd/Display YAML files also exist.\n"
       << "  4) Run zedLiveVIO with --params_folder_path pointing to that folder.\n";

  std::cout << "[OK] Wrote YAML files to: " << options.output_dir << "\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  sl::Camera zed;
  if (!OpenCamera(options, &zed)) {
    return 1;
  }

  try {
    switch (options.mode) {
      case Options::Mode::kPrintIntrinsics:
        PrintIntrinsics(zed, options.use_rectified);
        break;
      case Options::Mode::kRun:
        RunStreaming(zed, options);
        break;
      case Options::Mode::kDumpKimeraYaml:
        DumpKimeraYaml(zed, options);
        break;
      case Options::Mode::kNone:
        PrintUsage(argv[0]);
        zed.close();
        return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "[ERR] " << e.what() << "\n";
    zed.close();
    return 1;
  }

  zed.close();
  return 0;
}
