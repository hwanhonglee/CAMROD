/**
 * GPU Checker Node (C++ 버전)
 *
 * nvidia-smi 를 subprocess 로 실행해 NVIDIA GPU 상태를 주기적으로 수집하고
 * /diagnostics 토픽으로 발행한다.
 *
 * nvidia-smi 를 찾을 수 없거나 GPU 가 없으면 STALE 상태를 발행한다.
 *
 * 컨테이너 환경 지원
 * ------------------
 * NVIDIA Container Toolkit(nvidia-docker2 / CDI) 을 사용하는 환경에서는
 * 컨테이너 내부에서도 nvidia-smi 가 정상 동작하므로 별도 설정이 불필요하다.
 *
 * Toolkit 없이 GPU 디바이스만 마운트된 경우를 위해 호스트 경로도 탐색한다.
 *   - /host/usr/bin/nvidia-smi
 *   - /host/usr/local/bin/nvidia-smi
 *
 * 또는 파라미터 / 환경 변수로 명시적으로 지정할 수 있다.
 *   파라미터: container.nvidia_smi_path  (기본 "")
 *   환경 변수: NVIDIA_SMI_PATH
 */
#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── Container environment detected ────────────────────────────────────────────────────

static bool is_in_container()
{
  if (access("/.dockerenv", F_OK) == 0) return true;
  if (access("/run/.containerenv", F_OK) == 0) return true;
  std::ifstream f("/proc/1/cgroup");
  if (f.is_open()) {
    std::string line;
    while (std::getline(f, line)) {
      if (line.find("docker")     != std::string::npos ||
          line.find("lxc")        != std::string::npos ||
          line.find("kubepods")   != std::string::npos ||
          line.find("containerd") != std::string::npos)
      {
        return true;
      }
    }
  }
  return false;
}

// ── nvidia-smi 유틸리티 ───────────────────────────────────────────────────

static std::optional<std::string> find_nvidia_smi(const std::string & override_path = "")
{
  // 1. 명시적 오버라이드 경로
  if (!override_path.empty()) {
    if (access(override_path.c_str(), X_OK) == 0) return override_path;
    return std::nullopt;  // 명시했는데 없으면 실패로 처리
  }

  // 2. 일반 경로 탐색 (컨테이너 내부 + 호스트 마운트 경로 포함)
  const char * paths[] = {
    "/usr/bin/nvidia-smi",
    "/usr/local/bin/nvidia-smi",
    "/opt/nvidia/bin/nvidia-smi",
    "/host/usr/bin/nvidia-smi",           // 호스트 /usr 마운트
    "/host/usr/local/bin/nvidia-smi",
    "/host/opt/nvidia/bin/nvidia-smi",
  };
  for (const char * p : paths) {
    if (access(p, X_OK) == 0) return std::string(p);
  }

  // 3. PATH 탐색
  if (system("which nvidia-smi > /dev/null 2>&1") == 0) {
    return std::string("nvidia-smi");
  }
  return std::nullopt;
}

struct GpuInfo {
  std::string index;
  std::string name;
  double util{0.0};
  double mem_used{0.0};
  double mem_total{0.0};
  double temp{0.0};
  std::string power_draw;
  std::string power_limit;
};

static std::vector<GpuInfo> query_gpus(const std::string & nvidia_smi)
{
  const std::string query =
    "index,name,utilization.gpu,memory.used,memory.total,"
    "temperature.gpu,power.draw,power.limit";
  std::string cmd = nvidia_smi +
    " --query-gpu=" + query +
    " --format=csv,noheader,nounits 2>/dev/null";

  FILE * pipe = popen(cmd.c_str(), "r");
  if (!pipe) return {};

  std::vector<GpuInfo> gpus;
  char buf[512];
  while (fgets(buf, sizeof(buf), pipe)) {
    std::istringstream ss(buf);
    std::vector<std::string> parts;
    std::string token;
    while (std::getline(ss, token, ',')) {
      // trim whitespace
      size_t s = token.find_first_not_of(" \t\r\n");
      size_t e = token.find_last_not_of(" \t\r\n");
      parts.push_back((s != std::string::npos) ? token.substr(s, e - s + 1) : "");
    }
    if (parts.size() < 8) continue;

    GpuInfo g;
    g.index       = parts[0];
    g.name        = parts[1];
    try { g.util      = std::stod(parts[2]); } catch (...) {}
    try { g.mem_used  = std::stod(parts[3]); } catch (...) {}
    try { g.mem_total = std::stod(parts[4]); } catch (...) {}
    try { g.temp      = std::stod(parts[5]); } catch (...) {}
    g.power_draw  = parts[6];
    g.power_limit = parts[7];
    gpus.push_back(g);
  }
  pclose(pipe);
  return gpus;
}

// ── GpuCheckerNode ────────────────────────────────────────────────────────

class GpuCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit GpuCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("gpu_checker", "gpu_checker", options)
  {
    base_init();  // declare/load parameters → setup tasks

    RCLCPP_INFO(
      get_logger(),
      "GpuCheckerNode started. "
      "util warn=%.0f%% error=%.0f%% | "
      "mem warn=%.0f%% error=%.0f%% | "
      "temp warn=%.0f C error=%.0f C",
      util_warn_, util_error_,
      mem_warn_,  mem_error_,
      temp_warn_, temp_error_);

    if (in_container_) {
      RCLCPP_INFO(get_logger(), "Container environment detected. nvidia_smi=%s",
        nvidia_smi_ ? nvidia_smi_->c_str() : "N/A");
    }
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("gpu.util_warn_threshold",  85.0);
    declare_parameter("gpu.util_error_threshold", 95.0);
    declare_parameter("gpu.mem_warn_threshold",   80.0);
    declare_parameter("gpu.mem_error_threshold",  95.0);
    declare_parameter("gpu.temp_warn_threshold",  75.0);
    declare_parameter("gpu.temp_error_threshold", 85.0);
    declare_parameter("gpu.required", true);

    // 컨테이너 환경에서 nvidia-smi 경로 오버라이드 (빈 문자열 = 자동 탐색)
    declare_parameter("container.nvidia_smi_path", std::string(""));
  }

  void load_parameters_() override
  {
    util_warn_  = get_parameter("gpu.util_warn_threshold").as_double();
    util_error_ = get_parameter("gpu.util_error_threshold").as_double();
    mem_warn_   = get_parameter("gpu.mem_warn_threshold").as_double();
    mem_error_  = get_parameter("gpu.mem_error_threshold").as_double();
    temp_warn_  = get_parameter("gpu.temp_warn_threshold").as_double();
    temp_error_ = get_parameter("gpu.temp_error_threshold").as_double();
    gpu_required_ = get_parameter("gpu.required").as_bool();

    in_container_ = is_in_container();

    // 환경 변수 → 파라미터 → 자동 탐색
    std::string smi_override;
    const char * env_smi = std::getenv("NVIDIA_SMI_PATH");
    if (env_smi && env_smi[0] != '\0') {
      smi_override = env_smi;
    } else {
      smi_override = get_parameter("container.nvidia_smi_path").as_string();
    }

    auto smi = find_nvidia_smi(smi_override);
    if (!smi) {
      RCLCPP_WARN(get_logger(),
        "nvidia-smi not found. GPU status will be published as STALE.%s",
        in_container_
          ? " (container: check NVIDIA Container Toolkit or "
            "set container.nvidia_smi_path)"
          : "");
    }
    nvidia_smi_ = smi;
    cache_time_  = std::chrono::steady_clock::time_point{};  // 초기화
  }

  void setup_tasks_() override
  {
    if (!nvidia_smi_) {
      add_task("/hardware/gpu0", [this](auto & s) { check_missing_gpu(s); });
      return;
    }

    try {
      auto gpus = query_gpus(*nvidia_smi_);
      if (gpus.empty()) {
        add_task("/hardware/gpu0", [this](auto & s) { check_missing_gpu(s); });
      } else {
        for (size_t i = 0; i < gpus.size(); ++i) {
          add_task(
            "/hardware/gpu" + std::to_string(i),
            [this, i](auto & s) { check_gpu(s, i); });
        }
      }
    } catch (...) {
      add_task("/hardware/gpu0", [this](auto & s) { check_missing_gpu(s); });
    }
  }

private:
  const std::vector<GpuInfo> & query_cached()
  {
    auto now = std::chrono::steady_clock::now();
    double elapsed =
      std::chrono::duration<double>(now - cache_time_).count();

    if (elapsed > 0.1) {
      gpu_cache_  = query_gpus(*nvidia_smi_);
      cache_time_ = now;
    }
    return gpu_cache_;
  }

  void check_missing_gpu(StatusWrapper & stat)
  {
    if (!gpu_required_) {
      // HH_260617: Simulation/dev PCs may not expose NVIDIA GPU. When the
      // selected diagnostics profile marks GPU optional, publish OK instead
      // of blocking planning/control validation.
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "GPU check disabled or not required");
      stat.add("gpu_required", "false");
      return;
    }
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "nvidia-smi not found");
    if (in_container_) {
      stat.add("hint",
        "Container environment: check NVIDIA Container Toolkit or "
        "container.nvidia_smi_path");
    }
  }

  void check_gpu(StatusWrapper & stat, size_t idx)
  {
    try {
      const auto & gpus = query_cached();
      const GpuInfo * gpu = nullptr;
      for (const auto & g : gpus) {
        if (g.index == std::to_string(idx)) { gpu = &g; break; }
      }
      if (!gpu) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "GPU" + std::to_string(idx) + " unavailable");
        return;
      }

      double mem_pct   = gpu->mem_total > 0 ? gpu->mem_used / gpu->mem_total * 100.0 : 0.0;
      int8_t lvl_util  = check_high(gpu->util, util_warn_, util_error_);
      int8_t lvl_mem   = check_high(mem_pct,   mem_warn_,  mem_error_);
      int8_t lvl_temp  = check_high(gpu->temp, temp_warn_, temp_error_);
      int8_t overall   = std::max({lvl_util, lvl_mem, lvl_temp});

      std::string issues;
      auto append = [&](const std::string & s) {
        if (!issues.empty()) issues += ", ";
        issues += s;
      };
      char tmp[32];
      if (lvl_util >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        std::snprintf(tmp, sizeof(tmp), "util %.0f%%", gpu->util);
        append(tmp);
      }
      if (lvl_mem >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        std::snprintf(tmp, sizeof(tmp), "mem %.0f%%", mem_pct);
        append(tmp);
      }
      if (lvl_temp >= diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        std::snprintf(tmp, sizeof(tmp), "temp %.0f C", gpu->temp);
        append(tmp);
      }

      std::string prefix = "GPU" + std::to_string(idx);
      std::string message;
      if      (overall == diagnostic_msgs::msg::DiagnosticStatus::ERROR) message = prefix + " ERROR: " + issues;
      else if (overall == diagnostic_msgs::msg::DiagnosticStatus::WARN)  message = prefix + " WARN: "  + issues;
      else                                          message = prefix + " OK (" + gpu->name + ")";

      stat.summary(overall, message);
      stat.add("name", gpu->name);
      std::snprintf(tmp, sizeof(tmp), "%.1f", gpu->util);     stat.add("util_%",       std::string(tmp));
      std::snprintf(tmp, sizeof(tmp), "%.0f", gpu->mem_used); stat.add("mem_used_MB",  std::string(tmp));
      std::snprintf(tmp, sizeof(tmp), "%.0f", gpu->mem_total);stat.add("mem_total_MB", std::string(tmp));
      std::snprintf(tmp, sizeof(tmp), "%.1f", mem_pct);       stat.add("mem_%",        std::string(tmp));
      std::snprintf(tmp, sizeof(tmp), "%.0f", gpu->temp);     stat.add("temp_C",       std::string(tmp));
      if (gpu->power_draw  != "[N/A]") stat.add("power_W",       gpu->power_draw);
      if (gpu->power_limit != "[N/A]") stat.add("power_limit_W", gpu->power_limit);

      if (in_container_) {
        stat.add("nvidia_smi_path", *nvidia_smi_);
      }

    } catch (const std::exception & e) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE,
        std::string("nvidia-smi error: ") + e.what());
    }
  }

  double util_warn_{85.0},  util_error_{95.0};
  double mem_warn_{80.0},   mem_error_{95.0};
  double temp_warn_{75.0},  temp_error_{85.0};
  bool gpu_required_{true};

  bool                                        in_container_{false};
  std::optional<std::string>                  nvidia_smi_;
  std::vector<GpuInfo>                        gpu_cache_;
  std::chrono::steady_clock::time_point       cache_time_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(GpuCheckerNode)
