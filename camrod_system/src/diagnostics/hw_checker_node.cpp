/**
 * HW Checker Node (C++ 버전)
 *
 * /proc/stat, /proc/meminfo, statvfs(), /sys/class/thermal 을 사용해
 * CPU / 메모리 / 디스크 / CPU 온도 상태를 주기적으로 수집하고
 * /diagnostics 토픽으로 발행한다.
 *
 * 컨테이너 환경 지원
 * ------------------
 * 컨테이너 배포 시 호스트 리소스를 올바르게 읽기 위해 아래 방법을 지원한다.
 *
 *  1. 자동 감지: /.dockerenv / /run/.containerenv / /proc/1/cgroup 으로
 *     컨테이너 여부를 감지하고, /host/proc, /host/sys 마운트가 있으면 자동 사용.
 *
 *  2. 수동 지정 (파라미터):
 *       container.host_proc_path  (기본 "")  → 호스트 /proc 마운트 경로
 *       container.host_sys_path   (기본 "")  → 호스트 /sys 마운트 경로
 *       container.host_rootfs_path(기본 "")  → 호스트 루트 FS 마운트 경로
 *         (disk.path 가 "/"일 때, 컨테이너 overlay 대신 이 경로로 statvfs)
 *
 *  3. Docker 실행 예시 (호스트 리소스 마운트):
 *       docker run ... \
 *         -v /proc:/host/proc:ro \
 *         -v /sys:/host/sys:ro  \
 *         -v /:/host/rootfs:ro  \
 *         -e HOST_PROC=/host/proc \
 *         -e HOST_SYS=/host/sys   \
 *         -e HOST_ROOTFS=/host/rootfs
 *
 *     환경 변수가 설정되어 있으면 파라미터보다 우선 적용됩니다.
 *
 *  4. cgroup v1/v2 CPU·메모리 제한 정보를 diagnostics 에 함께 보고.
 */
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <sys/statvfs.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

namespace fs = std::filesystem;

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── Container environment detected ────────────────────────────────────────────────────

static bool is_in_container()
{
  // Docker / Moby
  if (access("/.dockerenv", F_OK) == 0) return true;
  // Podman (rootless)
  if (access("/run/.containerenv", F_OK) == 0) return true;
  // cgroup 경로에서 docker / lxc / kubepods / containerd 확인
  std::ifstream f("/proc/1/cgroup");
  if (f.is_open()) {
    std::string line;
    while (std::getline(f, line)) {
      if (line.find("docker")      != std::string::npos ||
          line.find("lxc")         != std::string::npos ||
          line.find("kubepods")    != std::string::npos ||
          line.find("containerd")  != std::string::npos)
      {
        return true;
      }
    }
  }
  return false;
}

// ── 경로 유틸리티 ─────────────────────────────────────────────────────────

/**
 * 후보 경로들을 순서대로 확인해 실제 접근 가능한 첫 번째 경로를 반환한다.
 * 모두 없으면 fallback 을 반환.
 */
static std::string resolve_path(
  const std::vector<std::string> & candidates,
  const std::string & fallback)
{
  for (const auto & p : candidates) {
    if (!p.empty() && access(p.c_str(), F_OK) == 0) return p;
  }
  return fallback;
}

// ── cgroup 리소스 제한 읽기 ───────────────────────────────────────────────

/**
 * cgroup v1/v2 에서 CPU 쿼터(vCPU 수)를 읽는다.
 * 반환값: 할당된 가상 CPU 코어 수 (예: 2.0 = 2 vCPU)
 */
static std::optional<double> read_cgroup_cpu_quota()
{
  // cgroup v2: /sys/fs/cgroup/cpu.max → "quota period" 형식
  {
    std::ifstream f("/sys/fs/cgroup/cpu.max");
    std::string line;
    if (f && std::getline(f, line) && line != "max") {
      std::istringstream ss(line);
      std::string quota_s, period_s;
      ss >> quota_s >> period_s;
      if (quota_s != "max" && !period_s.empty()) {
        try {
          double quota  = std::stod(quota_s);
          double period = std::stod(period_s);
          if (period > 0.0) return quota / period;
        } catch (...) {}
      }
    }
  }
  // cgroup v1: cpu.cfs_quota_us / cpu.cfs_period_us
  {
    int64_t quota = -1;
    uint64_t period = 100000;
    std::ifstream fq("/sys/fs/cgroup/cpu/cpu.cfs_quota_us");
    std::ifstream fp("/sys/fs/cgroup/cpu/cpu.cfs_period_us");
    if (fq >> quota && fp >> period && quota > 0 && period > 0) {
      return static_cast<double>(quota) / static_cast<double>(period);
    }
  }
  return std::nullopt;
}

/**
 * cgroup v1/v2 에서 메모리 제한(kB)을 읽는다.
 * 제한이 없으면 nullopt 반환.
 */
static std::optional<uint64_t> read_cgroup_mem_limit_kb()
{
  // cgroup v2: /sys/fs/cgroup/memory.max
  {
    std::ifstream f("/sys/fs/cgroup/memory.max");
    std::string line;
    if (f && std::getline(f, line) && line != "max") {
      try {
        uint64_t bytes = std::stoull(line);
        return bytes / 1024ULL;
      } catch (...) {}
    }
  }
  // cgroup v1: /sys/fs/cgroup/memory/memory.limit_in_bytes
  {
    std::ifstream f("/sys/fs/cgroup/memory/memory.limit_in_bytes");
    uint64_t bytes = 0;
    if (f >> bytes) {
      // 페이지 경계 최댓값(사실상 무제한)은 제외: 9 PB 기준
      constexpr uint64_t kUnlimited = 9ULL * 1024 * 1024 * 1024 * 1024 * 1024;
      if (bytes < kUnlimited) return bytes / 1024ULL;
    }
  }
  return std::nullopt;
}

// ── CPU 사용률 읽기 (/proc/stat) ──────────────────────────────────────────

struct CpuTimes {
  uint64_t user{0}, nice{0}, system{0}, idle{0},
           iowait{0}, irq{0}, softirq{0}, steal{0};
  uint64_t total()  const { return user + nice + system + idle + iowait + irq + softirq + steal; }
  uint64_t active() const { return total() - idle - iowait; }
};

static std::optional<CpuTimes> read_cpu_times(const std::string & proc_base)
{
  std::ifstream f(proc_base + "/stat");
  if (!f.is_open()) return std::nullopt;

  std::string line;
  while (std::getline(f, line)) {
    if (line.rfind("cpu ", 0) == 0) {
      CpuTimes t;
      std::istringstream ss(line.substr(4));
      ss >> t.user >> t.nice >> t.system >> t.idle
         >> t.iowait >> t.irq >> t.softirq >> t.steal;
      return t;
    }
  }
  return std::nullopt;
}

// ── メモリ 정보 읽기 (/proc/meminfo) ─────────────────────────────────────

struct MemInfo {
  uint64_t total_kb{0}, free_kb{0}, available_kb{0}, buffers_kb{0}, cached_kb{0};
  uint64_t used_kb() const { return total_kb - available_kb; }
  double   percent() const {
    return total_kb > 0 ? 100.0 * used_kb() / total_kb : 0.0;
  }
};

static std::optional<MemInfo> read_mem_info(const std::string & proc_base)
{
  std::ifstream f(proc_base + "/meminfo");
  if (!f.is_open()) return std::nullopt;

  MemInfo m;
  std::string line;
  while (std::getline(f, line)) {
    uint64_t val = 0;
    if (std::sscanf(line.c_str(), "MemTotal: %lu kB", &val) == 1)     m.total_kb     = val;
    if (std::sscanf(line.c_str(), "MemFree: %lu kB", &val) == 1)      m.free_kb      = val;
    if (std::sscanf(line.c_str(), "MemAvailable: %lu kB", &val) == 1) m.available_kb = val;
    if (std::sscanf(line.c_str(), "Buffers: %lu kB", &val) == 1)      m.buffers_kb   = val;
    if (std::sscanf(line.c_str(), "Cached: %lu kB", &val) == 1)       m.cached_kb    = val;
  }
  return m;
}

// ── CPU 온도 읽기 (/sys/class/thermal) ───────────────────────────────────

static std::optional<double> read_cpu_temp(const std::string & sys_base)
{
  const fs::path base = sys_base + "/class/thermal";
  if (!fs::exists(base)) return std::nullopt;

  for (const auto & entry : fs::directory_iterator(base)) {
    auto type_path = entry.path() / "type";
    auto temp_path = entry.path() / "temp";
    if (!fs::exists(type_path) || !fs::exists(temp_path)) continue;

    std::ifstream tf(type_path);
    std::string type;
    std::getline(tf, type);

    if (type.find("x86_pkg_temp") != std::string::npos ||
        type.find("coretemp")     != std::string::npos ||
        type.find("cpu-thermal")  != std::string::npos ||
        type == "thermal")
    {
      std::ifstream vf(temp_path);
      int milli_deg = 0;
      if (vf >> milli_deg) {
        return milli_deg / 1000.0;
      }
    }
  }
  return std::nullopt;
}

// ── HwCheckerNode ─────────────────────────────────────────────────────────

class HwCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  HwCheckerNode()
  : robot_diagnostics_base::BaseChecker("hw_checker", "hw_checker")
  {
    base_init();  // declare/load parameters → setup tasks

    RCLCPP_INFO(
      get_logger(),
      "HwCheckerNode started. "
      "CPU warn=%.0f%% error=%.0f%% | "
      "MEM warn=%.0f%% error=%.0f%% | "
      "DISK(%s) warn=%.0f%% error=%.0f%%",
      cpu_warn_, cpu_error_,
      mem_warn_, mem_error_,
      disk_path_.c_str(), disk_warn_, disk_error_);

    if (in_container_) {
      RCLCPP_INFO(get_logger(),
        "Container environment detected. proc_base=%s  sys_base=%s  disk_path=%s",
        proc_base_.c_str(), sys_base_.c_str(), disk_path_.c_str());
    }
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("cpu.enabled",             true);
    declare_parameter("cpu.warn_threshold",      70.0);
    declare_parameter("cpu.error_threshold",     90.0);
    declare_parameter("memory.enabled",          true);
    declare_parameter("memory.warn_threshold",   75.0);
    declare_parameter("memory.error_threshold",  90.0);
    declare_parameter("disk.enabled",            true);
    declare_parameter("disk.warn_threshold",     80.0);
    declare_parameter("disk.error_threshold",    95.0);
    declare_parameter("disk.path",               std::string("/"));
    declare_parameter("cpu_temp.enabled",         true);
    declare_parameter("cpu_temp.warn_threshold",  75.0);
    declare_parameter("cpu_temp.error_threshold", 90.0);

    // 컨테이너 환경 경로 오버라이드 파라미터
    // 빈 문자열이면 자동 감지 후 /host/proc, /host/sys 등을 시도
    declare_parameter("container.host_proc_path",   std::string(""));
    declare_parameter("container.host_sys_path",    std::string(""));
    declare_parameter("container.host_rootfs_path", std::string(""));
  }

  void load_parameters_() override
  {
    cpu_enabled_    = get_parameter("cpu.enabled").as_bool();
    cpu_warn_       = get_parameter("cpu.warn_threshold").as_double();
    cpu_error_      = get_parameter("cpu.error_threshold").as_double();
    mem_enabled_    = get_parameter("memory.enabled").as_bool();
    mem_warn_       = get_parameter("memory.warn_threshold").as_double();
    mem_error_      = get_parameter("memory.error_threshold").as_double();
    disk_enabled_   = get_parameter("disk.enabled").as_bool();
    disk_warn_      = get_parameter("disk.warn_threshold").as_double();
    disk_error_     = get_parameter("disk.error_threshold").as_double();
    disk_path_      = get_parameter("disk.path").as_string();
    cpu_temp_enabled_ = get_parameter("cpu_temp.enabled").as_bool();
    cpu_temp_warn_  = get_parameter("cpu_temp.warn_threshold").as_double();
    cpu_temp_error_ = get_parameter("cpu_temp.error_threshold").as_double();

    // ── 경로 해석 ──────────────────────────────────────────────────────
    in_container_ = is_in_container();

    // 환경 변수 → 파라미터 → 자동 감지 순으로 우선순위 적용
    auto env_or_param = [this](const char * env_key, const std::string & param_key) {
      const char * env_val = std::getenv(env_key);
      if (env_val && env_val[0] != '\0') return std::string(env_val);
      return get_parameter(param_key).as_string();
    };

    std::string param_proc    = env_or_param("HOST_PROC",    "container.host_proc_path");
    std::string param_sys     = env_or_param("HOST_SYS",     "container.host_sys_path");
    std::string param_rootfs  = env_or_param("HOST_ROOTFS",  "container.host_rootfs_path");

    // proc_base_: 파라미터 > 자동 감지(/host/proc) > 기본값
    if (!param_proc.empty()) {
      proc_base_ = param_proc;
    } else if (in_container_) {
      proc_base_ = resolve_path({"/host/proc"}, "/proc");
    } else {
      proc_base_ = "/proc";
    }

    // sys_base_: 파라미터 > 자동 감지(/host/sys) > 기본값
    if (!param_sys.empty()) {
      sys_base_ = param_sys;
    } else if (in_container_) {
      sys_base_ = resolve_path({"/host/sys"}, "/sys");
    } else {
      sys_base_ = "/sys";
    }

    // disk_path_: 컨테이너에서 "/" 이면 호스트 rootfs 마운트로 교체 시도
    if (!param_rootfs.empty()) {
      disk_path_ = param_rootfs;
    } else if (in_container_ && disk_path_ == "/") {
      std::string host_rootfs = resolve_path({"/host/rootfs", "/host"}, "");
      if (!host_rootfs.empty()) {
        disk_path_ = host_rootfs;
        RCLCPP_INFO(get_logger(),
          "Container overlay FS detected; disk.path automatically set to host rootfs (%s)",
          disk_path_.c_str());
      } else {
        RCLCPP_WARN(get_logger(),
          "In a container, disk.path=\"/\" points to the overlay FS. "
          "Mount -v /:/host/rootfs:ro to monitor the host disk, then "
          "set the container.host_rootfs_path parameter.");
      }
    }

    has_cpu_temp_ = read_cpu_temp(sys_base_).has_value();
    if (!has_cpu_temp_) {
      RCLCPP_WARN(get_logger(),
        "CPU temperature sensor not found (sys_base=%s). Publishing STALE.",
        sys_base_.c_str());
    }

    // 초기 CPU 스냅샷 저장
    prev_cpu_ = read_cpu_times(proc_base_);
  }

  void setup_tasks_() override
  {
    if (cpu_enabled_)      add_task("/hardware/cpu",      [this](auto & s) { check_cpu(s); });
    if (mem_enabled_)      add_task("/hardware/memory",   [this](auto & s) { check_memory(s); });
    if (disk_enabled_)     add_task("/hardware/disk",     [this](auto & s) { check_disk(s); });
    if (cpu_temp_enabled_) add_task("/hardware/cpu_temp", [this](auto & s) { check_cpu_temp(s); });
  }

private:
  void check_cpu(StatusWrapper & stat)
  {
    auto cur = read_cpu_times(proc_base_);
    double usage = 0.0;

    if (cur && prev_cpu_) {
      uint64_t d_total  = cur->total()  - prev_cpu_->total();
      uint64_t d_active = cur->active() - prev_cpu_->active();
      usage = d_total > 0 ? 100.0 * d_active / d_total : 0.0;
    }
    prev_cpu_ = cur;

    int8_t lvl = check_high(usage, cpu_warn_, cpu_error_);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "CPU %.1f%%", usage);
    std::string msg = buf;
    if      (lvl == DiagnosticStatus::WARN)  msg += " (High)";
    else if (lvl == DiagnosticStatus::ERROR) msg += " (Critical)";

    stat.summary(lvl, msg);
    char val[32];
    std::snprintf(val, sizeof(val), "%.1f", usage);
    stat.add("usage_%", std::string(val));

    // 컨테이너 CPU 쿼터 정보 추가
    if (in_container_) {
      auto quota = read_cgroup_cpu_quota();
      if (quota) {
        std::snprintf(val, sizeof(val), "%.2f", *quota);
        stat.add("cgroup_cpu_quota_vcpus", std::string(val));
      }
      stat.add("proc_source", proc_base_);
    }
  }

  void check_memory(StatusWrapper & stat)
  {
    auto mem = read_mem_info(proc_base_);
    if (!mem) {
      stat.summary(DiagnosticStatus::STALE, proc_base_ + "/meminfo read failed");
      return;
    }

    double usage  = mem->percent();
    uint64_t used  = mem->used_kb()  / 1024;
    uint64_t total = mem->total_kb / 1024;
    uint64_t avail = mem->available_kb / 1024;

    int8_t lvl = check_high(usage, mem_warn_, mem_error_);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Memory %.1f%% (%lu / %lu MB)", usage, used, total);
    std::string msg = buf;
    if      (lvl == DiagnosticStatus::WARN)  msg += " (High)";
    else if (lvl == DiagnosticStatus::ERROR) msg += " (Critical)";

    stat.summary(lvl, msg);
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", usage); stat.add("usage_%",      std::string(tmp));
    stat.add("used_MB",      used);
    stat.add("total_MB",     total);
    stat.add("available_MB", avail);

    // 컨테이너 메모리 제한 정보 추가
    if (in_container_) {
      auto limit_kb = read_cgroup_mem_limit_kb();
      if (limit_kb) {
        uint64_t limit_mb = *limit_kb / 1024;
        stat.add("cgroup_mem_limit_MB", limit_mb);

        // cgroup 제한 기준 사용률도 함께 보고
        double cgroup_pct = *limit_kb > 0
          ? 100.0 * static_cast<double>(mem->used_kb()) / *limit_kb
          : 0.0;
        std::snprintf(tmp, sizeof(tmp), "%.1f", cgroup_pct);
        stat.add("cgroup_usage_%", std::string(tmp));
      }
      stat.add("proc_source", proc_base_);
    }
  }

  void check_disk(StatusWrapper & stat)
  {
    struct statvfs st {};
    if (statvfs(disk_path_.c_str(), &st) != 0) {
      stat.summary(DiagnosticStatus::STALE, "statvfs failed: " + disk_path_);
      return;
    }

    double total_gb  = static_cast<double>(st.f_blocks) * st.f_frsize / (1024.0 * 1024 * 1024);
    double free_gb   = static_cast<double>(st.f_bfree)  * st.f_frsize / (1024.0 * 1024 * 1024);
    double used_gb   = total_gb - free_gb;
    double usage     = total_gb > 0 ? 100.0 * used_gb / total_gb : 0.0;

    int8_t lvl = check_high(usage, disk_warn_, disk_error_);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Disk %.1f%% (%.1f / %.1f GB)", usage, used_gb, total_gb);
    std::string msg = buf;
    if      (lvl == DiagnosticStatus::WARN)  msg += " (High)";
    else if (lvl == DiagnosticStatus::ERROR) msg += " (Critical)";

    stat.summary(lvl, msg);
    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.1f", usage);   stat.add("usage_%",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", used_gb); stat.add("used_GB",  std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", total_gb);stat.add("total_GB", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.1f", free_gb); stat.add("free_GB",  std::string(tmp));
    stat.add("path", disk_path_);
  }

  void check_cpu_temp(StatusWrapper & stat)
  {
    if (!has_cpu_temp_) {
      stat.summary(DiagnosticStatus::STALE,
        "CPU temperature sensor missing (sys_base=" + sys_base_ + ")");
      return;
    }

    auto temp_opt = read_cpu_temp(sys_base_);
    if (!temp_opt) {
      stat.summary(DiagnosticStatus::STALE, "CPU temperature sensor missing");
      return;
    }

    double temp = *temp_opt;
    int8_t lvl  = check_high(temp, cpu_temp_warn_, cpu_temp_error_);

    char buf[64];
    std::snprintf(buf, sizeof(buf), "CPU Temp %.1f C", temp);
    std::string msg = buf;
    if      (lvl == DiagnosticStatus::WARN)  msg += " (High)";
    else if (lvl == DiagnosticStatus::ERROR) msg += " (Critical)";

    stat.summary(lvl, msg);
    char tmp[16];
    std::snprintf(tmp, sizeof(tmp), "%.1f", temp);
    stat.add("temp_C", std::string(tmp));
  }

  // ── 파라미터 멤버 ──────────────────────────────────────────────────────
  bool   cpu_enabled_{true},      mem_enabled_{true};
  bool   disk_enabled_{true},     cpu_temp_enabled_{true};
  double cpu_warn_{70.0},      cpu_error_{90.0};
  double mem_warn_{75.0},      mem_error_{90.0};
  double disk_warn_{80.0},     disk_error_{95.0};
  double cpu_temp_warn_{75.0}, cpu_temp_error_{90.0};
  std::string disk_path_{"/"};
  bool   has_cpu_temp_{false};

  // ── 컨테이너 관련 멤버 ────────────────────────────────────────────────
  bool        in_container_{false};
  std::string proc_base_{"/proc"};  // 호스트 /proc 마운트 경로
  std::string sys_base_{"/sys"};    // 호스트 /sys 마운트 경로

  // ── 상태 멤버 ─────────────────────────────────────────────────────────
  std::optional<CpuTimes> prev_cpu_;
};

// ── main ──────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HwCheckerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
