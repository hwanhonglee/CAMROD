/**
 * Network Checker Node
 *
 * WiFi 인터페이스의 연결 상태, 신호 품질, 패킷 손실률을 모니터링하여
 * /diagnostics 토픽으로 발행한다.
 *
 * 데이터 소스
 * -----------
 *   /sys/class/net/<iface>/operstate  — 링크 상태 (up/down/unknown)
 *   /sys/class/net/<iface>/carrier    — 캐리어 상태 (1=연결, 0=끊김)
 *   /proc/net/wireless                — RSSI(dBm), 링크 품질, 노이즈
 *   /proc/net/dev                     — rx/tx 패킷·에러·드롭 카운터
 *
 * 진단 항목
 * ---------
 *   /hardware/network/connection  — 링크 up/down, 캐리어 유무
 *   /hardware/network/signal      — RSSI (dBm), 링크 품질
 *   /hardware/network/quality     — 패킷 손실률, 에러율
 */

#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <robot_diagnostics_base/base_checker.hpp>

// HH_260721 - Use explicit ROS interface types at publisher, subscriber, and diagnostic boundaries.
using StatusWrapper    = diagnostic_updater::DiagnosticStatusWrapper;

// ── /proc/net/wireless 파싱 ───────────────────────────────────────────────

struct WirelessInfo {
  double link_quality_pct{0.0};  // 링크 품질 (0~100 %)
  double signal_dbm{0.0};        // 신호 세기 (dBm, 음수)
  double noise_dbm{0.0};         // 노이즈 레벨 (dBm, 음수)
  bool   valid{false};
};

/**
 * /proc/net/wireless 에서 지정 인터페이스의 WiFi 상태를 읽는다.
 *
 * 파일 형식:
 *   Inter-| sta-|   Quality        |   Discarded packets  ...
 *    face | tus | link level noise | ...
 *   wlan0: 0000   57.  -53.  -95.  ...
 *
 * link 최댓값은 드라이버마다 다르나 일반적으로 70 또는 100 사용.
 * 여기서는 값 자체가 70 초과이면 100 기준, 이하이면 70 기준으로 정규화.
 */
static WirelessInfo read_wireless(const std::string & iface)
{
  std::ifstream f("/proc/net/wireless");
  if (!f.is_open()) return {};

  std::string line;
  while (std::getline(f, line)) {
    // 인터페이스 이름 매칭 ("wlan0:" 형태)
    auto pos = line.find(iface + ":");
    if (pos == std::string::npos) continue;

    // 인터페이스 이름 이후 파싱
    std::istringstream ss(line.substr(pos + iface.size() + 1));
    int    status = 0;
    double link = 0.0, level = 0.0, noise = 0.0;
    ss >> status >> link >> level >> noise;

    // 끝에 붙는 '.' 제거는 ss >> double 이 자동 처리
    // level/noise 값이 양수로 읽히면 256 오프셋 보정 (일부 커널)
    if (level > 0)  level  -= 256.0;
    if (noise > 0)  noise  -= 256.0;

    // link quality 정규화: 70 이하이면 max=70, 아니면 max=100
    double max_link = (link <= 70.0) ? 70.0 : 100.0;
    double pct = (max_link > 0.0) ? (link / max_link * 100.0) : 0.0;
    if (pct > 100.0) pct = 100.0;

    return WirelessInfo{pct, level, noise, true};
  }
  return {};
}

// ── /proc/net/dev 파싱 ────────────────────────────────────────────────────

struct NetDevStats {
  uint64_t rx_packets{0};
  uint64_t rx_errs{0};
  uint64_t rx_drop{0};
  uint64_t tx_packets{0};
  uint64_t tx_errs{0};
  uint64_t tx_drop{0};
  bool     valid{false};
};

/**
 * /proc/net/dev 에서 지정 인터페이스의 패킷 카운터를 읽는다.
 *
 * 필드 순서 (인터페이스명: 이후):
 *   rx_bytes rx_packets rx_errs rx_drop rx_fifo rx_frame rx_compressed rx_multicast
 *   tx_bytes tx_packets tx_errs tx_drop ...
 */
static NetDevStats read_netdev(const std::string & iface)
{
  std::ifstream f("/proc/net/dev");
  if (!f.is_open()) return {};

  std::string line;
  while (std::getline(f, line)) {
    auto pos = line.find(iface + ":");
    if (pos == std::string::npos) continue;

    std::istringstream ss(line.substr(pos + iface.size() + 1));
    uint64_t rx_bytes, rx_packets, rx_errs, rx_drop, rx_fifo, rx_frame,
             rx_compressed, rx_multicast;
    uint64_t tx_bytes, tx_packets, tx_errs, tx_drop;
    ss >> rx_bytes >> rx_packets >> rx_errs >> rx_drop
       >> rx_fifo  >> rx_frame   >> rx_compressed >> rx_multicast
       >> tx_bytes >> tx_packets >> tx_errs >> tx_drop;

    return NetDevStats{
      rx_packets, rx_errs, rx_drop,
      tx_packets, tx_errs, tx_drop,
      true
    };
  }
  return {};
}

// ── 링크/캐리어 상태 읽기 (/sys/class/net) ────────────────────────────────

static std::string read_sysfs_string(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) return "";
  std::string val;
  std::getline(f, val);
  return val;
}

// ── NetworkCheckerNode ────────────────────────────────────────────────────

class NetworkCheckerNode : public robot_diagnostics_base::BaseChecker
{
public:
  explicit NetworkCheckerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : robot_diagnostics_base::BaseChecker("network_checker", "network_checker", options)
  {
    base_init();

    RCLCPP_INFO(
      get_logger(),
      "NetworkCheckerNode started. iface=%s | "
      "RSSI warn=%.0f error=%.0f dBm | "
      "drop_rate warn=%.1f error=%.1f %%",
      iface_.c_str(),
      rssi_warn_, rssi_error_,
      drop_warn_, drop_error_);
  }

protected:
  void declare_parameters_() override
  {
    declare_parameter("interface",                 std::string("wlan0"));
    declare_parameter("connection.enabled",        true);
    declare_parameter("signal.enabled",            true);
    declare_parameter("signal.rssi_warn",          -70.0);   // dBm
    declare_parameter("signal.rssi_error",         -85.0);   // dBm
    declare_parameter("quality.enabled",           true);
    declare_parameter("quality.drop_rate_warn",    1.0);     // %
    declare_parameter("quality.drop_rate_error",   5.0);     // %
  }

  void load_parameters_() override
  {
    iface_       = get_parameter("interface").as_string();
    conn_enabled_ = get_parameter("connection.enabled").as_bool();
    sig_enabled_  = get_parameter("signal.enabled").as_bool();
    rssi_warn_    = get_parameter("signal.rssi_warn").as_double();
    rssi_error_   = get_parameter("signal.rssi_error").as_double();
    qual_enabled_ = get_parameter("quality.enabled").as_bool();
    drop_warn_    = get_parameter("quality.drop_rate_warn").as_double();
    drop_error_   = get_parameter("quality.drop_rate_error").as_double();

    // 초기 패킷 카운터 스냅샷
    prev_stats_ = read_netdev(iface_);
  }

  void setup_tasks_() override
  {
    if (conn_enabled_) add_task("/hardware/network/connection",
                                [this](auto & s) { check_connection(s); });
    if (sig_enabled_)  add_task("/hardware/network/signal",
                                [this](auto & s) { check_signal(s); });
    if (qual_enabled_) add_task("/hardware/network/quality",
                                [this](auto & s) { check_quality(s); });
  }

private:
  // ── /hardware/network/connection ──────────────────────────────────────
  void check_connection(StatusWrapper & stat)
  {
    const std::string base = "/sys/class/net/" + iface_;

    // 인터페이스 존재 여부
    if (access(base.c_str(), F_OK) != 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                   "Interface missing: " + iface_);
      stat.add("interface", iface_);
      return;
    }

    std::string operstate = read_sysfs_string(base + "/operstate");
    std::string carrier_s = read_sysfs_string(base + "/carrier");
    bool carrier = (carrier_s == "1");

    stat.add("interface",  iface_);
    stat.add("operstate",  operstate.empty() ? "unknown" : operstate);
    stat.add("carrier",    carrier ? "connected" : "no carrier");

    if (operstate == "up" && carrier) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "WiFi connected (" + iface_ + ")");
    } else if (operstate == "down") {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Interface down: " + iface_);
    } else {
      // operstate == "unknown" 또는 carrier unavailable
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                   "Link unstable (operstate=" + operstate + ")");
    }
  }

  // ── /hardware/network/signal ───────────────────────────────────────────
  void check_signal(StatusWrapper & stat)
  {
    stat.add("interface", iface_);

    // /proc/net/wireless 에 항목이 없으면 WiFi 미연결 상태
    auto info = read_wireless(iface_);
    if (!info.valid) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                   "/proc/net/wireless missing " + iface_ + " (disconnected?)");
      return;
    }

    char tmp[32];

    std::snprintf(tmp, sizeof(tmp), "%.0f", info.signal_dbm);
    stat.add("rssi_dBm", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.1f", info.link_quality_pct);
    stat.add("link_quality_%", std::string(tmp));

    std::snprintf(tmp, sizeof(tmp), "%.0f", info.noise_dbm);
    stat.add("noise_dBm", std::string(tmp));

    // RSSI 기준 레벨 판정 (낮을수록 나쁨 → check_low 사용)
    int8_t lvl = check_low(info.signal_dbm, rssi_warn_, rssi_error_);

    char msg[64];
    std::snprintf(msg, sizeof(msg),
                  "RSSI %.0f dBm quality %.0f%%",
                  info.signal_dbm, info.link_quality_pct);
    std::string msg_str = msg;
    if      (lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str += " (Weak)";
    else if (lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str += " (Very Weak)";

    stat.summary(lvl, msg_str);
  }

  // ── /hardware/network/quality ─────────────────────────────────────────
  void check_quality(StatusWrapper & stat)
  {
    stat.add("interface", iface_);

    auto cur = read_netdev(iface_);
    if (!cur.valid) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                   "/proc/net/dev missing " + iface_ + "");
      prev_stats_ = cur;
      return;
    }

    double rx_drop_rate = 0.0, tx_drop_rate = 0.0;
    double rx_err_rate  = 0.0, tx_err_rate  = 0.0;

    if (prev_stats_.valid) {
      uint64_t d_rx_pkt  = cur.rx_packets - prev_stats_.rx_packets;
      uint64_t d_tx_pkt  = cur.tx_packets - prev_stats_.tx_packets;
      uint64_t d_rx_drop = cur.rx_drop    - prev_stats_.rx_drop;
      uint64_t d_tx_drop = cur.tx_drop    - prev_stats_.tx_drop;
      uint64_t d_rx_err  = cur.rx_errs    - prev_stats_.rx_errs;
      uint64_t d_tx_err  = cur.tx_errs    - prev_stats_.tx_errs;

      if (d_rx_pkt > 0) rx_drop_rate = 100.0 * d_rx_drop / d_rx_pkt;
      if (d_tx_pkt > 0) tx_drop_rate = 100.0 * d_tx_drop / d_tx_pkt;
      if (d_rx_pkt > 0) rx_err_rate  = 100.0 * d_rx_err  / d_rx_pkt;
      if (d_tx_pkt > 0) tx_err_rate  = 100.0 * d_tx_err  / d_tx_pkt;
    }
    prev_stats_ = cur;

    char tmp[32];
    std::snprintf(tmp, sizeof(tmp), "%.2f", rx_drop_rate);
    stat.add("rx_drop_%", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", tx_drop_rate);
    stat.add("tx_drop_%", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", rx_err_rate);
    stat.add("rx_err_%", std::string(tmp));
    std::snprintf(tmp, sizeof(tmp), "%.2f", tx_err_rate);
    stat.add("tx_err_%", std::string(tmp));
    stat.add("rx_packets_total", cur.rx_packets);
    stat.add("tx_packets_total", cur.tx_packets);

    // rx drop rate 기준으로 레벨 판정
    double worst_drop = std::max(rx_drop_rate, tx_drop_rate);
    int8_t lvl = check_high(worst_drop, drop_warn_, drop_error_);

    char msg[80];
    std::snprintf(msg, sizeof(msg),
                  "RX drop %.2f%%  TX drop %.2f%%",
                  rx_drop_rate, tx_drop_rate);
    std::string msg_str = msg;
    if      (lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN)  msg_str += " (High drop)";
    else if (lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) msg_str += " (Critical drop)";

    stat.summary(lvl, msg_str);
  }

  // ── 파라미터 멤버 ──────────────────────────────────────────────────────
  std::string iface_{"wlan0"};
  bool   conn_enabled_{true}, sig_enabled_{true}, qual_enabled_{true};
  double rssi_warn_{-70.0}, rssi_error_{-85.0};
  double drop_warn_{1.0},   drop_error_{5.0};

  // ── 상태 멤버 ─────────────────────────────────────────────────────────
  NetDevStats prev_stats_;
};

#include "camrod_system/checker_entrypoint.hpp"
CAMROD_SYSTEM_CHECKER_ENTRYPOINT(NetworkCheckerNode)
