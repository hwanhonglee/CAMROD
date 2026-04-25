#include <libobsensor/ObSensor.hpp>

#include <array>
#include <cctype>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
  bool list = false;
  bool force_ip = false;
  bool open_test = false;
  int index = -1;
  std::string mac;
  std::string ip = "192.168.1.10";
  std::string mask = "255.255.255.0";
  std::string gateway = "0.0.0.0";
};

void PrintUsage(const char* argv0) {
  std::cout
      << "Orbbec Force IP Tool\n\n"
      << "Usage:\n"
      << "  " << argv0 << " --list\n"
      << "  " << argv0 << " --open-test --index 0\n"
      << "  " << argv0 << " --open-test --ip 192.168.1.10\n"
      << "  " << argv0
      << " --force-ip --index 0 --ip 192.168.1.10 --mask 255.255.255.0 --gateway 0.0.0.0\n"
      << "  " << argv0
      << " --force-ip --mac xx:xx:xx:xx:xx:xx --ip 192.168.1.10 --mask 255.255.255.0 --gateway 0.0.0.0\n";
}

bool ParseIpv4(const std::string& text, std::array<uint8_t, 4>* out) {
  if (!out) return false;
  std::array<uint8_t, 4> bytes{};
  size_t start = 0;
  for (int i = 0; i < 4; ++i) {
    const size_t dot = text.find('.', start);
    const std::string token =
        text.substr(start, dot == std::string::npos ? dot : dot - start);
    if (token.empty()) return false;
    for (const char c : token) {
      if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    }
    int value = -1;
    try {
      value = std::stoi(token);
    } catch (...) {
      return false;
    }
    if (value < 0 || value > 255) return false;
    bytes[i] = static_cast<uint8_t>(value);
    if (i < 3) {
      if (dot == std::string::npos) return false;
      start = dot + 1;
    }
  }
  *out = bytes;
  return true;
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") return false;
    if (arg == "--list") {
      options->list = true;
      continue;
    }
    if (arg == "--force-ip") {
      options->force_ip = true;
      continue;
    }
    if (arg == "--open-test") {
      options->open_test = true;
      continue;
    }
    if (arg == "--index" && i + 1 < argc) {
      options->index = std::stoi(argv[++i]);
      continue;
    }
    if (arg == "--mac" && i + 1 < argc) {
      options->mac = argv[++i];
      continue;
    }
    if (arg == "--ip" && i + 1 < argc) {
      options->ip = argv[++i];
      continue;
    }
    if (arg == "--mask" && i + 1 < argc) {
      options->mask = argv[++i];
      continue;
    }
    if (arg == "--gateway" && i + 1 < argc) {
      options->gateway = argv[++i];
      continue;
    }
    std::cerr << "[ERR] Unknown argument: " << arg << "\n";
    return false;
  }

  const int selected_modes = static_cast<int>(options->list) +
                             static_cast<int>(options->force_ip) +
                             static_cast<int>(options->open_test);
  if (selected_modes != 1) {
    std::cerr << "[ERR] Choose exactly one of --list, --force-ip or --open-test.\n";
    return false;
  }
  return true;
}

bool IsEthernetDevice(const std::shared_ptr<ob::DeviceList>& list, uint32_t i) {
  const std::string connection = list->getConnectionType(i);
  return connection == "Ethernet";
}

void PrintDeviceList(const std::shared_ptr<ob::DeviceList>& list) {
  std::cout << "Ethernet device list:\n";
  int shown = 0;
  for (uint32_t i = 0; i < list->getCount(); ++i) {
    if (!IsEthernetDevice(list, i)) continue;
    ++shown;
    std::cout << "[" << i << "] "
              << "Name=" << list->getName(i)
              << " SN=" << list->getSerialNumber(i)
              << " MAC=" << list->getUid(i)
              << " IP=" << list->getIpAddress(i)
              << " Mask=" << list->getSubnetMask(i)
              << " Gateway=" << list->getGateway(i)
              << " LocalIP=" << list->getLocalIP(i)
              << "\n";
  }
  if (shown == 0) {
    std::cout << "No Ethernet Orbbec devices discovered.\n";
  }
}

OBNetIpConfig MakeConfig(const Options& options) {
  OBNetIpConfig config{};
  std::array<uint8_t, 4> ip{};
  std::array<uint8_t, 4> mask{};
  std::array<uint8_t, 4> gateway{};

  if (!ParseIpv4(options.ip, &ip) || !ParseIpv4(options.mask, &mask) ||
      !ParseIpv4(options.gateway, &gateway)) {
    throw std::runtime_error("Invalid IPv4 address/mask/gateway.");
  }

  config.dhcp = 0;
  std::memcpy(config.address, ip.data(), ip.size());
  std::memcpy(config.mask, mask.data(), mask.size());
  std::memcpy(config.gateway, gateway.data(), gateway.size());
  return config;
}

std::string ResolveMac(const std::shared_ptr<ob::DeviceList>& list,
                       const Options& options) {
  if (!options.mac.empty()) return options.mac;
  if (options.index < 0 || static_cast<uint32_t>(options.index) >= list->getCount()) {
    throw std::runtime_error("Provide --mac or a valid --index.");
  }
  if (!IsEthernetDevice(list, static_cast<uint32_t>(options.index))) {
    throw std::runtime_error("Selected device is not Ethernet.");
  }
  return list->getUid(static_cast<uint32_t>(options.index));
}

const char* AccessModeName(const OBDeviceAccessMode mode) {
  switch (mode) {
    case OB_DEVICE_EXCLUSIVE_ACCESS:
      return "exclusive";
    case OB_DEVICE_CONTROL_ACCESS:
      return "control";
    case OB_DEVICE_MONITOR_ACCESS:
      return "monitor";
    case OB_DEVICE_DEFAULT_ACCESS:
      return "default";
    default:
      return "unknown";
  }
}

void RunOpenTest(ob::Context* context,
                 const std::shared_ptr<ob::DeviceList>& list,
                 const Options& options) {
  const OBDeviceAccessMode modes[] = {
      OB_DEVICE_DEFAULT_ACCESS,
      OB_DEVICE_CONTROL_ACCESS,
      OB_DEVICE_MONITOR_ACCESS,
      OB_DEVICE_EXCLUSIVE_ACCESS,
  };

  for (const auto mode : modes) {
    try {
      bool ok = false;
      if (options.index >= 0) {
        ok = static_cast<bool>(
            list->getDevice(static_cast<uint32_t>(options.index), mode));
        std::cout << "[TEST] list index=" << options.index
                  << " access=" << AccessModeName(mode)
                  << " => " << (ok ? "OK" : "null") << "\n";
      } else {
        ok = static_cast<bool>(context->createNetDevice(
            options.ip.c_str(), 8090, mode));
        std::cout << "[TEST] ip=" << options.ip
                  << " access=" << AccessModeName(mode)
                  << " => " << (ok ? "OK" : "null") << "\n";
      }
    } catch (const ob::Error& e) {
      std::cout << "[TEST] "
                << (options.index >= 0 ? "list" : "ip")
                << " access=" << AccessModeName(mode)
                << " => FAIL function=" << e.getFunction()
                << " message=" << e.what() << "\n";
    }
  }
}

}  // namespace

int main(int argc, char** argv) try {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    PrintUsage(argv[0]);
    return 1;
  }

  ob::Context context;
  context.enableNetDeviceEnumeration(true);
  auto list = context.queryDeviceList();
  if (!list) {
    std::cerr << "[ERR] Failed to query device list.\n";
    return 1;
  }

  if (options.list) {
    PrintDeviceList(list);
    return 0;
  }

  if (options.open_test) {
    PrintDeviceList(list);
    RunOpenTest(&context, list, options);
    return 0;
  }

  const std::string mac = ResolveMac(list, options);
  const OBNetIpConfig config = MakeConfig(options);
  const bool ok = context.forceIp(mac.c_str(), config);
  if (!ok) {
    std::cerr << "[ERR] ForceIP command was rejected by the device.\n";
    return 1;
  }

  std::cout << "[OK] ForceIP applied to " << mac
            << " -> ip=" << options.ip
            << " mask=" << options.mask
            << " gateway=" << options.gateway << "\n";
  std::cout << "[INFO] Gemini 335Le may lose this IP after reboot.\n";
  return 0;
} catch (const ob::Error& e) {
  std::cerr << "[ERR] Orbbec SDK failure\n"
            << "  function: " << e.getFunction() << "\n"
            << "  args    : " << e.getArgs() << "\n"
            << "  message : " << e.what() << "\n"
            << "  type    : " << e.getExceptionType() << "\n";
  return 1;
} catch (const std::exception& e) {
  std::cerr << "[ERR] " << e.what() << "\n";
  return 1;
}
