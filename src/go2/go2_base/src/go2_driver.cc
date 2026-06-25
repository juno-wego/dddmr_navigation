#include "go2_base/go2_driver.hpp"
#include <net/if.h>  
#include <ifaddrs.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>


static bool iface_up_with_ipv4(const std::string& iface, std::string* out_ip=nullptr) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return false;

  ifreq ifr{};
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.c_str());

  // flags (UP/RUNNING)
  if (ioctl(fd, SIOCGIFFLAGS, &ifr) < 0) { close(fd); return false; }
  bool up = (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
  if (!up) { close(fd); return false; }

  // IPv4 address
  if (ioctl(fd, SIOCGIFADDR, &ifr) < 0) { close(fd); return false; }
  if (out_ip) {
    auto* addr = (sockaddr_in*)&ifr.ifr_addr;
    *out_ip = inet_ntoa(addr->sin_addr);
  }
  close(fd);
  return true;
}

static bool wait_for_iface_ready(const std::string& iface,
                                 const rclcpp::Logger& logger,
                                 int max_backoff_sec = 8)
{
  int backoff = 1;
  while (rclcpp::ok()) {
    if (if_nametoindex(iface.c_str()) != 0) {
      std::string ip;
      if (iface_up_with_ipv4(iface, &ip)) {
        RCLCPP_INFO(logger, "[Go2] iface ready: %s (IPv4=%s)", iface.c_str(), ip.c_str());
        return true;
      } else {
        RCLCPP_WARN(logger, "[Go2] iface %s exists but not UP/RUNNING or no IPv4. retry in %ds",
                    iface.c_str(), backoff);
      }
    } else {
      RCLCPP_WARN(logger, "[Go2] iface %s not found; retry in %ds", iface.c_str(), backoff);
    }
    rclcpp::sleep_for(std::chrono::seconds(backoff));
    backoff = std::min(backoff * 2, max_backoff_sec);
  }
  return false;
}

static bool init_unitree_blocking(const std::string& iface,
                                  const rclcpp::Logger& logger,
                                  int domain = 0,
                                  int max_backoff_sec = 8)
{
  int backoff = 1;
  while (rclcpp::ok()) {
    try {
      unitree::robot::ChannelFactory::Instance()->Init(domain, iface);
      RCLCPP_INFO(logger, "[Go2] ChannelFactory Init OK (iface=%s)", iface.c_str());
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(logger, "[Go2] ChannelFactory init failed: %s (retry in %ds)",
                  e.what(), backoff);
    } catch (...) {
      RCLCPP_WARN(logger, "[Go2] ChannelFactory init failed (unknown). retry in %ds",
                  backoff);
    }
    rclcpp::sleep_for(std::chrono::seconds(backoff));
    backoff = std::min(backoff * 2, max_backoff_sec);
  }
  return false;
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto logger = rclcpp::get_logger("go2_main");

  std::string interface_name = "enp60s0";
  if (argc > 1) { interface_name = argv[1]; }

  RCLCPP_INFO(logger, "[Go2] target iface: %s", interface_name.c_str());

  if (!wait_for_iface_ready(interface_name, logger)) {
    RCLCPP_ERROR(logger, "[Go2] aborted while waiting for iface ready (UP+IPv4)");
    rclcpp::shutdown();
    return 2;
  }

  RCLCPP_INFO(logger, "[Go2] calling ChannelFactory::Init...");
  if (!init_unitree_blocking(interface_name, logger, /*domain=*/0)) {
    RCLCPP_ERROR(logger, "[Go2] aborted while initializing ChannelFactory");
    rclcpp::shutdown();
    return 3;
  }
  RCLCPP_INFO(logger, "[Go2] ChannelFactory::Init done.");

  auto node = std::make_shared<Go2Driver>();
  RCLCPP_INFO(logger, "[Go2] Go2Driver constructed.");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}