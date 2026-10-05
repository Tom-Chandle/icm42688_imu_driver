/**
 * @brief ICM42688同一FIFO包组合采集。
 * 链路：FIFO -> INT2/GPIO0_PC3硬IRQ -> 内核时间重建 -> 组合IIO buffer -> ROS2。
 * 用户态poll/read/发布时刻不参与消息时间戳计算。
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr std::size_t kScanSize = 24; // 6*int16 + temp + pad + timestamp
struct Sample { int16_t ax, ay, az, gx, gy, gz, temp; int64_t timestamp; };
int16_t be16(const uint8_t *p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
Sample decode(const uint8_t *p) {
  int64_t ts;
  std::memcpy(&ts, p + 16, sizeof(ts)); // IIO时间戳为本机序且8字节对齐
  return {be16(p), be16(p+2), be16(p+4), be16(p+6), be16(p+8),
    be16(p+10), be16(p+12), ts};
}
std::string read_text(const std::string &path, bool required=true) {
  std::ifstream f(path); std::string value;
  if (!f || !std::getline(f, value)) {
    if (required) throw std::runtime_error("Cannot read " + path);
    return {};
  }
  while (!value.empty() && (value.back() == '\r' || value.back() == ' ')) value.pop_back();
  return value;
}
void write_text(const std::string &path, const std::string &value) {
  std::ofstream f(path);
  if (!f) throw std::runtime_error("Cannot open " + path + " for writing");
  f << value; f.flush();
  if (!f) throw std::runtime_error("Cannot write " + path);
}
std::string dev_path(const std::string &sysfs) {
  const auto slash = sysfs.find_last_of('/');
  if (slash == std::string::npos || sysfs.substr(slash+1).rfind("iio:device", 0) != 0)
    throw std::runtime_error("Invalid IIO path: " + sysfs);
  return "/dev/" + sysfs.substr(slash+1);
}
} // namespace

class Driver final : public rclcpp::Node {
public:
  Driver() : Node("icm42688_driver") {
    declare_parameter("frame_id", "imu_link");
    declare_parameter("topic", "/imu/data_raw");
    declare_parameter("auto_detect", true);
    declare_parameter("acc_device_path", "/sys/bus/iio/devices/iio:device2");
    declare_parameter("gyro_device_path", "/sys/bus/iio/devices/iio:device1");
    declare_parameter("imu_device_path", "/sys/bus/iio/devices/iio:device3");
    declare_parameter("acc_sampling_frequency", 200.0);
    declare_parameter("gyro_sampling_frequency", 200.0);
    declare_parameter("accel_filter", "odr-div-10");
    declare_parameter("gyro_filter", "odr-div-10");
    declare_parameter("acc_scale", 0.0);
    declare_parameter("gyro_scale", 0.0);
    declare_parameter("buffer_length", 128);
    declare_parameter("timestamp_clock", "monotonic");
    declare_parameter("poll_timeout_ms", 1000);
    frame_ = get_parameter("frame_id").as_string();
    acc_ = get_parameter("acc_device_path").as_string();
    gyro_ = get_parameter("gyro_device_path").as_string();
    imu_ = get_parameter("imu_device_path").as_string();
    afreq_ = get_parameter("acc_sampling_frequency").as_double();
    gfreq_ = get_parameter("gyro_sampling_frequency").as_double();
    afilter_ = get_parameter("accel_filter").as_string();
    gfilter_ = get_parameter("gyro_filter").as_string();
    req_ascale_ = get_parameter("acc_scale").as_double();
    req_gscale_ = get_parameter("gyro_scale").as_double();
    length_ = get_parameter("buffer_length").as_int();
    clock_ = get_parameter("timestamp_clock").as_string();
    timeout_ = get_parameter("poll_timeout_ms").as_int();
    if (afreq_ != gfreq_) throw std::runtime_error("accel/gyro ODR must match");
    if (get_parameter("auto_detect").as_bool()) detect();
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      get_parameter("topic").as_string(), rclcpp::SensorDataQoS());
    try { configure(); } catch (...) { close_fd(); disable(); throw; }
    running_ = true;
    thread_ = std::thread(&Driver::loop, this);
  }
  ~Driver() override {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    close_fd(); disable();
  }
private:
  void detect() {
    DIR *dir = opendir("/sys/bus/iio/devices");
    if (!dir) throw std::runtime_error("Cannot open IIO sysfs");
    while (auto *e = readdir(dir)) {
      std::string item(e->d_name);
      if (item.rfind("iio:device", 0) != 0) continue;
      std::string path = "/sys/bus/iio/devices/" + item;
      std::string name = read_text(path + "/name", false);
      if (name == "icm42688-accel") acc_ = path;
      else if (name == "icm42688-gyro") gyro_ = path;
      else if (name == "icm42688-imu") imu_ = path;
    }
    closedir(dir);
    if (acc_.empty() || gyro_.empty() || imu_.empty())
      throw std::runtime_error("ICM42688 accel/gyro/imu IIO device missing");
  }
  static std::string freq(double value) {
    auto integer = static_cast<int64_t>(value);
    return value == integer ? std::to_string(integer) : std::to_string(value);
  }
  void sensor(const std::string &path, bool accel) {
    write_text(path+"/buffer/enable", "0");
    write_text(path+"/current_timestamp_clock", clock_);
    write_text(path+"/sampling_frequency", freq(accel ? afreq_ : gfreq_));
    if (accel) {
      write_text(path+"/in_accel_power_mode", "low-noise");
      write_text(path+"/in_accel_filter", afilter_);
      if (req_ascale_ > 0) write_text(path+"/in_accel_scale", std::to_string(req_ascale_));
    } else {
      write_text(path+"/in_anglvel_filter", gfilter_);
      if (req_gscale_ > 0) write_text(path+"/in_anglvel_scale", std::to_string(req_gscale_));
    }
    write_text(path+"/buffer/length", std::to_string(length_));
    const std::string type = accel ? "accel" : "anglvel";
    for (char axis : {'x','y','z'})
      write_text(path+"/scan_elements/in_"+type+"_"+axis+"_en", "1");
    write_text(path+"/scan_elements/in_temp_en", "1");
    write_text(path+"/scan_elements/in_timestamp_en", "1");
  }
  void combined() {
    write_text(imu_+"/buffer/enable", "0");
    write_text(imu_+"/current_timestamp_clock", clock_);
    write_text(imu_+"/buffer/length", std::to_string(length_));
    for (char axis : {'x','y','z'}) {
      write_text(imu_+"/scan_elements/in_accel_"+axis+"_en", "1");
      write_text(imu_+"/scan_elements/in_anglvel_"+axis+"_en", "1");
    }
    write_text(imu_+"/scan_elements/in_temp_en", "1");
    write_text(imu_+"/scan_elements/in_timestamp_en", "1");
  }
  void configure() {
    sensor(acc_, true); acc_on_ = true;
    sensor(gyro_, false); gyro_on_ = true;
    combined(); imu_on_ = true;
    ascale_ = std::stod(read_text(acc_+"/in_accel_scale"));
    gscale_ = std::stod(read_text(gyro_+"/in_anglvel_scale"));
    // 原accel/gyro设备拥有硬件FIFO生命周期；组合设备是被动输出端。
    write_text(acc_+"/buffer/enable", "1");
    write_text(gyro_+"/buffer/enable", "1");
    write_text(imu_+"/buffer/enable", "1");
    device_ = dev_path(imu_);
    fd_ = open(device_.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("Cannot open "+device_+": "+std::strerror(errno));
    RCLCPP_INFO(get_logger(), "combined IRQ IIO=%s timestamp=%s",
      device_.c_str(), clock_.c_str());
  }
  void close_fd() noexcept { if (fd_ >= 0) { close(fd_); fd_ = -1; } }
  void disable() noexcept {
    try { if (imu_on_) write_text(imu_+"/buffer/enable", "0"); } catch (...) {}
    try { if (gyro_on_) write_text(gyro_+"/buffer/enable", "0"); } catch (...) {}
    try { if (acc_on_) write_text(acc_+"/buffer/enable", "0"); } catch (...) {}
  }
  void loop() {
    pollfd pfd{fd_, POLLIN, 0};
    std::vector<uint8_t> pending;
    uint8_t bytes[kScanSize*32];
    while (running_) {
      int rc = poll(&pfd, 1, static_cast<int>(timeout_));
      if (rc < 0) { if (errno == EINTR) continue; break; }
      if (rc == 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No combined IIO data");
        continue;
      }
      if (!(pfd.revents & POLLIN)) continue;
      ssize_t count = read(fd_, bytes, sizeof(bytes));
      if (count <= 0) continue;
      pending.insert(pending.end(), bytes, bytes+count);
      std::size_t frames = pending.size()/kScanSize;
      for (std::size_t i=0; i<frames; ++i) publish(decode(pending.data()+i*kScanSize));
      pending.erase(pending.begin(), pending.begin()+frames*kScanSize);
    }
  }
  void publish(const Sample &s) {
    // 严禁使用Node::now()、read()或publish()时刻作为回退。
    if (s.timestamp <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Invalid kernel IRQ timestamp");
      return;
    }
    sensor_msgs::msg::Imu msg;
    msg.header.stamp.sec = static_cast<int32_t>(s.timestamp/1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(s.timestamp%1000000000LL);
    msg.header.frame_id = frame_;
    msg.orientation_covariance[0] = -1.0;
    msg.linear_acceleration.x=s.ax*ascale_; msg.linear_acceleration.y=s.ay*ascale_;
    msg.linear_acceleration.z=s.az*ascale_;
    msg.angular_velocity.x=s.gx*gscale_; msg.angular_velocity.y=s.gy*gscale_;
    msg.angular_velocity.z=s.gz*gscale_;
    publisher_->publish(msg);
  }
  std::string frame_, acc_, gyro_, imu_, device_, afilter_, gfilter_, clock_;
  double afreq_{}, gfreq_{}, req_ascale_{}, req_gscale_{}, ascale_{}, gscale_{};
  int64_t length_{}, timeout_{}; int fd_{-1};
  bool acc_on_{false}, gyro_on_{false}, imu_on_{false};
  std::atomic<bool> running_{false}; std::thread thread_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<Driver>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("icm42688_driver"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
