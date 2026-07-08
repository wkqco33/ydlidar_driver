#include "x4pro_lidar.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <mutex>

static constexpr double DEG2RAD = M_PI / 180.0;

/// YdlidarNode
/// - X4 Pro 전용 ROS2 드라이버 노드
/// - 외부 SDK 없이 UART 직접 통신
class YdlidarNode : public rclcpp::Node
{
public:
  explicit YdlidarNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("ydlidar_node", options)
  {
    // ---- 파라미터 선언 ----
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<int>("baudrate", 128000);
    this->declare_parameter<double>("angle_min", -180.0);
    this->declare_parameter<double>("angle_max", 180.0);
    this->declare_parameter<double>("range_min", 0.1);
    this->declare_parameter<double>("range_max", 12.0);
    this->declare_parameter<bool>("invalid_range_is_inf", false);

    port_ = this->get_parameter("port").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    baudrate_ = this->get_parameter("baudrate").as_int();
    angle_min_deg_ = this->get_parameter("angle_min").as_double();
    angle_max_deg_ = this->get_parameter("angle_max").as_double();
    range_min_ = this->get_parameter("range_min").as_double();
    range_max_ = this->get_parameter("range_max").as_double();
    invalid_range_is_inf_ = this->get_parameter("invalid_range_is_inf").as_bool();

    // ---- 퍼블리셔 ----
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "scan", rclcpp::SensorDataQoS());

    // ---- 서비스 ----
    stop_srv_ = this->create_service<std_srvs::srv::Empty>(
        "stop_scan",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
          lidar_.stopScan();
          RCLCPP_INFO(get_logger(), "Scan stopped.");
        });

    start_srv_ = this->create_service<std_srvs::srv::Empty>(
        "start_scan",
        [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
               std::shared_ptr<std_srvs::srv::Empty::Response>)
        {
          if (!lidar_.isScanning())
          {
            lidar_.setScanCallback(
                [this](const LidarScan &s)
                { onScan(s); });
            lidar_.startScan();
            RCLCPP_INFO(get_logger(), "Scan started.");
          }
        });

    // 스캔 중 하드 에러(포트 소실) 발생 시 재연결 루프 트리거
    lidar_.setErrorCallback([this](const std::string &reason)
                            {
                              RCLCPP_ERROR(get_logger(),
                                           "Lidar disconnected: %s. Reconnecting every 3s.",
                                           reason.c_str());
                              scheduleReconnect();
                            });

    // ---- 라이다 연결 ----
    if (!connectAndStart())
    {
      RCLCPP_ERROR(get_logger(),
                   "Failed to open serial port %s. "
                   "Check USB connection and port permissions (dialout group). "
                   "Retrying every 3s.",
                   port_.c_str());
      scheduleReconnect();
    }
  }

  ~YdlidarNode()
  {
    lidar_.disconnect();
  }

private:
  /// (재)연결 + 장치 정보 조회 + 스캔 시작을 한 번에 수행. 성공 여부 반환.
  bool connectAndStart()
  {
    RCLCPP_INFO(get_logger(), "Connecting to YDLidar X4 Pro on %s @ %d",
                port_.c_str(), baudrate_);

    if (!lidar_.connect(port_, baudrate_))
    {
      return false;
    }

    if (lidar_.isDeviceInfoOk()) {
      const auto &di = lidar_.deviceInfo();
      RCLCPP_INFO(get_logger(),
                  "Connected: model=0x%02X, fw=%d.%d, hw=%d",
                  di.model,
                  (di.firmware_version >> 8) & 0xFF,
                  di.firmware_version & 0xFF,
                  di.hardware_version);
    } else {
      RCLCPP_WARN(get_logger(),
                  "Port %s opened but device info query timed out. "
                  "X4 Pro SingleChannel mode — continuing anyway.",
                  port_.c_str());
    }

    // 스캔 콜백 등록 후 시작
    lidar_.setScanCallback([this](const LidarScan &s)
                           { onScan(s); });
    if (!lidar_.startScan())
    {
      RCLCPP_ERROR(get_logger(), "Failed to start scan.");
      lidar_.disconnect();
      return false;
    }

    RCLCPP_INFO(get_logger(), "Scan started. Publishing on /scan");
    return true;
  }

  void scheduleReconnect()
  {
    if (reconnect_timer_)
    {
      return; // already retrying
    }
    reconnect_timer_ = create_wall_timer(
        std::chrono::seconds(3),
        [this]()
        {
          if (connectAndStart())
          {
            RCLCPP_INFO(get_logger(), "Lidar reconnected.");
            reconnect_timer_->cancel();
            reconnect_timer_.reset();
          }
        });
  }

  void onScan(const LidarScan &scan)
  {
    if (scan.points.empty())
      return;

    const float a_min_rad = static_cast<float>(angle_min_deg_ * DEG2RAD);
    const float a_max_rad = static_cast<float>(angle_max_deg_ * DEG2RAD);
    const float angle_increment = static_cast<float>(1.0 * DEG2RAD); // 1도 해상도
    const int size = static_cast<int>(
                         std::ceil((a_max_rad - a_min_rad) / angle_increment)) +
                     1;

    auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();

    // 타임스탬프
    uint64_t ns = scan.stamp_ns;
    msg->header.stamp.sec = static_cast<int32_t>(ns / 1'000'000'000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>(ns % 1'000'000'000ULL);
    msg->header.frame_id = frame_id_;

    msg->angle_min = a_min_rad;
    msg->angle_max = a_max_rad;
    msg->angle_increment = angle_increment;
    msg->range_min = static_cast<float>(range_min_);
    msg->range_max = static_cast<float>(range_max_);
    // scan_time / time_increment: 10 Hz 기준 추정
    msg->scan_time = 0.1f;
    msg->time_increment = 0.1f / static_cast<float>(size);

    const float fill_val = invalid_range_is_inf_
                               ? std::numeric_limits<float>::infinity()
                               : 0.0f;

    msg->ranges.assign(size, fill_val);
    msg->intensities.assign(size, 0.0f);

    for (const auto &pt : scan.points)
    {
      // 각도를 [-180, 180] 범위로 정규화
      float a = pt.angle_deg;
      while (a > 180.0f)
        a -= 360.0f;
      while (a < -180.0f)
        a += 360.0f;

      float a_rad = static_cast<float>(a * DEG2RAD);

      if (a_rad < a_min_rad || a_rad > a_max_rad)
        continue;
      if (pt.dist_m < range_min_ || pt.dist_m > range_max_)
        continue;

      int idx = static_cast<int>(
          std::round((a_rad - a_min_rad) / angle_increment));
      if (idx >= 0 && idx < size)
      {
        msg->ranges[idx] = pt.dist_m;
      }
    }

    scan_pub_->publish(std::move(msg));
  }

  // ---- 멤버 ----
  X4ProLidar lidar_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_srv_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  std::string port_;
  std::string frame_id_;
  int baudrate_;
  double angle_min_deg_;
  double angle_max_deg_;
  double range_min_;
  double range_max_;
  bool invalid_range_is_inf_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YdlidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
