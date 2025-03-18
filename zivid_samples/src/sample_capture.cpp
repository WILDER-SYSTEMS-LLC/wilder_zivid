#include <exception>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

/*
 * This sample shows how to set the settings_file_path parameter of the zivid node, subscribe for
 * the points/xyzrgba topic, and invoke the capture service. When a point cloud is received, a new
 * capture is triggered.
 */

void set_settings(const std::shared_ptr<rclcpp::Node> & node)
{
  RCLCPP_INFO(node->get_logger(), "Setting parameter 'settings_yaml'");
  const std::string settings_yml =
    R"(
__version__: 27
Settings:
  Acquisitions:
    - Acquisition:
        Aperture: 2.59
        Brightness: 2.5
        ExposureTime: 2000
        Gain: 2
  Color:
    __version__: 7
    Settings2D:
      Acquisitions:
        - Acquisition:
            Aperture: 2.59
            Brightness: 2.5
            ExposureTime: 1000
            Gain: 2
      Processing:
        Color:
          Balance:
            Blue: 1
            Green: 1
            Red: 1
          Experimental:
            Mode: automatic
          Gamma: 1
      Sampling:
        Color: rgb
        Pixel: by2x2
  Diagnostics:
    Enabled: no
  Engine: omni
  Processing:
    Color:
      Balance:
        Blue: __not_set__
        Green: __not_set__
        Red: __not_set__
      Experimental:
        Mode: __not_set__
      Gamma: __not_set__
    Filters:
      Cluster:
        Removal:
          Enabled: yes
          MaxNeighborDistance: 6
          MinArea: 500
      Experimental:
        ContrastDistortion:
          Correction:
            Enabled: no
            Strength: 0
          Removal:
            Enabled: no
            Threshold: 0.4
      Hole:
        Repair:
          Enabled: yes
          HoleSize: 0.7
          Strictness: 1
      Noise:
        Removal:
          Enabled: yes
          Threshold: 2
        Repair:
          Enabled: yes
        Suppression:
          Enabled: yes
      Outlier:
        Removal:
          Enabled: yes
          Threshold: 10
      Reflection:
        Removal:
          Enabled: yes
          Mode: global
      Smoothing:
        Gaussian:
          Enabled: yes
          Sigma: 1.5
    Resampling:
      Mode: disabled
  RegionOfInterest:
    Box:
      Enabled: no
      Extents: [-10, 100]
      PointA: [0, 0, 0]
      PointB: [0, 0, 0]
      PointO: [0, 0, 0]
    Depth:
      Enabled: no
      Range: [300, 1100]
  Sampling:
    Color: __not_set__
    Pixel: by2x2
)";

  auto param_client = std::make_shared<rclcpp::AsyncParametersClient>(node, "zivid_camera");
  while (!param_client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the parameters client to appear...");
  }

  auto result = param_client->set_parameters({rclcpp::Parameter("settings_yaml", settings_yml)});
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set `settings_yaml` parameter");
    std::terminate();
  }
}

auto create_capture_client(std::shared_ptr<rclcpp::Node> & node)
{
  auto client = node->create_client<std_srvs::srv::Trigger>("capture");
  while (!client->wait_for_service(std::chrono::seconds(3))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Client interrupted while waiting for service to appear.");
      std::terminate();
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for the capture service to appear...");
  }

  RCLCPP_INFO(node->get_logger(), "Capture service is available");
  return client;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("sample_capture");
  RCLCPP_INFO(node->get_logger(), "Started the sample_capture node");

  set_settings(node);

  auto capture_client = create_capture_client(node);
  auto trigger_capture = [&]() {
    RCLCPP_DEBUG_STREAM(node->get_logger(), "Triggering capture");
    capture_client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  };

  auto points_xyzrgba_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points/xyzrgba", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_DEBUG_STREAM(
        node->get_logger(), "Received point cloud of size %d x %d" + msg->width + msg->height);
      trigger_capture();
    });

  auto color_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "color/image_color", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_DEBUG_STREAM(
        node->get_logger(), "Received image of size %d x %d" + msg->width + msg->height);
      trigger_capture();
    });

  auto depth_image_color_subscription = node->create_subscription<sensor_msgs::msg::Image>(
    "depth/image", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr msg) -> void {
      RCLCPP_DEBUG_STREAM(
        node->get_logger(), "Received image of size %d x %d" + msg->width + msg->height);
      trigger_capture();
    });

  auto normals_subscription = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "normals/xyz", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) -> void {
      RCLCPP_DEBUG_STREAM(
        node->get_logger(), "Received normals of size %d x %d" + msg->width + msg->height);
      trigger_capture();
    });

  trigger_capture();

  RCLCPP_INFO(node->get_logger(), "Spinning node.. Press Ctrl+C to abort.");
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
