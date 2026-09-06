#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <Eigen/Core>


class Go2HeightmapNode : public rclcpp::Node {

public:
  Go2HeightmapNode();

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishImage(const std::vector<float>& hm, const rclcpp::Time& stamp);
  void publishPoints3D(const std::vector<float>& hm, const rclcpp::Time& stamp);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  // 2D визуализация как "imshow"
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // опционально оставим 3D точки для отладки
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_pub_;

  // URDF joint: parent="base", child="radar"
  Eigen::Vector3f t_base_to_radar_;
  Eigen::Matrix3f R_base_to_radar_;

  // параметры heightmap (как в python, но сетка фикс 17x11)
  double min_height_;
  double max_height_;
  bool publish_points_3d_;
  bool flip_y_;  // как np.flip(axis=1) в python

  static constexpr int NX = 17;
  static constexpr int NY = 11;
  static constexpr float RES = 0.1f; // 10 см
};
//     Go2HeightmapNode();

// private:
//     void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr heightmap_pub_;

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     //Grid format
//       // Трансформ radar -> base (обратный к тому, что в URDF)
//     Eigen::Vector3f t_base_to_radar_;   // из URDF (base -> radar)
//     Eigen::Matrix3f R_base_to_radar_;   // из URDF (base -> radar)

//     static constexpr int NX =17;
//     static constexpr int NY =11;
//     static constexpr float RES =0.1f;
// };
