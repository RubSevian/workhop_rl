#include "go2_heightmap_node.hpp"

using std::placeholders::_1;

Go2HeightmapNode::Go2HeightmapNode()
: Node("go2_heightmap_node")
{
  // параметры визуализации/нормализации
  this->declare_parameter("min_height", -1.0);
  this->declare_parameter("max_height",  1.0);
  this->declare_parameter("publish_points_3d", true);
  this->declare_parameter("flip_y", true);

  min_height_ = this->get_parameter("min_height").as_double();
  max_height_ = this->get_parameter("max_height").as_double();
  publish_points_3d_ = this->get_parameter("publish_points_3d").as_bool();
  flip_y_ = this->get_parameter("flip_y").as_bool();

  // подписка на облако
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud",
    rclcpp::SensorDataQoS(),
    std::bind(&Go2HeightmapNode::cloudCallback, this, _1));

  // 2D картинка (RViz Image)
  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/height_map/image", 10);

  // 3D debug точки (как было у тебя)
  heightmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/go2/heightmap_17x11", 10);

  // ===== URDF: base -> radar =====
  // origin xyz="0.28945 0 -0.046825" rpy="0 2.8782 0"
  t_base_to_radar_ = Eigen::Vector3f(0.28945f, 0.0f, -0.046825f);

  float pitch = 2.8782f;
  float c = std::cos(pitch);
  float s = std::sin(pitch);
  R_base_to_radar_ <<
      c,   0.0f,  s,
    0.0f, 1.0f, 0.0f,
     -s,  0.0f,  c;

  RCLCPP_INFO(this->get_logger(),
              "Go2HeightmapNode started. input=rt/utlidar/cloud, image=/height_map/image, points=/go2/heightmap_17x11");
}

void Go2HeightmapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // heightmap как у python: стартуем min_height, потом max(z)
  std::vector<float> hm(NX * NY, static_cast<float>(min_height_));

  const float size_x = NX * RES;
  const float size_y = NY * RES;
  const float x_min = -size_x / 2.0f;
  const float y_min = -size_y / 2.0f;

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    float xr = *it_x;
    float yr = *it_y;
    float zr = *it_z;

    if (!std::isfinite(xr) || !std::isfinite(yr) || !std::isfinite(zr)) continue;

    Eigen::Vector3f p_radar(xr, yr, zr);

    // radar -> base (инверсия URDF base->radar):
    // p_base = R^T * (p_radar - t)
    Eigen::Vector3f p_base = R_base_to_radar_.transpose() * (p_radar - t_base_to_radar_);

    float xb = p_base.x();
    float yb = p_base.y();
    float zb = p_base.z();

    int ix = static_cast<int>((xb - x_min) / RES);
    int iy = static_cast<int>((yb - y_min) / RES);

    if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) continue;

    int idx = ix * NY + iy;
    hm[idx] = std::max(hm[idx], zb);
  }

  const auto stamp = this->now();
  publishImage(hm, stamp);

  if (publish_points_3d_) {
    publishPoints3D(hm, stamp);
  }
}

void Go2HeightmapNode::publishImage(const std::vector<float>& hm, const rclcpp::Time& stamp)
{
  // MONO8 для RViz Image [web:242]
  sensor_msgs::msg::Image img;
  img.header.stamp = stamp;
  img.header.frame_id = "base";   // по URDF

  img.height = NX;                // rows
  img.width  = NY;                // cols
  img.encoding = "mono8";
  img.is_bigendian = false;
  img.step = NY;                  // 1 byte per pixel
  img.data.resize(static_cast<size_t>(NX * NY));

  const float hmin = static_cast<float>(min_height_);
  const float hmax = static_cast<float>(max_height_);
  const float denom = (hmax > hmin) ? (hmax - hmin) : 1.0f;

  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NY; ++j) {
      int idx = i * NY + j;
      float h = hm[idx];

      uint8_t pix = 0;
      if (h > hmin + 1e-6f) {
        if (h < hmin) h = hmin;
        if (h > hmax) h = hmax;
        float norm = (h - hmin) / denom;           // 0..1
        norm = std::min(1.0f, std::max(0.0f, norm));
        pix = static_cast<uint8_t>(std::round(norm * 255.0f));
      }

      int jj = flip_y_ ? (NY - 1 - j) : j; // как np.flip(axis=1)
      img.data[i * NY + jj] = pix;
    }
  }

  image_pub_->publish(img);
}

void Go2HeightmapNode::publishPoints3D(const std::vector<float>& hm, const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 out;
  out.header.stamp = stamp;
  out.header.frame_id = "base";
  out.height = 1;
  out.width  = NX * NY;
  out.is_dense = false;

  sensor_msgs::PointCloud2Modifier mod(out);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(NX * NY);

  sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
  sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
  sensor_msgs::PointCloud2Iterator<float> oz(out, "z");

  const float size_x = NX * RES;
  const float size_y = NY * RES;
  const float x_min = -size_x / 2.0f;
  const float y_min = -size_y / 2.0f;

  for (int i = 0; i < NX; ++i) {
    for (int j = 0; j < NY; ++j, ++ox, ++oy, ++oz) {
      int idx = i * NY + j;
      float cx = x_min + (i + 0.5f) * RES;
      float cy = y_min + (j + 0.5f) * RES;
      float cz = hm[idx];

      *ox = cx;
      *oy = cy;
      *oz = (cz <= static_cast<float>(min_height_) + 1e-6f) ? -0.5f : cz;
    }
  }

  heightmap_pub_->publish(out);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Go2HeightmapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}















































// using std::placeholders::_1;

// Go2HeightmapNode::Go2HeightmapNode()
// : Node("go2_heightmap_node"),
//   tf_buffer_(this->get_clock()),
//   tf_listener_(tf_buffer_)
// {
//   // Подписка на deskewed облако лидара (в odom)
//   cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//     "/utlidar/cloud",
//     rclcpp::SensorDataQoS(),
//     std::bind(&Go2HeightmapNode::cloudCallback, this, _1));

//   // Публикация отладочной карты высот
//   heightmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//     "/go2/heightmap_17x11",
//     10);

//     //Transform 

//     t_base_to_radar_ = Eigen::Vector3f(0.28945f, 0.0f, -0.046825f);

//   // R_base_to_radar = Ry(pitch), pitch = 2.8782 рад
//   float pitch = 2.8782f;
//   float c = std::cos(pitch);
//   float s = std::sin(pitch);
//   R_base_to_radar_ <<
//       c,   0.0f,  s,
//     0.0f, 1.0f, 0.0f,
//      -s,  0.0f,  c;

//   RCLCPP_INFO(this->get_logger(),
//               "Go2HeightmapNode started with URDF transform base->radar (xyz: %.5f, %.5f, %.5f, pitch=%.4f)",
//               t_base_to_radar_.x(), t_base_to_radar_.y(), t_base_to_radar_.z(), pitch);


//   RCLCPP_INFO(this->get_logger(), "Go2HeightmapNode started");
// }

// void Go2HeightmapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
// {
//   //  // 1) НЕ ДЕЛАЕМ transform, работаем сразу в odom
//   // const sensor_msgs::msg::PointCloud2 & cloud_odom = *msg;

//     const auto & cloud = *msg;

//   // 1) Читаем точки в фрейме radar (как они приходят из rt/utlidar/cloud)
//   std::vector<Eigen::Vector3f> points_base;
//   points_base.reserve(cloud.width * cloud.height);

//   // // 2) Собираем точки (x,y,z) в odom
//   // std::vector<Eigen::Vector3f> points;
//   // points.reserve(cloud_odom.width * cloud_odom.height);

//   sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
//   sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
//   sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

//   for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
//     float x = *iter_x;
//     float y = *iter_y;
//     float z = *iter_z;
//     if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
//     points_base.emplace_back(x, y, z);
//   // }

//   // if (points.empty()) {
//   //   RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//   //                        "No valid lidar points in cloud");
//   //   return;
//   // }
//     Eigen::Vector3f p_radar(x, y, z);

//     // 2) Переводим в координаты base.
//     // URDF даёт base -> radar:  p_radar = R_base_to_radar * p_base + t_base_to_radar.
//     // Нужно обратное: p_base = R^T * (p_radar - t).
//     Eigen::Vector3f p_base = R_base_to_radar_.transpose() * (p_radar - t_base_to_radar_);
//     points_base.push_back(p_base);
//   }

//   if (points_base.empty()) {
//     RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
//                          "No valid points in rt/utlidar/cloud after transform radar->base");
//     return;
//   }

//   // 3) Сетку 17x11 пока строим ВОКРУГ (0,0) в odom (то есть «где‑то около старта робота»)
//   const float size_x = NX * RES; // 1.7 м
//   const float size_y = NY * RES; // 1.1 м
//   const float x_min = -size_x / 2.0f;
//   const float y_min = -size_y / 2.0f;

//   std::vector<float> heightmap(NX * NY, std::numeric_limits<float>::quiet_NaN());

//   for (const auto &p : points_base) {
//     float x = p.x();
//     float y = p.y();
//     float z = p.z();

//     int ix = static_cast<int>((x - x_min) / RES);
//     int iy = static_cast<int>((y - y_min) / RES);

//     if (ix < 0 || ix >= NX || iy < 0 || iy >= NY) continue;

//     int idx = ix * NY + iy;
//     float &cell_z = heightmap[idx];

//     if (std::isnan(cell_z) || z > cell_z) {
//       cell_z = z;
//     }
//   }

//   // 4) Формируем PointCloud2 в odom
//   sensor_msgs::msg::PointCloud2 out_cloud;
//   out_cloud.header.stamp = this->now();
//   out_cloud.header.frame_id = "base";   // тот же линк, что в URDF

//   out_cloud.height = 1;
//   out_cloud.width = NX * NY;
//   out_cloud.is_dense = false;

//   sensor_msgs::PointCloud2Modifier modifier(out_cloud);
//   modifier.setPointCloud2FieldsByString(1, "xyz");
//   modifier.resize(NX * NY);

//   sensor_msgs::PointCloud2Iterator<float> o_x(out_cloud, "x");
//   sensor_msgs::PointCloud2Iterator<float> o_y(out_cloud, "y");
//   sensor_msgs::PointCloud2Iterator<float> o_z(out_cloud, "z");

//   for (int ix = 0; ix < NX; ++ix) {
//     for (int iy = 0; iy < NY; ++iy, ++o_x, ++o_y, ++o_z) {
//       int idx = ix * NY + iy;
//       float z = heightmap[idx];

//       float cx = x_min + (ix + 0.5f) * RES;
//       float cy = y_min + (iy + 0.5f) * RES;

//       *o_x = cx;
//       *o_y = cy;
//       *o_z = std::isnan(z) ? -0.5f : z;
//     }
//   }

//   heightmap_pub_->publish(out_cloud);
// }

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<Go2HeightmapNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }