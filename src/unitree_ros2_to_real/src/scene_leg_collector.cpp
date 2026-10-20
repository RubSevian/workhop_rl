// Dedicated read-only observer: no actuator types, clients, SDK or motor output.
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <go2_scene_interfaces/msg/scene_leg_batch.hpp>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

using Batch = go2_scene_interfaces::msg::SceneLegBatch;
using Sample = go2_scene_interfaces::msg::SceneLegSample;
static uint64_t MonoNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}
class SceneLegCollector : public rclcpp::Node {
 public:
  SceneLegCollector() : Node("scene_leg_collector"), session_(std::to_string(MonoNs())) {
    const auto period = declare_parameter<int>("publish_period_ms",20);
    const auto capacity = declare_parameter<int>("pending_capacity",512);
    if(period<1||period>100||capacity<1||capacity>1000)
      throw std::invalid_argument("Invalid bounded scene collector settings");
    capacity_=static_cast<size_t>(capacity);pending_.reserve(capacity_);
    publisher_=create_publisher<Batch>("/go2/scene/leg_samples",rclcpp::QoS(32));
    subscription_=create_subscription<unitree_go::msg::LowState>(
      "/lowstate",rclcpp::SensorDataQoS().keep_last(1),[this](unitree_go::msg::LowState::ConstSharedPtr msg) {
        const auto stamp=MonoNs();
        // Same single-source contract as the Python consumers; never fuse robots.
        if(count_publishers("/lowstate")!=1)return;
        if(have_tick_ && (uint32_t(msg->tick-last_tick_)==0 || uint32_t(msg->tick-last_tick_)>=0x80000000u))return;
        Sample sample;sample.observed_ns=stamp;sample.robot_tick=msg->tick;
        for(size_t i=0;i<sample.q.size();++i) {
          sample.q[i]=msg->motor_state[i].q;
          if(!std::isfinite(sample.q[i]))return;
        }
        have_tick_=true;last_tick_=msg->tick;++received_;
        if(pending_.size()==capacity_) { pending_.erase(pending_.begin());++dropped_; }
        pending_.push_back(sample);
      });
    timer_=create_wall_timer(std::chrono::milliseconds(period),[this] {
      if(pending_.empty())return;
      Batch batch;batch.observed_ns=MonoNs();batch.source_session_id=session_;
      batch.samples_received=received_;batch.samples_dropped=dropped_;
      batch.samples.assign(pending_.begin(),pending_.end());pending_.clear();
      publisher_->publish(batch);
    });
    RCLCPP_INFO(get_logger(),"Read-only native leg observer: no actuator output; batch period %d ms",period);
  }
 private:
  std::string session_;std::vector<Sample> pending_;size_t capacity_=512;
  bool have_tick_=false;uint32_t last_tick_=0;uint64_t received_=0,dropped_=0;
  rclcpp::Publisher<Batch>::SharedPtr publisher_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<SceneLegCollector>()); }
  catch(const std::exception& error) {
    if(rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("scene_leg_collector"),"%s",error.what());
      rclcpp::shutdown();return 1;
    }
    // SIGINT can invalidate the context during an in-flight graph query.
  }
  if(rclcpp::ok())rclcpp::shutdown();return 0;
}
