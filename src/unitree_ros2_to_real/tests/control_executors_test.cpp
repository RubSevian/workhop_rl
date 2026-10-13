#include "control_executors.hpp"
#include <atomic>
#include <cassert>
#include <chrono>

int main(int argc,char** argv) {
  using namespace std::chrono_literals;
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("control_executor_isolation_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto background=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    std::atomic<int> ticks{0},independent{0};
    auto fast=node->create_wall_timer(2ms,[&]{++ticks;},io);
    auto slow=node->create_wall_timer(20ms,[&]{
      auto before=ticks.load();std::this_thread::sleep_for(60ms);
      if(ticks.load()-before>=5)++independent;
    },policy);
    auto busy=node->create_wall_timer(20ms,[]{std::this_thread::sleep_for(60ms);},background);
    auto stop=node->create_wall_timer(400ms,[]{rclcpp::shutdown();});
    sim2real::SpinControlExecutors(node,io,policy);
    assert(ticks>=30 && independent>=2);
  }
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("control_executor_exception_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto timer=node->create_wall_timer(5ms,[]{throw std::runtime_error("probe worker error");},policy);
    bool propagated=false;
    try{sim2real::SpinControlExecutors(node,io,policy);}
    catch(const std::runtime_error& e){propagated=std::string(e.what())=="probe worker error";}
    assert(propagated && !rclcpp::ok());
  }
}
