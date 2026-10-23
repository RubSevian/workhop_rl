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
    std::atomic<int> initialized{0};std::thread::id policy_thread;
    auto fast=node->create_wall_timer(2ms,[&]{++ticks;},io);
    auto slow=node->create_wall_timer(20ms,[&]{
      assert(initialized==1 && policy_thread==std::this_thread::get_id());
      auto before=ticks.load();std::this_thread::sleep_for(60ms);
      if(ticks.load()-before>=5)++independent;
    },policy);
    auto busy=node->create_wall_timer(20ms,[]{std::this_thread::sleep_for(60ms);},background);
    auto stop=node->create_wall_timer(400ms,[]{rclcpp::shutdown();});
    sim2real::SpinControlExecutors(node,io,policy,false,[&]{policy_thread=std::this_thread::get_id();++initialized;});
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
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("direct_policy_isolation_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    std::atomic<int> ios{0};int jobs=0;int64_t previous_expected=0,previous_end=0;
    std::thread::id worker;
    auto timer=node->create_wall_timer(2ms,[&]{++ios;},io);
    sim2real::SpinControlExecutors(node,io,policy,false,[&]{worker=std::this_thread::get_id();},
      [&](int64_t expected,int64_t actual){
        assert(worker==std::this_thread::get_id()&&actual>=expected);
        if(previous_expected){assert(expected>previous_expected&&expected>previous_end);assert((expected-previous_expected)%20000000==0);}
        if(jobs==0)std::this_thread::sleep_for(55ms); // missed releases must be skipped
        previous_expected=expected;previous_end=sim2real::PolicyMonotonicNs();
        if(++jobs==6)rclcpp::shutdown();
      },20000000);
    assert(jobs==6&&ios>=25);
  }
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("direct_policy_exception_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    bool propagated=false;
    try{sim2real::SpinControlExecutors(node,io,policy,false,{},[](int64_t,int64_t){throw std::runtime_error("direct failed");},1000000);}
    catch(const std::runtime_error& e){propagated=std::string(e.what())=="direct failed";}
    assert(propagated&&!rclcpp::ok());
  }
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("policy_initializer_failure_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    bool propagated=false;
    try{sim2real::SpinControlExecutors(node,io,policy,false,[]{throw std::runtime_error("init failed");});}
    catch(const std::runtime_error& e){propagated=std::string(e.what())=="init failed";}
    assert(propagated && !rclcpp::ok());
  }
  // Reproduce shared-worker exhaustion, then verify isolated observations.
  // No robot topics or publishers: timers model runnable input callbacks.
  for(bool isolated : {false,true}) {
    rclcpp::init(argc,argv);
    auto node=std::make_shared<rclcpp::Node>("observation_worker_isolation_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto observations=isolated?node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false):nullptr;
    auto arm=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto status=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    std::atomic<int> occupied{0},received{0},outputs{0};std::atomic<bool> release{false};
    auto block=[&]{++occupied;while(!release&&rclcpp::ok())std::this_thread::sleep_for(1ms);};
    auto a=node->create_wall_timer(50ms,block,arm);
    auto b=node->create_wall_timer(50ms,block,status);
    auto input=node->create_wall_timer(2ms,[&]{++received;},observations);
    auto output=node->create_wall_timer(2ms,[&]{++outputs;},io);
    bool verified=false;
    std::jthread observer([&]{
      auto deadline=std::chrono::steady_clock::now()+3s;
      while(occupied<2&&std::chrono::steady_clock::now()<deadline)std::this_thread::sleep_for(1ms);
      if(occupied<2){release=true;rclcpp::shutdown();return;}
      int before=received.load(),out_before=outputs.load();
      std::this_thread::sleep_for(100ms);
      int delta=received-before;
      verified=(isolated?delta>=20:delta==0)&&outputs-out_before>=20;
      release=true;rclcpp::shutdown();
    });
    sim2real::SpinControlExecutors(node,io,policy,false,{},{},20000000,observations);
    observer.join();assert(verified);
  }
  rclcpp::init(argc,argv);
  {
    auto node=std::make_shared<rclcpp::Node>("observation_exception_test");
    auto io=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto policy=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto observations=node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive,false);
    auto timer=node->create_wall_timer(5ms,[]{throw std::runtime_error("observation failed");},observations);
    bool propagated=false;
    try{sim2real::SpinControlExecutors(node,io,policy,false,{},{},20000000,observations);}
    catch(const std::runtime_error& e){propagated=std::string(e.what())=="observation failed";}
    assert(propagated&&!rclcpp::ok());
  }
}
