#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <fstream>
#include <mutex>
#include <thread>
namespace sim2real {
struct MotionTraceSample {
 int kind=0,phase=0,accepted=-1;long long stamp_ns=0;unsigned long long packets=0;
 double interval_s=0,io_min_s=0,io_max_s=0,low_age_s=0,remote_age_s=0,arm_age_s=0,target_age_s=0;
 double compute_ms=0,job_age_s=0,result_age_s=0;
 std::array<double,3> sticks{},requested{},command{},gyro{};
 std::array<float,4> quat{};
 std::array<float,12> q{},dq{},target{},action{},kp{},kd{};
};
// Fixed storage; producers never wait for disk or queue ownership.
class MotionTrace {
 public:
 MotionTrace(const std::string& path,double duration_s);
 ~MotionTrace();
 bool enabled() const {return enabled_.load(std::memory_order_relaxed);}
 bool Submit(const MotionTraceSample& sample);
 void Stop(){enabled_.store(false,std::memory_order_relaxed);}
 size_t dropped() const {return dropped_.load();}
 private:
 void Write();
 std::ofstream file_;std::thread writer_;std::mutex queue_mutex_;
 std::array<MotionTraceSample,256> queue_{};size_t head_=0,size_=0;
 std::atomic<bool> enabled_{true};std::atomic<size_t> dropped_{0};
 std::chrono::steady_clock::time_point first_{};bool started_=false;double duration_s_;
};
}
