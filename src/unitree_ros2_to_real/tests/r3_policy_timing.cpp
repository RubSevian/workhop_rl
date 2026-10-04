#include "real_controller_core.hpp"
#include <ATen/Parallel.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <filesystem>
#include <fstream>
#include <thread>
#include <cmath>
#include <stdexcept>
void Sensors(const char* stage) {
 namespace fs=std::filesystem;std::cout<<"sensors="<<stage<<'\n';
 for(const auto& e:fs::directory_iterator("/sys/devices/system/cpu")) {
  const auto p=e.path()/"cpufreq/scaling_cur_freq";if(fs::exists(p)){std::ifstream f(p);std::string value;f>>value;std::cout<<p.string()<<'='<<value<<'\n';}
 }
 for(const auto& e:fs::directory_iterator("/sys/class/thermal")) {
  const auto temp=e.path()/"temp",type=e.path()/"type";
  if(fs::exists(temp)&&fs::exists(type)){std::ifstream t(temp),n(type);std::string v,k;t>>v;n>>k;std::cout<<"temperature_mC["<<k<<"]="<<v<<'\n';}
 }
}
int main(int argc,char** argv) {
 try {
  if(argc!=4)throw std::invalid_argument("config model samples required");
  size_t samples=std::stoul(argv[3]);if(samples<1000||samples>100000)throw std::invalid_argument("samples must be 1000..100000");
  at::set_num_threads(1);at::set_num_interop_threads(1);torch::InferenceMode inference;
  sim2real::RealControllerCore core;core.Load(argv[1],argv[2]);auto& a=core.agent();
  a.obs.dof_pos=a.params.default_dof_pos.clone();a.ResetPolicyState();
  for(int i=0;i<100;++i)a.Act();Sensors("before");
  std::vector<double> latency,arrival;size_t misses=0,loop_misses=0;
  const auto period=std::chrono::milliseconds(20);auto scheduled=std::chrono::steady_clock::now();
  for(size_t i=0;i<samples;++i) {
   std::this_thread::sleep_until(scheduled);const auto started=std::chrono::steady_clock::now();
   const auto action=a.Act();if(!torch::isfinite(action).all().item<bool>())throw std::runtime_error("Invalid inference");
   const auto ended=std::chrono::steady_clock::now();
   const double ms=std::chrono::duration<double,std::milli>(ended-started).count();
   const double end_ms=std::chrono::duration<double,std::milli>(ended-scheduled).count();
   latency.push_back(ms);arrival.push_back(end_ms);misses+=ms>=20;loop_misses+=end_ms>=20;
   scheduled+=period;
   // Never burst through missed schedules; report them, resynchronize next period.
   if(ended>scheduled)scheduled=ended+period;
   if((i+1)%1000==0)Sensors("interval");
  }
  auto stats=[&](const char* name,std::vector<double> v,size_t failed){
   const double mean=std::accumulate(v.begin(),v.end(),0.)/v.size();std::sort(v.begin(),v.end());
   auto p=[&](double x){return v[std::min(v.size()-1,size_t(std::ceil(x*v.size()))-1)];};
   std::cout<<name<<" samples="<<v.size()<<" mean_ms="<<mean<<" p95_ms="<<p(.95)<<" p99_ms="<<p(.99)
    <<" p99.9_ms="<<p(.999)<<" max_ms="<<v.back()<<" deadline_misses="<<failed<<'/'<<v.size()<<" period_ms=20\n";
  };
  stats("agent_compute",latency,misses);stats("scheduled_loop_end",arrival,loop_misses);Sensors("after");
  std::cout<<"CPU only; no ROS/serial/LowCmd; deadline review required before RL_ACTIVE\n";
  return 0; // Diagnostic records misses; strict R2 threshold is unchanged.
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
