#include "real_controller_core.hpp"
#include <ATen/Parallel.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <stdexcept>

template<class Call> void Measure(const char* name,Call call) {
  for(int i=0;i<50;++i) call();
  std::vector<double> ms;
  for(int i=0;i<500;++i) {
    const auto start=std::chrono::steady_clock::now(); call();
    ms.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
  }
  const double mean=std::accumulate(ms.begin(),ms.end(),0.0)/ms.size();
  std::sort(ms.begin(),ms.end());
  std::cout<<name<<" samples=500 mean_ms="<<mean<<" p50_ms="<<ms[250]<<" p95_ms="<<ms[475]<<" p99_ms="<<ms[495]<<" max_ms="<<ms.back()<<" period_ms=20\n";
  if(ms.back()>=20) throw std::runtime_error("CPU measurement exceeded 20 ms; investigate deadline before R2");
}
int main(int argc,char** argv) {
  try {
    if(argc!=3) throw std::runtime_error("explicit config and model paths required");
    at::set_num_threads(1); at::set_num_interop_threads(1);
    torch::InferenceMode inference;
    auto module=torch::jit::load(argv[2],torch::kCPU); module.eval();
    const auto input=torch::zeros({1,315});
    auto output=module.forward({input}).toTensor();
    if(output.sizes()!=torch::IntArrayRef({1,12}) || !torch::isfinite(output).all().item<bool>()) throw std::runtime_error("Invalid actor output");
    auto test=torch::linspace(-0.2,0.2,315).view({1,315});
    if(!torch::isfinite(module.forward({test}).toTensor()).all().item<bool>()) throw std::runtime_error("Nonfinite test output");
    sim2real::RealControllerCore core; core.Load(argv[1],argv[2]);
    auto& a=core.agent(); a.obs.dof_pos=a.params.default_dof_pos.clone();
    a.ResetPolicyState();
    if(a.Act().numel()!=12) throw std::runtime_error("Agent output dimension");
    Measure("jit_cpu_315_to_12",[&] { module.forward({input}); });
    Measure("agent_cpu_frame_history_action",[&] { a.Act(); });
    std::cout<<"policy_cpu_test: PASS C++ load CPU [1,315] -> [1,12]\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
