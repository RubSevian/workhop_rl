#pragma once
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <unistd.h>

namespace sim2real {
// This Torch build's Arm Compute Library caches its own OpenMP team before
// main(). ATen thread initialization cannot resize that cached scheduler.
// Re-exec before ROS/node/SDK construction so all loaded libraries start with
// OMP1. This affects only this process, preserves argv/PID, and runs at most once.
inline void InitializePolicyProcess(char** argv) {
  const char* value=std::getenv("OMP_NUM_THREADS");
  if(value&&std::strcmp(value,"1")==0)return;
  if(setenv("OMP_NUM_THREADS","1",1)!=0)
    throw std::runtime_error("Cannot establish process-local OMP_NUM_THREADS=1");
  execv("/proc/self/exe",argv);
  throw std::runtime_error("Cannot re-exec policy process with OMP1");
}
}
