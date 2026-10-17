#pragma once
#include <ATen/Parallel.h>
#include <omp.h>
#include <stdexcept>

namespace sim2real {
struct PolicyThreadSettings { int openmp_max, interop; };
// Supported ATen thread-initialization API. The constructor's setter records
// the process setting; each new executor thread must install its OpenMP ICV.
inline const PolicyThreadSettings& InitializePolicyThread() {
  thread_local const PolicyThreadSettings settings=[] {
    at::init_num_threads();
    // Do not call get_num_threads(): its lazy initialization masks this bug.
    PolicyThreadSettings result{omp_get_max_threads(),static_cast<int>(at::get_num_interop_threads())};
    if(result.openmp_max!=1 || result.interop!=1)
      throw std::runtime_error("Policy worker requires effective OpenMP/inter-op=1");
    return result;
  }();
  return settings;
}
}
