#include "real_controller_core.hpp"
#include <ATen/Parallel.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <stdexcept>
static void require(bool v,const char* why){if(!v)throw std::runtime_error(why);}
static void feedback(sim2real::RealControllerCore& c,int n){auto& a=c.agent();
 a.obs.dof_pos=a.params.default_dof_pos.clone();a.obs.command=torch::tensor({float((n%3-1)*.5),float((n%5-2)*.25),float((n%3-1)*.5)});
}
int main(int argc,char** argv){try{
 require(argc==3,"config/model required");at::set_num_threads(1);at::set_num_interop_threads(1);
 sim2real::RealControllerCore before,after;before.Load(argv[1],argv[2]);after.Load(argv[1],argv[2]);
 feedback(before,0);before.agent().ResetPolicyState();
 {torch::InferenceMode guard;feedback(after,0);after.agent().ResetPolicyState();}
 for(int n=0;n<30;++n){feedback(before,n);auto expected=before.agent().Act();
  require(expected.requires_grad(),"reproduce unguarded autograd output");
  if(n>0)require(before.agent().UnifiedActorHistory().requires_grad(),"previous action retained in history graph");
  torch::InferenceMode guard;feedback(after,n);auto actual=after.agent().Act();
  require(!actual.requires_grad()&&!after.agent().obs.action.requires_grad()&&!after.agent().UnifiedActorHistory().requires_grad(),"no inference graphs");
  require(torch::allclose(expected,actual,1e-5,1e-6),"guard changes numerical outputs/history");
 }
 std::exception_ptr error;double worst_first=0;
 // Constructor guard is thread-local: execute reset/Act on a separate worker as ROS does.
 std::thread worker([&]{try{require(torch::autograd::GradMode::is_enabled(),"worker default grad enabled");
  for(int session=0;session<5;++session){std::this_thread::sleep_for(std::chrono::milliseconds(20));
   for(int n=0;n<200;++n){torch::InferenceMode guard;feedback(after,n);
    if(n==0)after.agent().ResetPolicyState();auto start=std::chrono::steady_clock::now();auto output=after.agent().Act();
    if(n==0)worst_first=std::max(worst_first,std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
    require(output.numel()==12&&torch::isfinite(output).all().item<bool>(),"finite output");
    require(!output.requires_grad()&&!after.agent().UnifiedActorHistory().requires_grad(),"restart retained autograd graph");
   }
  }
 }catch(...){error=std::current_exception();}});worker.join();if(error)std::rethrow_exception(error);
 std::cout<<"PASS real actor: unguarded output/history retain autograd; guarded numerical parity30 frames; worker restarts5x200 frames no graphs; max_first_act_ms="<<worst_first<<" (synthetic, not physical deadline certification)\n";
 return 0;
 }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
