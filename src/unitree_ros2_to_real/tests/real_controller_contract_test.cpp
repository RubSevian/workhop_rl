#include "real_controller_core.hpp"
#include <iostream>
#include <stdexcept>

using namespace sim2real;
void Check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
int main(int argc,char** argv) {
  try {
    Check(argc==3,"config and policy required");
    Legs p{}; for(int i=0;i<12;++i) p[i]=i+1;
    const Legs expected{4,5,6,1,2,3,10,11,12,7,8,9};
    Check(PolicyToMotor(p)==expected,"policy to motor mapping");
    Check(MotorToPolicy(expected)==p,"motor to policy mapping");
    Readiness ready{true,true,true,true};
    OutputGate gate;
    Check(!gate.Allows(ready),"startup gate must be disarmed");
    for(int i=0;i<4;++i) {
      auto r=ready;
      if(i==0) r.model_loaded=false;
      if(i==1) r.low_state_fresh=false;
      if(i==2) r.arm_state_fresh=false;
      if(i==3) r.ownership_verified=false;
      Check(!gate.Arm(r),"incomplete readiness must refuse arming");
      Check(gate.Arm(ready) && !gate.Allows(r),"freshness must gate an armed controller");
      gate.Disarm();
    }
    Check(gate.Arm(ready) && gate.Allows(ready),"explicit offline gate transition");
    gate.Fault(); Check(!gate.Allows(ready) && !gate.Arm(ready),"fault must latch");
    RealControllerCore c;
    Check(c.mode()==Mode::DISARMED && !c.Tick(0),"startup returns no targets");
    Check(!c.RequestMode(Mode::RL,ready),"unloaded controller must refuse RL");
    c.Load(argv[1],argv[2]);
    Check(c.mode()==Mode::DISARMED && !c.Tick(0),"loading must never arm or stand");
    Check(!c.RequestMode(Mode::RL,ready),"RL requires HOLD first");
    auto& a=c.agent();
    // Synthetic fixtures, never obtained through hardware transport.
    a.obs.arm_pos=torch::arange(6,torch::kFloat32)*0.1;
    a.obs.arm_vel=torch::arange(6,torch::kFloat32)*0.2;
    a.obs.arm_target=torch::arange(6,torch::kFloat32)*0.3;
    Legs measured{}; measured.fill(0.2F);
    c.SetMeasuredLegs(measured,{});
    a.params.fixed_kp=torch::arange(12,torch::kFloat32)+40;
    a.params.fixed_kd=torch::arange(12,torch::kFloat32)+1;
    a.params.torque_limits=torch::arange(12,torch::kFloat32)+20;
    Check(c.RequestMode(Mode::STAND,ready),"explicit offline stand");
    auto start=c.Tick(0).value(); Check(start.q==measured,"stand must start at measured motor pose");
    auto mid=c.Tick(4).value(); auto end=c.Tick(8).value();
    Check(c.mode()==Mode::HOLD,"stand must enter hold continuously");
    Check(end.q[0]==-0.1F && end.q[3]==0.1F && end.q[6]==-0.1F && end.q[9]==0.1F,"asymmetric hip signs in stand");
    for(int i=0;i<12;++i) {
      Check(std::abs(mid.q[i]-(measured[i]+end.q[i])*0.5F)<1e-6,"stand interpolation");
      Check(end.kp[i]==40+motor_to_policy[i],"stand gain mapping");
      Check(end.kd[i]==1+motor_to_policy[i],"stand damping mapping");
      Check(end.torque_limits[i]==20+motor_to_policy[i],"limit mapping");
    }
    Check(c.Tick(0)->q==end.q,"hold target mapping");
    for(int entry=0;entry<2;++entry) {
      a.obs.action=torch::full({12},7.0F);
      a.obs.arm_pos=torch::full({6},0.1F+entry);
      Check(c.RequestMode(Mode::RL,ready),"RL entry from HOLD");
      auto history=a.UnifiedActorHistory().view({5,63});
      Check(history.numel()==315,"actor history dimension");
      for(int i=0;i<5;++i) {
        Check(torch::allclose(history[i],history[0]),"repeat current frame at reset");
        Check(torch::allclose(history[i].slice(0,33,45),torch::zeros({12})),"previous action cleared on every entry");
        Check(torch::allclose(history[i].slice(0,45,51),a.obs.arm_pos),"reset uses current arm feedback");
      }
      auto action=c.Tick(0); Check(action.has_value(),"offline RL target computed");
      Check(c.RequestMode(Mode::HOLD_TRANSITION,ready),"offline hold transition");
      Check(c.Tick(0)->q==measured,"transition starts at measured pose");
      Check(c.Tick(1)->q==end.q && c.mode()==Mode::HOLD,"transition ends in mapped hold");
    }
    c.RequestMode(Mode::DISARMED,{}); Check(!c.Tick(0),"disarm suppresses targets");
    c.RequestMode(Mode::FAULT,{}); Check(!c.Tick(0) && !c.RequestMode(Mode::HOLD,ready),"core fault latch");
    bool rejected=false; try { c.Load("",argv[2]); } catch(...) { rejected=true; }
    Check(rejected && !c.loaded() && !c.Tick(0),"missing config fails closed");
    rejected=false; try { c.Load(argv[1],"/nonexistent/policy.pt"); } catch(...) { rejected=true; }
    Check(rejected && !c.loaded(),"missing policy fails closed");
    rejected=false; try { c.Load(argv[1],argv[1]); } catch(...) { rejected=true; }
    Check(rejected && !c.loaded(),"invalid TorchScript fails closed");
    std::cout<<"real_controller_contract_test: PASS\n";
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
