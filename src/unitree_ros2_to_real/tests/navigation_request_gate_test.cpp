#include "navigation_request_gate.hpp"
#include <cassert>
#include <limits>
using namespace sim2real;
int main(){
 auto at=[](double t){return SafetyTime{}+std::chrono::duration_cast<SafetyClock::duration>(std::chrono::duration<double>(t));};
 NavigationRequestGate g({.2,.1,.1});
 assert(!g.Accept({.1,0,0},0,at(1)));
 assert(g.Observe("a",true,true,at(1),at(1)));
 for(int axis=0;axis<3;++axis)for(int sign:{-1,1}){
  std::array<double,3> v{};v[axis]=sign*9.;assert(g.Accept(v,0,at(1)));
  assert(g.Get(at(1))[axis]==sign*(axis==0?.2:.1));
 }
 assert(g.Accept({.1,0,0},.2,at(1)));
 assert(g.Get(at(1.04))[0]==.1);assert(g.Get(at(1.06))[0]==0);
 assert(g.Observe("a",true,false,at(1.1),at(1.1)));assert(g.Ready(at(1.1)));assert(!g.Enabled(at(1.1)));
 assert(!g.Accept({.1,0,0},0,at(1.1)));
 assert(g.Observe("a",true,true,at(1.2),at(1.2)));assert(g.Get(at(1.2))[0]==0);
 assert(g.Accept({.1,0,0},0,at(1.2)));
 assert(g.Observe("b",true,true,at(1.3),at(1.3)));assert(g.Get(at(1.3))[0]==0);
 assert(g.Accept({.1,0,0},0,at(1.3)));
 assert(!g.Accept({std::numeric_limits<double>::quiet_NaN(),0,0},0,at(1.3)));assert(g.Get(at(1.3))[0]==0);
 assert(!g.Observe("b",true,true,at(1.3),at(1.4)));assert(!g.Ready(at(1.4)));
 assert(!g.Observe("",true,true,at(2),at(2)));
 assert(!g.Observe("c",true,true,at(2.1),at(2)));
 assert(g.Observe("c",true,true,at(2),at(2)));assert(!g.Ready(at(2.6)));
}
