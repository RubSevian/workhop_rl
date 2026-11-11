#include "transition_events.hpp"
#include <cassert>
using namespace sim2real;
TransitionEvent event(int id,bool fault=false){TransitionEvent e;e.data.Add("sequence",id);e.fault=fault;return e;}
int main(){
 TransitionEvents<2> q;q.Push(event(1));q.Push(event(2));q.Push(event(3,true));
 assert(q.dropped()==1);
 for(int id:{1,2}){auto e=q.Pop();assert(e);assert(YAML::Load(e->data.Serialize())["sequence"].as<int>()==id);}
 assert(!q.Pop());auto fault=q.TakeOverflowFault();assert(fault&&fault->fault);
 assert(YAML::Load(fault->data.Serialize())["sequence"].as<int>()==3);assert(!q.TakeOverflowFault());
 q.Push(event(4));assert(YAML::Load(q.Pop()->data.Serialize())["sequence"].as<int>()==4);
}
