#include "control_latency.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unistd.h>
using namespace sim2real;
using namespace std::chrono_literals;
int main(){
 const auto path="/tmp/control_latency_test_"+std::to_string(getpid())+".csv";
 PriorityMutex mutex;
 int counter=0;
 {std::vector<std::thread> threads;for(int i=0;i<4;++i)threads.emplace_back([&]{for(int j=0;j<5000;++j){std::lock_guard lock(mutex);++counter;}});
 for(auto& t:threads)t.join();}
 assert(counter==20000);
 uint64_t burst_dropped=0;
 {
  ControlLatency trace(path,8192,2,20);
  std::atomic<bool> held{false};
  std::thread owner([&]{ControlLock lock(mutex,&trace,11);held=true;std::this_thread::sleep_for(35ms);});
  while(!held)std::this_thread::yield();
  {ControlScope callback(&trace,2,13,ControlMonotonicNs(),ControlMonotonicNs());ControlLock waiter(mutex,&trace,12);++counter;}
  owner.join();assert(trace.dropped()==0);
 }
 std::ifstream stream(path);std::string line;bool owner=false,waiter=false,scope=false;
 std::getline(stream,line);
 while(std::getline(stream,line)){
  if(line.starts_with('#'))continue;
  for(char& c:line)if(c==',')c=' ';
  ControlLatencySample s;std::istringstream in(line);
  in>>s.kind>>s.site>>s.tid>>s.begin>>s.acquired>>s.end>>s.cpu_ns>>s.expected>>s.actual;
  assert(in&&s.end>=s.begin&&s.cpu_ns>=0&&s.tid>0);
  if(s.site==11){owner=true;assert(s.end-s.acquired>=30000000);}
  if(s.site==12){waiter=true;assert(s.acquired-s.begin>=20000000);assert(s.cpu_ns<s.end-s.begin);}
  if(s.site==13){scope=true;assert(s.end-s.begin>=20000000&&s.expected>0);}
 }
 assert(owner&&waiter&&scope);
 {
  ControlLatency trace(path,8192,2,20);
  std::vector<std::thread> producers;
  for(int id=0;id<4;++id)producers.emplace_back([&,id]{
   for(int n=0;n<1000;++n){ControlLatencySample s;s.kind=7;s.site=id;s.tid=n;s.end=ControlMonotonicNs();trace.Submit(s);}
  });
  for(auto& p:producers)p.join();burst_dropped=trace.dropped();
 }
 {std::ifstream f(path);std::string row;bool seen[4][1000]{};int count=0;
  std::getline(f,row);
  while(std::getline(f,row)){if(row.starts_with('#'))continue;for(char& c:row)if(c==',')c=' ';
   std::istringstream in(row);int kind,id,n;in>>kind>>id>>n;
   assert(kind==7&&id>=0&&id<4&&n>=0&&n<1000&&!seen[id][n]);seen[id][n]=true;++count;
  }assert(count>0&&count+burst_dropped==4000);
 }
 {ControlLatency trace(path,256,.005,1);std::this_thread::sleep_for(15ms);assert(!trace.enabled());}
 bool invalid=false;try{ControlLatency trace(path,0,1,20);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
 std::filesystem::remove(path);
 std::cout<<"PASS PI mutex exclusion, complete scopes, injected 35ms owner/waiter attribution, trace expiry/config\n";
}
