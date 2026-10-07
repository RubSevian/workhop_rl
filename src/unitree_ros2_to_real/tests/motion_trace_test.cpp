#include "motion_trace.hpp"
#include <cassert>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>
#include <sstream>
using namespace sim2real;
int main(){
 const auto base="/tmp/go2-motion-trace-test-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
 MotionTraceSample sample;sample.kind=1;sample.accepted=1;sample.command={.5,.1,-.2};sample.target[0]=1.25;
 {MotionTrace trace(base+".csv",.04);assert(trace.Submit(sample));std::this_thread::sleep_for(std::chrono::milliseconds(80));assert(!trace.enabled());assert(!trace.Submit(sample));}
 std::ifstream input(base+".csv");std::string header,row,extra;assert(std::getline(input,header)&&std::getline(input,row));assert(!std::getline(input,extra));
 auto count=[](const auto& s){return std::count(s.begin(),s.end(),',');};assert(count(header)==102&&count(row)==102);
 assert(header.find("compute_ms")!=std::string::npos&&header.find("action11")!=std::string::npos);
 auto split=[](const std::string& s){std::vector<std::string> v;std::istringstream stream(s);std::string field;while(std::getline(stream,field,','))v.push_back(field);return v;};
 const auto names=split(header),data=split(row);
 auto value=[&](const std::string& name){auto at=std::find(names.begin(),names.end(),name);assert(at!=names.end());return std::stod(data[at-names.begin()]);};
 assert(value("kind")==1&&value("accepted")==1&&value("command0")==.5&&value("command2")==-.2&&value("target0")==1.25);
 {MotionTrace trace(base+"-stop.csv",120);assert(trace.Submit(sample));trace.Stop();assert(!trace.Submit(sample));}
 {MotionTrace trace(base+"-burst.csv",120);std::vector<std::thread> producers;
  for(int t=0;t<4;++t)producers.emplace_back([&]{for(int n=0;n<10000;++n)trace.Submit(sample);});
  for(auto& t:producers)t.join();trace.Stop();assert(trace.dropped()>0);assert(!trace.Submit(sample));}
 bool invalid=false;try{MotionTrace trace(base+"-invalid.csv",0);}catch(const std::invalid_argument&){invalid=true;}assert(invalid);
 for(const auto& suffix:{".csv","-stop.csv","-burst.csv"})std::filesystem::remove(base+suffix);
 std::cout<<"PASS bounded CSV queue, concurrent overload/drop, auto-stop, explicit stop/drain,103-column schema and invalid duration\n";
}
