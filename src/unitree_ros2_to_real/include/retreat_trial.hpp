#pragma once
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>
namespace sim2real {
// No qualification bit is written. Authorization is for one immutable staged
// plan/ticket only, and live authority must still hold at approval and consume.
class RetreatTrialApproval {
 public:
 void Stage(std::string ticket){ticket_=std::move(ticket);approved_=consumed_=false;}
 bool Approve(const std::string& ticket,bool live){
  if(!live||ticket.empty()||ticket!=ticket_||approved_||consumed_)return false;
  approved_=true;return true;
 }
 bool Consume(const std::string& ticket,bool live){
  if(!live||ticket!=ticket_||!approved_||consumed_)return false;
  consumed_=true;approved_=false;return true;
 }
 void Revoke(){ticket_.clear();approved_=consumed_=false;}
 bool approved()const{return approved_;}
 const std::string& ticket()const{return ticket_;}
 private:std::string ticket_;bool approved_=false,consumed_=false;
};
// Same geometric polyline, including every original sample. Four interpolated
// command intervals per original interval: 25% of original sampled velocity.
inline std::vector<std::array<float,7>> SlowRetreat(const std::vector<std::array<float,7>>& points,size_t cap){
 if(cap<2||points.size()<2||points.size()-1>(cap-1)/4)throw std::invalid_argument("slow RETREAT exceeds existing duration/sample cap");
 std::vector<std::array<float,7>> out;out.reserve((points.size()-1)*4+1);out.push_back(points.front());
 for(size_t i=1;i<points.size();++i)for(int k=1;k<=4;++k){
  std::array<float,7> q;for(size_t j=0;j<7;++j){
   if(!std::isfinite(points[i-1][j])||!std::isfinite(points[i][j]))throw std::invalid_argument("nonfinite RETREAT");
   q[j]=k==4?points[i][j]:float(double(points[i-1][j])+(double(points[i][j])-points[i-1][j])*k/4.);
  }out.push_back(q);
 }return out;
}
}
