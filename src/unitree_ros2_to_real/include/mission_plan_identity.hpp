#pragma once
#include <yaml-cpp/yaml.h>
#include <optional>
#include <string>
#include <cstdint>
#include <array>
#include <algorithm>
#include <stdexcept>
#include <vector>

namespace sim2real {
struct MissionPlanIdentity {
 uint64_t id{}, revision{};
 std::string session, owner, navigation;
 static MissionPlanIdentity Read(const YAML::Node& v) {
  MissionPlanIdentity result{v["id"].as<uint64_t>(),v["goal_revision"].as<uint64_t>(),
   v["session_id"].as<std::string>(),v["owner_session_id"].as<std::string>(),v["navigation_session_id"].as<std::string>()};
  if(result.session.empty()||result.owner.empty()||result.navigation.empty())throw std::runtime_error("empty plan session");
  return result;
 }
 bool Matches(const MissionPlanIdentity& other) const {
  return id==other.id&&revision==other.revision&&session==other.session&&owner==other.owner&&navigation==other.navigation;
 }
};
inline std::optional<MissionPlanIdentity> PlanWireIdentity(const std::string& wire) {
 // Do not scan or parse an unbounded message on the owner's control thread.
 const auto end=wire.substr(0,2049).find('\n');
 if(end==std::string::npos||end==0||end>2048||wire[end-1]!=',')return std::nullopt;
 try{return MissionPlanIdentity::Read(YAML::Load(wire.substr(0,end-1)+"}")["request_identity"]);}
 catch(const std::exception&){return std::nullopt;}
}

struct ParsedMissionPlan {
 std::optional<MissionPlanIdentity> identity;
 YAML::Node header;
 std::vector<std::array<float,7>> points;
 std::string error;
};
inline ParsedMissionPlan ParseMissionPlan(const std::string& wire,size_t limit,
                                         std::optional<MissionPlanIdentity> identity) {
 ParsedMissionPlan result;result.identity=identity;
 try {
  auto v=YAML::Load(wire);
  const auto decoded=MissionPlanIdentity::Read(v);
  if(identity&&!identity->Matches(decoded))throw std::runtime_error("PLAN_IDENTITY_MISMATCH");
  result.identity=decoded;result.header=v;
  const auto samples=v["points"].as<std::vector<std::vector<float>>>();
  if(samples.size()>limit)throw std::runtime_error("plan sample cap exceeded");
  result.points.reserve(samples.size());
  for(const auto& q:samples){
   if(q.size()!=7)throw std::runtime_error("seven target values required");
   std::array<float,7> p;std::copy(q.begin(),q.end(),p.begin());result.points.push_back(p);
  }
  v.remove("points");result.header=v;
 }catch(const std::exception& e){result.error=e.what();}
 return result;
}
}
