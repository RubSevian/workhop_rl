// Synthetic replay transport only: links messages/rclcpp, no SDK/controller.
#include "isolated_replay.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <array>
#include <vector>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstring>
#include <ctime>
#include <cerrno>
#include <algorithm>
using Ns=int64_t;
static Ns now(){timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);return Ns(t.tv_sec)*1000000000+t.tv_nsec;}
static bool sleep_until(Ns deadline){timespec t{deadline/1000000000,deadline%1000000000};int rc;do{rc=clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&t,nullptr);}while(rc==EINTR&&rclcpp::ok());if(rc&&rc!=EINTR)throw std::runtime_error("absolute sleep failed");return rclcpp::ok();}
struct Frame {uint64_t recorded_ns;std::array<float,31> values;};
struct Row {uint64_t sequence,frame,recorded_ns,skipped;Ns expected,begin,end;};
static std::vector<Frame> read_frames(const char* path) {
 std::ifstream f(path,std::ios::binary);char magic[8]{};uint64_t count=0;
 f.read(magic,8);f.read(reinterpret_cast<char*>(&count),8);
 if(std::memcmp(magic,"GO2LSA1",8)||!count||count>100000)throw std::runtime_error("invalid bounded measured-frame file");
 std::vector<Frame> result(count);
 for(auto& r:result){f.read(reinterpret_cast<char*>(&r.recorded_ns),8);f.read(reinterpret_cast<char*>(r.values.data()),31*4);for(float x:r.values)if(!std::isfinite(x))throw std::runtime_error("nonfinite measured frame");}
 if(!f||f.peek()!=EOF)throw std::runtime_error("truncated/trailing frame data");return result;
}
static std::string arm_json(Ns stamp) {
 const auto s=std::to_string(stamp);
 return "{\"synthetic_replay\":true,\"observed_ns\":"+s+",\"frame_received_ns\":"+s+",\"accepted_ns\":"+s+
 ",\"common_feedback_age_s\":0,\"feedback_ready\":true,\"measured_q\":[0,0,0,0,0,0],\"measured_dq\":[0,0,0,0,0,0],\"motor_ids\":[1,2,3,4,5,6],\"valid\":[true,true,true,true,true,true],\"static_hold\":true,\"arm_emergency_validated\":false,\"arm_home_ready\":true,\"motor_id\":[1,2,3,4,5,6,7],\"motor_status\":[1,1,1,1,1,1,1],\"valid7\":[true,true,true,true,true,true,true],\"q\":[0,0,0,0,0,0,0],\"dq\":[0,0,0,0,0,0,0],\"home_target\":[0,0,0,0,0,0,0],\"connected\":true,\"enabled_local\":true,\"watchdog_tripped\":false,\"stm32_watchdog_tripped\":false,\"owner_state\":\"HOLD_HOME\",\"target_valid\":true,\"accepted_target\":[0,0,0,0,0,0]}";
}
int main(int argc,char** argv){try{
 sim2real::RequireIsolatedReplay();
 if(argc!=5)throw std::runtime_error("usage: read_only_lowstate_source frames.bin seconds start.ns NEW_OUTPUT_DIR");
 const double seconds=std::stod(argv[2]);if(!std::isfinite(seconds)||seconds<5||seconds>570)throw std::runtime_error("seconds must be 5..570");
 const auto frames=read_frames(argv[1]);const std::filesystem::path output=argv[4],start_file=argv[3];
 std::vector<Row> rows;rows.reserve(size_t(seconds*500)+2);std::vector<std::array<Ns,3>> arms;arms.reserve(size_t(seconds*100)+2);
 rclcpp::init(argc,argv);auto node=std::make_shared<rclcpp::Node>("isolated_measured_lowstate_source");
 auto publisher=node->create_publisher<unitree_go::msg::LowState>("/lowstate",rclcpp::SensorDataQoS().keep_last(1));
 auto arm=node->create_publisher<std_msgs::msg::String>("/rars01/commissioning/state",rclcpp::QoS(1));
 std::ofstream(output/"low_source.ready")<<"ready\n";
 const auto timeout=now()+30000000000LL;Ns begin=0;
 while(rclcpp::ok()&&now()<timeout){std::ifstream f(start_file);if(f>>begin)break;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 if(!begin)throw std::runtime_error("no replay start deadline");
 const Ns period=2000000,end=begin+Ns(std::llround(seconds*1e9));uint64_t index=0,skipped_total=0;Ns next_arm=begin;
 unitree_go::msg::LowState message;std_msgs::msg::String arm_message;uint64_t last_frame=UINT64_MAX;
 while(rclcpp::ok()) {
  Ns due=begin+Ns(index)*period;if(due>=end||!sleep_until(due))break;
  const auto woke=now();const uint64_t skipped=std::max<Ns>(0,(woke-due)/period);
  index+=skipped;skipped_total+=skipped;due=begin+Ns(index)*period;if(due>=end)break;
  const auto frame=uint64_t(std::max<Ns>(0,now()-begin)/20000000)%frames.size();const auto& sample=frames[frame];
  if(frame!=last_frame){for(int i=0;i<12;++i){message.motor_state[i].q=sample.values[i];message.motor_state[i].dq=sample.values[12+i];}
   for(int i=0;i<4;++i)message.imu_state.quaternion[i]=sample.values[24+(i==0?3:i-1)];
   for(int i=0;i<3;++i)message.imu_state.gyroscope[i]=sample.values[28+i];last_frame=frame;}
  message.tick=uint32_t(index+1);const auto pub_begin=now();publisher->publish(message);const auto pub_end=now();
  rows.push_back({index+1,frame,sample.recorded_ns,skipped,due,pub_begin,pub_end});++index;
  if(pub_end>=next_arm){const auto stamp=now();arm_message.data=arm_json(stamp);const auto a=now();arm->publish(arm_message);arms.push_back({next_arm,a,now()});next_arm=begin+((stamp-begin)/10000000+1)*10000000;}
 }
 const auto finished=now();
 {std::ofstream f(output/"low_source.csv");f<<"sequence,frame_index,recorded_sample_ns,expected_send_ns,publish_begin_ns,publish_end_ns,publish_duration_ns,skipped_deadlines,source_lateness_ns,repeated_measurement\n";uint64_t previous=UINT64_MAX;
  for(auto& r:rows){f<<r.sequence<<','<<r.frame<<','<<r.recorded_ns<<','<<r.expected<<','<<r.begin<<','<<r.end<<','<<r.end-r.begin<<','<<r.skipped<<','<<r.begin-r.expected<<','<<(r.frame==previous)<<'\n';previous=r.frame;}}
 {std::ofstream f(output/"arm_source.csv");f<<"expected_send_ns,publish_begin_ns,publish_end_ns\n";for(auto& r:arms)f<<r[0]<<','<<r[1]<<','<<r[2]<<'\n';}
 std::ofstream(output/"low_source.done")<<"{\"begin_ns\":"<<begin<<",\"end_ns\":"<<end<<",\"finished_ns\":"<<finished<<",\"published\":"<<rows.size()<<",\"skipped_deadlines\":"<<skipped_total<<",\"synthetic_arm\":true,\"physical_sample_period_ns\":20000000}\n";
 rclcpp::shutdown();return 0;
 }catch(const std::exception& e){fprintf(stderr,"Read-only source failed closed: %s\n",e.what());if(rclcpp::ok())rclcpp::shutdown();return 1;}}
