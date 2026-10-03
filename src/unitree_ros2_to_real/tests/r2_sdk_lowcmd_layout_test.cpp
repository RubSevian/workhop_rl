#include <unitree/idl/go2/LowCmd_.hpp>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <iostream>
using Cmd=unitree_go::msg::dds_::LowCmd_;
using Motor=unitree_go::msg::dds_::MotorCmd_;
int main(){
 static_assert(sizeof(Cmd)==812);static_assert(sizeof(Motor)==36);
 static_assert(std::is_trivially_copyable<Cmd>::value);
 Cmd c;std::memset(&c,0,sizeof(c));
 const auto* base=reinterpret_cast<const uint8_t*>(&c);
 auto offset=[&](const auto& field){return reinterpret_cast<const uint8_t*>(&field)-base;};
 assert(offset(c.head())==0&&offset(c.level_flag())==2&&offset(c.frame_reserve())==3);
 assert(offset(c.sn())==4&&offset(c.version())==12&&offset(c.bandwidth())==20);
 assert(offset(c.motor_cmd())==24&&offset(c.bms_cmd())==744&&offset(c.wireless_remote())==748);
 assert(offset(c.led())==788&&offset(c.fan())==800&&offset(c.gpio())==802&&offset(c.reserve())==804&&offset(c.crc())==808);
 c.head()={0xfe,0xef};c.level_flag()=0xff;
 for(int i=0;i<20;++i){auto& m=c.motor_cmd()[i];m.mode()=1;m.q()=2.146E9F;m.dq()=16000;
  const auto* mb=reinterpret_cast<const uint8_t*>(&m);
  assert(reinterpret_cast<const uint8_t*>(&m.q())-mb==4&&reinterpret_cast<const uint8_t*>(&m.dq())-mb==8&&reinterpret_cast<const uint8_t*>(&m.tau())-mb==12);
  assert(reinterpret_cast<const uint8_t*>(&m.kp())-mb==16&&reinterpret_cast<const uint8_t*>(&m.kd())-mb==20&&reinterpret_cast<const uint8_t*>(&m.reserve())-mb==24);
  if(i<12){m.q()=(i+1)*.125F;m.dq()=0;m.kp()=20+i;m.kd()=1+i*.125F;}
 }
 // SDK example CRC algorithm, executed on actual SDK IDL object bytes.
 uint32_t crc=0xffffffffU;
 for(size_t i=0;i<808;i+=4){uint32_t data;std::memcpy(&data,base+i,4);
  for(int j=0;j<32;++j){bool high=crc&0x80000000U;crc<<=1;if(high)crc^=0x04c11db7U;if(data&0x80000000U)crc^=0x04c11db7U;data<<=1;}
 }
 assert(crc==0xc0fb03a5U);
 std::cout<<"PASS SDK aarch64 LowCmd size812 Motor36 all offsets; CRC first808 bytes golden 0xc0fb03a5; no channels created\n";
}
