// Actual BNO implementation with all device syscalls intercepted. No real I/O.
// Linux link: --wrap=open,--wrap=open64,--wrap=ioctl,--wrap=read,--wrap=write,--wrap=close.
#include <cstdarg>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <string>
#include <thread>
#include "rt/rt_bno055.h"
#include "control_board_hardware_interface/imu_health.hpp"
#include "test_bno_raw_fixtures.hpp"

static std::vector<RawRead> input;
static size_t cursor=0;
static unsigned opens=0,closes=0,writes=0;
static bool fail_open=false;
void require(bool v,const char *m) { if(!v)throw std::runtime_error(m); }
extern "C" int bno_fake_open(const char *path,int,...) {
  require(std::string(path)=="/dev/i2c-1","unexpected device path");++opens;
  return fail_open ? -1 : 73;
}
extern "C" int bno_fake_close(int fd) { require(fd==73,"unexpected close");++closes;return 0; }
extern "C" int bno_fake_ioctl(int fd,unsigned long request,...) {
  require(fd==73 && request==I2C_SLAVE,"unexpected ioctl");
  va_list args;va_start(args,request);int address=va_arg(args,int);va_end(args);
  require(address==0x4b,"unexpected sensor address");return 0;
}
extern "C" ssize_t bno_fake_read(int fd,void *dst,size_t count) {
  require(fd==73,"unexpected read fd");
  if(cursor==input.size()) { require(count==4,"unexpected extra payload read");std::memset(dst,0,count);return count; }
  const auto &r=input[cursor++];require(r.requested==count,"fixture read differs");
  std::memcpy(dst,r.bytes.data(),r.bytes.size());return r.returned;
}
extern "C" ssize_t bno_fake_write(int fd,const void *src,size_t count) {
  require(fd==73,"unexpected write fd");const auto *b=static_cast<const uint8_t*>(src);
  require((count==5 && b[2]==1 && b[4]==1) || (count==6 && b[2]==2 && b[4]==0xf9)
      || (count==21 && b[2]==2 && b[4]==0xfd && (b[5]==2 || b[5]==5)),"unexpected sensor command");
  ++writes;return count;
}
extern "C" int __wrap_open(const char *p,int flags,...) { return bno_fake_open(p,flags); }
extern "C" int __wrap_open64(const char *p,int flags,...) { return bno_fake_open(p,flags); }
extern "C" int __wrap_close(int fd) { return bno_fake_close(fd); }
extern "C" ssize_t __wrap_read(int fd,void *p,size_t n) { return bno_fake_read(fd,p,n); }
extern "C" ssize_t __wrap_write(int fd,const void *p,size_t n) { return bno_fake_write(fd,p,n); }
extern "C" int __wrap_ioctl(int fd,unsigned long request,...) {
  va_list args;va_start(args,request);int address=va_arg(args,int);va_end(args);
  return bno_fake_ioctl(fd,request,address);
}
void load(std::vector<RawRead> rows) { require(cursor==input.size(),"fixture was not fully consumed");input=std::move(rows);cursor=0; }
std::vector<RawRead> packet(std::vector<uint8_t> payload,uint8_t channel=3) {
  require(payload.size()<=28,"test packet too large");
  const uint8_t size=uint8_t(payload.size()+4);
  std::vector<uint8_t> wire={size,0x80,channel,1};wire.insert(wire.end(),payload.begin(),payload.end());
  return {{4,4,{size,0,channel,0}},{size,size,wire}};
}
void queued_age_checks(BNO055 &imu) {
  using namespace std::chrono;
  // Exact base + gyro record sliced from post-motor-recurrence-01 packet2:
  // fb19ef0b00 ... 027e0261e5ff0100ffff; relative ticks=-782105+97.
  load(packet({0xfb,0x19,0xef,0x0b,0,2,0x7e,2,0x61,0xe5,0xff,1,0,0xff,0xff}));
  const auto before=steady_clock::now();const auto queued=imu.sample();const auto now=steady_clock::now();
  require(queued.gyro_valid && queued.gyro_minimum_age_at_receipt==microseconds(78200800),"captured queued gyro age lost");
  require(queued.gyro_receipt_time>=before && queued.gyro_receipt_time<=now,"true receipt shifted to inferred sample time");
  require(pupper_imu::feedback_error(queued,now),"78second queued gyro accepted as fresh");
  // Signed base=-3, rebase=-1000, delay=97 => -900 ticks, or90ms.
  load(packet({0xfb,0xfd,0xff,0xff,0xff,0xfa,0x18,0xfc,0xff,0xff,2,0x7f,3,97,0,0,0,0,0,0}));
  const auto aged=imu.sample();require(aged.gyro_minimum_age_at_receipt==milliseconds(90),"signed base/rebase/delay age wrong");
  auto aligned=aged;aligned.quaternion_receipt_time=aged.gyro_receipt_time;aligned.quaternion_minimum_age_at_receipt=microseconds(0);
  require(!pupper_imu::feedback_error(aligned,aged.gyro_receipt_time),"90ms lower bound prematurely rejected");
  require(pupper_imu::feedback_error(aligned,aged.gyro_receipt_time+milliseconds(20)),"90ms plus20ms cached accepted");
  load(packet({0xfb,0xff,0xff,0xff,0x7f,2,0x80,3,0,0,0,0,0,0,0}));
  const auto unknown=imu.sample();require(!unknown.gyro_valid&&unknown.gyro_receipt_time==aged.gyro_receipt_time,"unknown timing freshened usable gyro");
  // Positive relative offset is no proven age, not an absolute future timestamp.
  load(packet({0xfb,0xfd,0xff,0xff,0xff,2,0x81,3,0,0,0,0,0,0,0}));
  const auto recovered=imu.sample();require(recovered.gyro_valid&&recovered.gyro_minimum_age_at_receipt==microseconds(0),"fresh signed timing did not recover");
  require(recovered.gyro_receipt_time<=steady_clock::now(),"positive offset created future receipt");
  load(packet({0xfb,0xff,0xff,0xff,0x7f,5,2,0,0,0,0,0,0,0,0,0,0x40,0,0}));
  const auto unknown_q=imu.sample();require(!unknown_q.quaternion_valid&&unknown_q.gyro_valid,"unknown quaternion timing hidden by fresh gyro");
  load(packet({0xfb,0,0,0,0,5,3,0,0,0,0,0,0,0,0,0,0x40,0,0}));
  const auto fresh=imu.sample();require(fresh.quaternion_valid&&fresh.quaternion_minimum_age_at_receipt==microseconds(0),"quaternion did not recover from unknown time");
  require(!pupper_imu::feedback_error(fresh,steady_clock::now()),"fresh streams rejected after recovery");
}
int main(int argc,char **argv) {
 try {
  require(argc==2,"scenario required");const std::string mode=argv[1];
  if(mode=="open_failure") {
    fail_open=true;bool threw=false;try{BNO055 imu(1,10000);}catch(const std::runtime_error&){threw=true;}
    require(threw && writes==0 && closes==0,"failed open used device");
  } else {
   {
    BNO055 imu(1,10000);
    if(mode=="initial") {
      auto x=imu.sample();require(!x.quaternion_valid&&!x.gyro_valid&&!x.acceleration_valid,"unobserved component valid");
      require(x.acc.isZero()&&x.gyro.isZero()&&x.quat.coeffs().isZero(),"uninitialized output");
    } else if(mode=="gyro_only") {
      load(normal_gyro());auto x=imu.sample();require(x.gyro_valid&&!x.quaternion_valid,"gyro invented quaternion");
    } else {
      load(mode=="delayed_tail" ? zero_header_then_late_one_byte() : valid_two_reports());
      const auto before=imu.sample();
      require(before.quaternion_valid&&before.gyro_valid&&before.gyro_accuracy==3&&before.quaternion_accuracy==0,"valid report metadata lost");
      require(!before.acceleration_valid&&before.acc.isZero(),"unavailable acceleration claimed measured");
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      if(mode=="delayed_tail" || mode=="valid") {
        require(before.quat.squaredNorm()>.99f&&before.quat.squaredNorm()<1.01f,"quaternion decode invalid");
      } else if(mode=="duplicate")load(valid_two_reports());
      else if(mode=="gyro_refresh")load(normal_gyro());
      else if(mode=="control")load(packet({0xf1,0,7,0,0,0,1,0,1,0,1,0,0,0,0,0},2));
      else if(mode=="game_not_selected")load(packet({0xfb,0,0,0,0,8,2,3,0,0,0,0,0,0,0,0,0x40}));
      else if(mode=="bad_quaternion")load(packet({0xfb,0,0,0,0,5,2,0,0,0,0,0,0,0,0,0,0,0,0}));
      else if(mode=="truncated")load(packet({0xfb,0,0,0,0,2,2,3,0,0,0}));
      else if(mode=="short_read") {auto rows=normal_gyro();--rows.back().returned;rows.back().bytes.pop_back();load(rows);}
      else throw std::runtime_error("unknown scenario");
      const auto after=imu.sample();
      if(mode=="bad_quaternion")require(!after.quaternion_valid&&after.gyro_valid,"invalid quaternion kept valid");
      else if(mode=="gyro_refresh")require(after.gyro_receipt_time>before.gyro_receipt_time&&after.quaternion_receipt_time==before.quaternion_receipt_time,"independent timestamps conflated");
      else require(after.quaternion_receipt_time==before.quaternion_receipt_time&&after.gyro_receipt_time==before.gyro_receipt_time,"non-new/invalid report refreshed cached state");
    }
    if(mode=="valid")queued_age_checks(imu);
    require(cursor==input.size(),"unconsumed fake reads");
   }
   require(opens==1&&closes==1&&writes==4,"unexpected constructor/device command lifecycle");
  }
  std::cout<<"PASS "<<mode<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
