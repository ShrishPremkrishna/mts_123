// Actual BNO implementation with all device syscalls intercepted. No real I/O.
// Linux link: --wrap=open,--wrap=open64,--wrap=ioctl,--wrap=read,--wrap=write,--wrap=close.
#include <atomic>
#include <cstdarg>
#include <cstring>
#include <functional>
#include <fstream>
#include "control_board_hardware_interface/redundant_imu.hpp"
#include <stdexcept>
#include <iostream>
#include <string>
#include <thread>
#include "control_board_hardware_interface/imu_manager.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include "test_bno_raw_fixtures.hpp"

static std::vector<RawRead> input;
static size_t cursor=0;
static unsigned opens=0,closes=0,writes=0;
using Clock = std::chrono::steady_clock;
// Only this intercepted-device test links the Linux steady_clock::now wrapper.
// All age comparisons and the real BNO decoder see the exact archived receipt.
static Clock::time_point replay_time{};
extern "C" Clock::time_point __real__ZNSt6chrono3_V212steady_clock3nowEv();
extern "C" Clock::time_point __wrap__ZNSt6chrono3_V212steady_clock3nowEv() {
  return replay_time==Clock::time_point{} ? __real__ZNSt6chrono3_V212steady_clock3nowEv() : replay_time;
}
static bool fail_open=false;
static bool delay_next_empty_read = false;
static std::function<void()> during_next_probe;
static std::function<void()> after_sensor_setup;
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
  if (cursor == input.size()) {
    require(count == 4, "unexpected extra payload read");
    if (during_next_probe) {
      auto observe = std::move(during_next_probe);
      during_next_probe = {};
      observe();
    }
    if (delay_next_empty_read) {
      delay_next_empty_read = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(120));
    }
    std::memset(dst, 0, count);
    return count;
  }
  const auto &r=input[cursor++];require(r.requested==count,"fixture read differs");
  std::memcpy(dst,r.bytes.data(),r.bytes.size());return r.returned;
}
extern "C" ssize_t bno_fake_write(int fd,const void *src,size_t count) {
  require(fd==73,"unexpected write fd");const auto *b=static_cast<const uint8_t*>(src);
  require((count==5 && b[2]==1 && b[4]==1) || (count==6 && b[2]==2 && b[4]==0xf9)
      || (count==21 && b[2]==2 && b[4]==0xfd && (b[5]==2 || b[5]==5 || b[5]==7 || b[5]==8)),"unexpected sensor command");
  ++writes;
  if (writes == 4 && after_sensor_setup) after_sensor_setup();
  return count;
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
  const unsigned size=unsigned(payload.size())+4;
  std::vector<RawRead> reads{{4,4,{uint8_t(size),uint8_t(size>>8),channel,0}}};
  unsigned seq=1;
  for (size_t i=0;i<payload.size();i+=28) {
    const unsigned remain=size-i;
    const auto end=std::min(i+28,payload.size());
    std::vector<uint8_t> wire{uint8_t(remain),uint8_t(0x80|(remain>>8)),channel,uint8_t(seq++)};
    wire.insert(wire.end(),payload.begin()+i,payload.begin()+end);
    reads.push_back({unsigned(wire.size()),int(wire.size()),wire});
  }
  return reads;
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
#include "replay_redundant_reports.hpp"

int main(int argc,char **argv) {
 try {
  if (argc==4 && std::string(argv[1])=="archive") {
    replay_archive(argv[2],argv[3]);return 0;
  }
  require(argc==2,"scenario required");const std::string mode=argv[1];
  if (mode == "manager_blocked_read") {
    using namespace std::chrono;
    std::atomic<bool> probe_started{false};
    after_sensor_setup = [&] {
      load(valid_two_reports());
      delay_next_empty_read = true;
      during_next_probe = [&] { probe_started = true; };
    };
    IMUManager manager;
    manager.begin(10000);
    const auto deadline = steady_clock::now() + seconds(1);
    while (!probe_started && steady_clock::now() < deadline) std::this_thread::yield();
    require(probe_started, "sampler did not enter the following I2C read");
    const auto published = manager.get_imu_data();
    const auto observed = steady_clock::now();
    require(!pupper_imu::feedback_error(published, observed), "controller missed a completed pair during blocked I2C read");
    std::cout << "threaded_publication_age_us="
              << duration_cast<microseconds>(observed - published.gyro_receipt_time).count() << '\n';
    manager.stop();
    const auto cached = manager.get_imu_data();
    require(cached.gyro_receipt_time == published.gyro_receipt_time &&
            cached.quaternion_receipt_time == published.quaternion_receipt_time &&
            pupper_imu::feedback_error(cached, steady_clock::now()), "blocked read concealed actual expiry");
  } else if(mode=="open_failure") {
    fail_open=true;bool threw=false;try{BNO055 imu(1,10000);}catch(const std::runtime_error&){threw=true;}
    require(threw && writes==0 && closes==0,"failed open used device");
  } else {
   {
    BNO055 imu(1,10000);
    if(mode=="redundant_wire") {
      imu.enable_redundant_reports(10000);
      std::vector<uint8_t> bytes={0xfb,15,0,0,0,7,19,2,0,0,4,0,0,0,0,0,2,0,0,0,0,
                                  8,42,3,0,0,0,0,0,0,0,0,0x40};
      load(packet(bytes));const auto a=imu.sample();
      require(a.alternative.gyro.x()==1.f && a.alternative.quat.w()==1.f &&
              a.alternative.quaternion_report_sequence==42 && a.alternative.gyro_report_sequence==19 &&
              a.alternative.gyro_minimum_age_at_receipt==std::chrono::microseconds(1500) &&
              !a.quaternion_valid && !a.gyro_valid,"independent wire reports or gyro bias incorrect");
      load(packet(bytes));const auto duplicate=imu.sample();
      bytes[6]=18;bytes[22]=41;
      load(packet(bytes));const auto earlier=imu.sample();
      require(duplicate.alternative.gyro_receipt_time==a.alternative.gyro_receipt_time &&
              earlier.alternative.quaternion_receipt_time==a.alternative.quaternion_receipt_time,"replay advanced clock");
      bytes[6]=26;bytes[22]=49;
      load(packet(bytes));const auto gap=imu.sample();
      require(gap.alternative.gap_count==2,"missed auxiliary sequence discontinuity");
      load(packet({1},1));const auto reset=imu.sample();
      require(reset.alternative.reset_count==1 && !reset.alternative.quaternion_valid &&
              !reset.alternative.gyro_valid && !reset.quaternion_valid && !reset.gyro_valid,"chip reset hidden");
    } else if(mode=="initial") {
      auto x=imu.sample();require(!x.quaternion_valid&&!x.gyro_valid&&!x.acceleration_valid,"unobserved component valid");
      require(x.acc.isZero()&&x.gyro.isZero()&&x.quat.coeffs().isZero(),"uninitialized output");
      require(x.quaternion_report_sequence == -1 && x.gyro_report_sequence == -1, "unseen sequence invented");
    } else if (mode == "stale_before_fresh") {
      load(valid_two_reports());
      const auto first = imu.sample();
      require(first.gyro_valid && first.quaternion_valid, "missing initial paired reports");
      // One second of known age in the first queued gyro. Do not end the drain
      // early until the second, genuinely fresh report completes the pair.
      auto rows = packet({0xfb,0x10,0x27,0,0,2,2,3,0,0,0,0,0,0,0});
      const auto fresh = packet({0xfb,0,0,0,0,2,3,3,0,0,0,0,0,0,0});
      rows.insert(rows.end(), fresh.begin(), fresh.end());
      load(std::move(rows));
      const auto recovered = imu.sample();
      require(recovered.gyro_report_sequence == 3 && recovered.gyro_minimum_age_at_receipt.count() == 0,
              "an obsolete gyro hid the queued fresh one");
      require(recovered.quaternion_receipt_time == first.quaternion_receipt_time,
              "gyro packet renewed cached quaternion");
    } else if (mode == "overlap_deadline") {
      // Model an independent controller read 25 ms into the next I2C probe.
      // The last published pair is 90 ms old. The packet already read from
      // the device contains a fresh pair. The 100 ms limit stays in effect.
      using namespace std::chrono;
      BNO055::Output published;
      const auto start = steady_clock::now();
      published.quat = Eigen::Quaternionf::Identity();
      published.gyro_valid = published.quaternion_valid = true;
      published.quaternion_receipt_time = published.gyro_receipt_time = start - milliseconds(90);
      require(!pupper_imu::feedback_error(published, start), "old pair was not initially usable");
      bool saw_usable_pair = false;
      during_next_probe = [&]() {
        saw_usable_pair = !pupper_imu::feedback_error(published, steady_clock::now() + milliseconds(25));
      };
      load(valid_two_reports());
      published = imu.sample();
      if (during_next_probe) imu.sample();
      require(saw_usable_pair, "a short following read caused an avoidable deadline fault");
    } else if (mode == "following_read_blocked") {
      // One captured SH-2 packet carries both reports. The next header read stalls
      // in the OS. No sensor report arrives during that stall.
      load(valid_two_reports());
      delay_next_empty_read = true;
      const auto first = imu.sample();
      const auto returned = std::chrono::steady_clock::now();
      std::cout << "packet available after "
                << std::chrono::duration_cast<std::chrono::milliseconds>(returned - first.gyro_receipt_time).count()
                << " ms from receipt" << std::endl;
      require(delay_next_empty_read, "reader waited on next I2C transfer before publishing the complete report");
      require(!pupper_imu::feedback_error(first, returned), "fresh complete packet rejected");
      const auto cached = imu.sample();
      require(!delay_next_empty_read, "following transfer never attempted");
      require(cached.gyro_receipt_time == first.gyro_receipt_time &&
              cached.quaternion_receipt_time == first.quaternion_receipt_time, "stall renewed cached reports");
      require(pupper_imu::feedback_error(cached, std::chrono::steady_clock::now()), "lost input hid expiry");
    } else if(mode=="gyro_only") {
      load(normal_gyro());auto x=imu.sample();require(x.gyro_valid&&!x.quaternion_valid,"gyro invented quaternion");
      require(x.gyro_report_sequence == 2 && x.quaternion_report_sequence == -1, "gyro-only sequence lost");
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
      if (mode == "gyro_refresh" || mode == "duplicate") {
        require(after.quaternion_report_sequence == 1, "unchanged orientation sequence advanced");
        require(after.gyro_report_sequence == (mode == "gyro_refresh" ? 2 : 1), "wrong gyro report sequence");
      }
      if(mode=="bad_quaternion")require(!after.quaternion_valid&&after.gyro_valid,"invalid quaternion kept valid");
      else if(mode=="gyro_refresh")require(after.gyro_receipt_time>before.gyro_receipt_time&&after.quaternion_receipt_time==before.quaternion_receipt_time,"independent timestamps conflated");
      else require(after.quaternion_receipt_time==before.quaternion_receipt_time&&after.gyro_receipt_time==before.gyro_receipt_time,"non-new/invalid report refreshed cached state");
    }
    if(mode=="valid")queued_age_checks(imu);
    require(cursor==input.size(),"unconsumed fake reads");
   }
   require(opens==1&&closes==1&&writes==(mode=="redundant_wire"?6u:4u),"unexpected constructor/device command lifecycle");
  }
  std::cout<<"PASS "<<mode<<'\n';return 0;
 }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
