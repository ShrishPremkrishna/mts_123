// Real driver read/activation callbacks with fake SPI/BNO and isolated locks.
// No device implementation is linked and every open except the fake lock fails.
#include "control_board_hardware_interface/control_board_hardware_interface.hpp"
#include "control_board_hardware_interface/imu_health.hpp"
#include <atomic>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>
namespace {
spi_command_t command{}; spi_data_t data{};
std::vector<spi_command_t> sent;
std::atomic<int> mode{0};  // 0fresh,1stalegyro,2stalequat,3bias,4badnorm,5missing
bool lose_during_enable=false;
constexpr int lock_fd=19322;
void require(bool ok,const char *why) { if(!ok) throw std::runtime_error(why); }
BNO055::Output output(std::chrono::steady_clock::time_point now) {
  BNO055::Output o;
  o.quat=Eigen::Quaternionf::Identity();o.gyro=Eigen::Vector3f::Zero();
  o.quaternion_valid=o.gyro_valid=true;
  o.quaternion_receipt_time=o.gyro_receipt_time=now;
  o.packet_timestamp=o.measurement_timestamp=std::chrono::high_resolution_clock::now();
  switch(mode.load()) {
    case 1:o.gyro_receipt_time-=std::chrono::seconds(1);break;
    case 2:o.quaternion_receipt_time-=std::chrono::seconds(1);break;
    case 3:o.gyro.x()=.13;break;
    case 4:o.quat.w()=.5;break;
    case 5:o.quaternion_valid=false;break;
    case 6:o.gyro_minimum_age_at_receipt=std::chrono::seconds(78);break;
    case 7:o.quaternion_minimum_age_at_receipt=std::chrono::seconds(78);break;
  }
  return o;
}
void neutral_only() {
  require(!sent.empty(),"no disable sent");const spi_command_t zero{};
  for(auto &p:sent)require(std::memcmp(&p,&zero,sizeof zero)==0,"nonneutral command escaped failed check");
}
void unit_health() {
  using namespace std::chrono;
  auto now=steady_clock::time_point{}+seconds(100);
  auto o=output(now);
  require(!pupper_imu::feedback_error(o,now),"fresh rejected");
  o.gyro_receipt_time-=milliseconds(101);
  require(pupper_imu::feedback_error(o,now),"fresh q hid stale gyro");
  o=output(now);o.quaternion_receipt_time-=milliseconds(101);
  require(pupper_imu::feedback_error(o,now),"fresh gyro hid stale q");
  o=output(now);o.gyro_receipt_time+=milliseconds(1);
  require(pupper_imu::feedback_error(o,now),"future receipt accepted");
  o=output(now);o.gyro.x()=std::numeric_limits<float>::infinity();
  require(pupper_imu::feedback_error(o,now),"infinite gyro accepted");
  o=output(now);o.quat.w()=.5;
  require(pupper_imu::feedback_error(o,now),"bad quaternion norm accepted");
  o=output(now);o.gyro_minimum_age_at_receipt=milliseconds(90);
  require(!pupper_imu::feedback_error(o,now),"90ms report rejected before age limit");
  require(pupper_imu::feedback_error(o,now+milliseconds(20)),"receipt freshened90ms plus20ms cached gyro");
  o=output(now);o.quaternion_minimum_age_at_receipt=seconds(78);
  require(pupper_imu::feedback_error(o,now),"fresh gyro hid queued quaternion");
  o=output(now);o.gyro_minimum_age_at_receipt=microseconds(-1);
  require(pupper_imu::feedback_error(o,now),"negative age bound accepted");
  pupper_imu::StartupWindow queued;
  for(int i=0;i<=400;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.gyro_minimum_age_at_receipt=seconds(78);require(!queued.observe(s,t),"old queue manufactured startup dwell");}
  pupper_imu::StartupWindow stable;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);bool ready=stable.observe(s,t);require(ready==(i==300),"stationary window timing wrong");}
  pupper_imu::StartupWindow bias;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.gyro.x()=.13;require(!bias.observe(s,t),"stationary bias accepted");}
  pupper_imu::StartupWindow drift;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto s=output(t);s.quat=Eigen::Quaternionf(Eigen::AngleAxisf(i*.0001f,Eigen::Vector3f::UnitX()));require(!drift.observe(s,t),"gravity drift accepted");}
  pupper_imu::StartupWindow yaw;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.quat=Eigen::Quaternionf(Eigen::AngleAxisf(i*.001f,Eigen::Vector3f::UnitZ()));require(yaw.observe(sample,t)==(i==300),"yaw-only rotation changed gravity gate");}
  pupper_imu::StartupWindow modest_bias;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.gyro.x()=.0144f;require(modest_bias.observe(sample,t)==(i==300),"allowed gyro mean rejected");}
  pupper_imu::StartupWindow coherent_bias;
  for(int i=0;i<=1600;++i){auto t=now+milliseconds(i*10);auto sample=output(t);sample.gyro.x()=.03f;pupper_imu::StartupObservation d;require(!coherent_bias.observe(sample,t,&d),"coherent gyro mean accepted");if(i==300)require(std::strcmp(d.reason,"gyro_mean")==0,"mean rejection reason missing");if(i==301)require(d.gyro_samples==1,"mean rejection did not clear window");}
  pupper_imu::StartupWindow distinct;
  for(int i=0;i<=600;++i){auto t=now+milliseconds(i*5);auto sample=output(t);sample.gyro.x()=.0144f;sample.gyro_receipt_time=now+milliseconds((i/2)*10);pupper_imu::StartupObservation d;require(distinct.observe(sample,t,&d)==(i==600),"distinct-receipt dwell timing wrong");require(d.gyro_samples==static_cast<std::size_t>(i/2+1),"cached gyro counted twice in mean");}
  pupper_imu::StartupWindow duplicated;
  o=output(now);
  for(int i=0;i<=400;++i)require(!duplicated.observe(o,now+milliseconds(i*10)),"cached sample advanced startup");
  pupper_imu::StartupWindow gap;
  require(!gap.observe(output(now),now),"initial window accepted");
  auto later=now+milliseconds(3100);
  require(!gap.observe(output(later),later),"unobserved scheduler gap counted as dwell");
  pupper_imu::StartupWindow sign;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);if(i%2)s.quat.coeffs()*=-1;require(sign.observe(s,t)==(i==300),"q/-q treated as rotation");}
  pupper_imu::StartupWindow spike;
  for(int i=0;i<=300;++i){auto t=now+milliseconds(i*10);auto s=output(t);if(i==250)s.gyro.x()=.06;require(!spike.observe(s,t),"gyro spike did not restart window");}
}
}
void init_spi(){command={};data={};}
spi_command_t *get_spi_command(){return &command;}
spi_data_t *get_spi_data(){return &data;}
void spi_driver_run(){sent.push_back(command);if(lose_during_enable && command.flags[0])mode=1;}
BNO055::BNO055(int,uint32_t){}
BNO055::~BNO055(){}
BNO055::Output BNO055::sample(){return output(std::chrono::steady_clock::now());}
extern "C" int __wrap_open(const char *p,int,...){require(std::strcmp(p,"/tmp/control_board_hardware.lock")==0,"unexpected open");return lock_fd;}
extern "C" int __wrap_open64(const char *p,int f,...){return __wrap_open(p,f);}
extern "C" int __wrap_flock(int fd,int){require(fd==lock_fd,"unexpected flock");return 0;}
extern "C" int __wrap_close(int fd){require(fd==lock_fd,"unexpected close");return 0;}
extern "C" int __wrap_unlink(const char *p){require(std::strcmp(p,"/tmp/control_board_hardware.lock")==0,"unexpected unlink");return 0;}
int main(int argc,char **argv){try{
  require(argc==2,"scenario required");std::string scenario=argv[1];
  if(scenario=="health"){unit_health();std::cout<<"PASS health\n";return 0;}
  auto driver=std::make_unique<control_board_hardware_interface::ControlBoardHardwareInterface>();
  hardware_interface::HardwareInfo info;info.name="imu_guard_fake";info.type="system";
  hardware_interface::ComponentInfo sensor;sensor.name="imu_sensor";sensor.parameters={{"roll","0"},{"pitch","0"},{"yaw","0"}};info.sensors.push_back(sensor);
  if(scenario=="init_missing")mode=5;
  const auto initialized=driver->on_init(info);
  if(scenario=="init_missing"){
    require(initialized==hardware_interface::CallbackReturn::ERROR,"missing IMU initialized");neutral_only();
  }else{
    require(initialized==hardware_interface::CallbackReturn::SUCCESS,"fresh init failed");require(sent.empty(),"init sent a command");
    if(scenario=="read_gyro"||scenario=="read_quat"||scenario=="read_norm"){
      mode=scenario=="read_gyro"?1:scenario=="read_quat"?2:4;
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
      for(int i=0;i<4;++i)command.flags[i]=1;
      command.kp_hip[0]=2;
      require(driver->read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01))==hardware_interface::return_type::ERROR,"invalid feedback accepted by read");neutral_only();
      if(scenario!="read_norm"){
        mode=scenario=="read_gyro"?6:7;std::this_thread::sleep_for(std::chrono::milliseconds(30));
        sent.clear();for(int i=0;i<4;++i)command.flags[i]=1;
        require(driver->read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.01))==hardware_interface::return_type::ERROR,"queued report allowed SPI command");neutral_only();
      }
    }else if(scenario=="activate_bias"){
      mode=3;std::this_thread::sleep_for(std::chrono::milliseconds(30));
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::ERROR,"biased IMU enabled");neutral_only();
    }else if(scenario=="homing_stale"){
      lose_during_enable=true;
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::ERROR,"lost IMU during homing accepted");
      require(!sent.empty() && sent.back().flags[0]==0,"homing fault did not disable");
    }else if(scenario=="activate_fresh"){
      const auto start=std::chrono::steady_clock::now();
      require(driver->on_activate(rclcpp_lifecycle::State())==hardware_interface::CallbackReturn::SUCCESS,"stationary startup rejected");
      require(std::chrono::steady_clock::now()-start>=pupper_imu::stationary_window,"startup skipped window");
      require(!sent.empty()&&sent.front().flags[0]==1,"fresh activation never enabled fake SPI");
    }else throw std::runtime_error("unknown scenario");
  }
  driver.reset();std::cout<<"PASS "<<scenario<<" fake_SPI_packets="<<sent.size()<<'\n';return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
