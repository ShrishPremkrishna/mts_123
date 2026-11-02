// Real IMUManager with a link-time BNO055 fake: no I2C, SPI, ROS node, executor or device access.
#include "control_board_hardware_interface/imu_manager.hpp"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
std::atomic<int> instances{0}, samples{0}, in_sample{0}, destroyed_during_sample{0};
std::atomic<bool> invalid_sample{false};
std::mutex fake_mutex;
std::condition_variable fake_changed;
bool block_sample=false, release_sample=false;
void require(bool condition,const char* message) { if(!condition)throw std::runtime_error(message); }
void await_sample() {
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(samples.load()==0 && std::chrono::steady_clock::now()<deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  require(samples.load()>0,"sampler did not run");
}
}
// These three definitions intentionally replace rt_bno055.cpp in this test target.
BNO055::BNO055(int,uint32_t) { ++instances; }
BNO055::~BNO055() { if(in_sample.load()!=0)++destroyed_during_sample; --instances; }
BNO055::Output BNO055::sample() {
  ++in_sample; ++samples;
  {
    std::unique_lock<std::mutex> guard(fake_mutex);
    fake_changed.notify_all();
    if(block_sample)fake_changed.wait(guard,[]{return release_sample;});
  }
  Output output;
  output.quat=Eigen::Quaternionf::Identity();
  output.acc=Eigen::Vector3f::Zero(); output.gyro=Eigen::Vector3f::Zero();
  output.packet_timestamp=output.measurement_timestamp=std::chrono::high_resolution_clock::now();
  output.quaternion_valid=output.gyro_valid=true;
  output.quaternion_receipt_time=output.gyro_receipt_time=std::chrono::steady_clock::now();
  output.gyro_minimum_age_at_receipt=std::chrono::milliseconds(90);
  output.quaternion_minimum_age_at_receipt=std::chrono::seconds(78);
  if(invalid_sample.load()) { output.quaternion_valid=false; output.quat=Eigen::Quaternionf(0,0,0,0); }
  --in_sample;return output;
}
int main(int argc,char** argv) {
  try {
    require(argc==2,"one scenario required"); const std::string mode=argv[1];
    if(mode=="never_started") { IMUManager manager; }
    else if(mode=="invalid_after_good") {
      IMUManager manager;manager.begin(10000);
      const auto ready=std::chrono::steady_clock::now()+std::chrono::seconds(1);
      while(!manager.get_imu_data().quaternion_valid && std::chrono::steady_clock::now()<ready)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      const auto previous=manager.get_imu_data();require(previous.quaternion_valid,"no initial valid sample");
      require(previous.gyro_minimum_age_at_receipt==std::chrono::milliseconds(90)&&previous.quaternion_minimum_age_at_receipt==std::chrono::seconds(78),"manager lost component age bounds");
      invalid_sample=true;
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
      while(manager.get_imu_data().quaternion_valid && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      const auto failed=manager.get_imu_data();
      require(!failed.quaternion_valid&&!failed.gyro_valid,"sampling failure retained valid cached flags");
      require(failed.quaternion_receipt_time>=previous.quaternion_receipt_time,"receipt moved backwards");
      require(failed.gyro_minimum_age_at_receipt==previous.gyro_minimum_age_at_receipt&&failed.quaternion_minimum_age_at_receipt==previous.quaternion_minimum_age_at_receipt,"manager invalidation erased age evidence");
      invalid_sample=false;
      const auto recovery=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
      while(!manager.get_imu_data().quaternion_valid && std::chrono::steady_clock::now()<recovery)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      require(manager.get_imu_data().quaternion_valid,"valid sampling did not recover");
      manager.stop();
    } else if(mode=="implicit_destroy") {
      IMUManager manager; manager.begin(10000); await_sample();
      // Scope exit must join its live thread even without an explicit stop call.
    } else if(mode=="explicit_stop_twice") {
      IMUManager manager; manager.begin(10000); await_sample();manager.stop();manager.stop();
      const int stopped=samples.load();std::this_thread::sleep_for(std::chrono::milliseconds(5));
      require(samples.load()==stopped,"polling continued after stop returned");
    } else if(mode=="restart_after_stop") {
      IMUManager manager;manager.begin(10000);await_sample();manager.stop();
      const int stopped=samples.load();manager.begin(10000);
      const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
      while(samples.load()==stopped && std::chrono::steady_clock::now()<deadline)std::this_thread::yield();
      require(samples.load()>stopped,"restart did not sample");manager.stop();
    } else if(mode=="destroy_during_sample") {
      block_sample=true;
      auto manager=std::make_unique<IMUManager>();manager->begin(10000);
      {
        std::unique_lock<std::mutex> guard(fake_mutex);
        require(fake_changed.wait_for(guard,std::chrono::seconds(2),[]{return in_sample.load()>0;}),"sample did not enter");
      }
      std::thread release([]{
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        std::lock_guard<std::mutex> guard(fake_mutex);release_sample=true;fake_changed.notify_all();
      });
      manager.reset(); // Must wait for the in-flight sample before destroying BNO055.
      release.join();
    } else throw std::runtime_error("unknown scenario");
    require(instances.load()==0,"sensor still alive after manager destruction");
    require(in_sample.load()==0,"sample still executing after manager destruction");
    require(destroyed_during_sample.load()==0,"sensor destroyed while sampling");
    std::cout<<"PASS "<<mode<<" samples="<<samples.load()<<'\n';return 0;
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
