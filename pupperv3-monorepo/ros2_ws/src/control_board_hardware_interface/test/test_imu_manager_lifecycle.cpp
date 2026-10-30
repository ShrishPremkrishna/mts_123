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
  --in_sample;return output;
}
int main(int argc,char** argv) {
  try {
    require(argc==2,"one scenario required"); const std::string mode=argv[1];
    if(mode=="never_started") { IMUManager manager; }
    else if(mode=="implicit_destroy") {
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
