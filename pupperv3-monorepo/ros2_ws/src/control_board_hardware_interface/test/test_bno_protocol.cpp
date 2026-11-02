// Entirely offline: only pure header + captured/synthetic bytes, no device API.
#include "rt/bno_protocol.hpp"
#include "test_bno_raw_fixtures.hpp"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace pupper_bno;
void require(bool value, const char *message) { if (!value) throw std::runtime_error(message); }
struct Replay {
  std::vector<RawRead> reads;
  size_t cursor = 0;
  uint64_t clock = 0;
  int read(uint8_t *dst, size_t n) {
    require(cursor < reads.size(), "unexpected extra read");
    const auto &r = reads[cursor++];
    require(r.requested == n, "request differs from exact captured trace");
    require(r.bytes.size() <= n, "bad fixture");
    std::copy(r.bytes.begin(), r.bytes.end(), dst);
    clock += 10;
    return r.returned;
  }
  ReadResult run(Packet &packet, uint64_t budget=25000) {
    return read_packet(packet, [this](uint8_t *p, size_t n) { return read(p,n); },
                       [this]() { return clock; }, [this]() { clock += 1000; }, budget);
  }
};
std::vector<Report> fixture(std::vector<RawRead> raw, unsigned count, unsigned bytes) {
  Replay replay{std::move(raw)};
  Packet packet;
  require(replay.run(packet) == ReadResult::packet, "captured packet rejected");
  require(replay.cursor == replay.reads.size(), "not all exact captured reads consumed");
  require(packet.payload.size() == bytes, "payload not complete");
  std::vector<Report> reports;
  require(parse_reports(packet.payload, reports), "captured known batch rejected");
  require(reports.size() == count, "not all batched reports parsed");
  return reports;
}
int main(int argc, char **argv) {
  try {
    require(argc == 2, "scenario required"); const std::string mode = argv[1];
    if (mode == "normal") {
      const auto r=fixture(normal_gyro(),1,15);
      require(r[0].id==2 && r[0].sequence==2 && r[0].accuracy==3 && r[0].values[0]==-10,
              "gyro raw sign/identity changed");
    } else if (mode == "two") {
      const auto r=fixture(valid_two_reports(),2,29);
      require(r[0].id==2 && r[1].id==5 && r[1].rotation_accuracy==0x3244, "two report data lost");
    } else if (mode == "delayed_tail") {
      const auto r=fixture(zero_header_then_late_one_byte(),2,29);
      require(r[1].rotation_accuracy==0x3244, "zero padding was accepted instead of delayed byte");
    } else if (mode == "large") {
      const auto r=fixture(over_128_byte_sensor_packet(),13,163);
      require(r.back().id==5, "large packet tail lost");
    } else if (mode == "three") {
      const auto r=fixture(valid_three_reports(),3,39);
      require(r[0].id==2 && r[1].id==5 && r[2].id==2, "multiple sensor updates lost");
    } else if (mode == "probe2") {
      const auto r=fixture(probe2_first_batch(),2,29);
      require(r[0].id==2 && r[1].id==5, "probe2 batch changed");
    } else if (mode == "short_read") {
      auto raw=normal_gyro(); raw.back().returned-=1; raw.back().bytes.pop_back();
      Replay replay{raw}; Packet packet; packet.payload={1,2,3};
      require(replay.run(packet)==ReadResult::invalid && packet.payload.empty(), "short read reused state");
    } else if (mode == "bad_channel" || mode == "bad_sequence" || mode == "bad_length" || mode == "missing_continuation") {
      auto raw=normal_gyro();
      if (mode=="bad_channel")raw[1].bytes[2]=2;
      if (mode=="bad_sequence")raw[1].bytes[3]+=1;
      if (mode=="bad_length")raw[1].bytes[0]+=1;
      if (mode=="missing_continuation")raw[1].bytes[1]=0;
      Replay replay{raw};Packet packet;
      require(replay.run(packet)==ReadResult::invalid && packet.payload.empty(), "bad repeated header accepted");
    } else if (mode == "orphan") {
      auto raw=normal_gyro();raw[0].bytes[1]|=0x80;
      Replay replay{raw};Packet packet;
      require(replay.run(packet)==ReadResult::invalid && replay.cursor==2 && packet.payload.empty(), "orphan not drained/rejected");
    } else if (mode == "too_short" || mode == "too_large") {
      Replay replay{{{4,4, mode=="too_short" ? std::vector<uint8_t>{3,0,3,0} : std::vector<uint8_t>{5,16,3,0}}}};
      Packet packet;require(replay.run(packet)==ReadResult::invalid && replay.cursor==1, "invalid length accepted");
    } else if (mode == "empty") {
      Replay replay{{{4,4,{0,0,0,0}}}};Packet packet;
      require(replay.run(packet)==ReadResult::empty && packet.payload.empty(), "empty report invented");
    } else if (mode == "bounded_retry") {
      auto raw=normal_gyro();raw[1].bytes.assign(19,0);
      for (int i=0;i<30;++i)raw.push_back({4,4,{0,0,0,0}});
      Replay replay{raw};Packet packet;
      require(replay.run(packet,4000)==ReadResult::timed_out && replay.clock<5100 && packet.payload.empty(), "retry unbounded");
    } else if (mode == "atomic_truncated" || mode == "atomic_unknown") {
      Replay replay{valid_two_reports()};Packet packet;require(replay.run(packet)==ReadResult::packet,"fixture failed");
      if(mode=="atomic_truncated")packet.payload.pop_back(); else packet.payload.push_back(0xee);
      std::vector<Report> reports{{}};
      require(!parse_reports(packet.payload,reports) && reports.empty(),"partial batch escaped");
    } else if (mode == "signed_timing") {
      std::vector<uint8_t> bytes={0xfb,0xff,0xff,0xff,0xff, 0xfa,2,0,0,0, 2,1,0xff,0xff,0,0,0,0,0,0};
      std::vector<Report> reports;require(parse_reports(bytes,reports),"timing rejected");
      require(reports[0].reference_delta_ticks==3 && reports[0].delay_ticks==16383 && reports[0].accuracy==3,"timing sign/units bits wrong");
    } else if (mode == "unknown_time") {
      std::vector<uint8_t> bytes={0xfb,0xff,0xff,0xff,0x7f, 2,1,3,0,0,0,0,0,0,0};
      std::vector<Report> reports;require(parse_reports(bytes,reports) && !reports[0].timing_valid,"reserved delta called precise");
    } else if (mode == "no_base") {
      std::vector<Report> reports;require(!parse_reports({2,1,3,0,0,0,0,0,0,0},reports),"report without base accepted");
    } else if (mode == "game_and_stability") {
      std::vector<uint8_t> bytes={0xfb,0,0,0,0, 8,1,3,0,0,0,0,0,0,0,0,0x40, 19,2,3,0,3,0};
      std::vector<Report> r;require(parse_reports(bytes,r) && r.size()==2 && r[0].values[3]==16384 && r[1].stability==3,"known boundaries wrong");
    } else if (mode == "sequence") {
      ReportSequence q,g;
      require(q.accept(254) && q.accept(255) && q.accept(0),"sequence wrap rejected");
      require(!q.accept(0) && !q.accept(255),"duplicate/backward refreshed");
      require(g.accept(255) && q.accept(2) && g.last==255,"sensor sequences conflated");
    } else throw std::runtime_error("unknown scenario");
    std::cout << "PASS " << mode << '\n'; return 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
