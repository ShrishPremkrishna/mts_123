// Isolated SH-2 diagnostic. No BNO constructor, ROS, SPI or motor symbols.
#include "diagnostic_protocol.hpp"
#include <chrono>
#include <csignal>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/i2c-dev.h>
#else
static constexpr unsigned long I2C_SLAVE=0x0703;
#endif
static constexpr int sensor_address=0x4b;
static volatile std::sig_atomic_t stopped=0;
static void signal_stop(int s){stopped=s;}
static double monotonic_s(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
static double wall_s(){return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();}
static std::string hex(const uint8_t*p,size_t n){static const char*h="0123456789abcdef";std::string s;for(size_t i=0;i<n;++i){s+=h[p[i]>>4];s+=h[p[i]&15];}return s;}
static std::string hex(const diagnostic::Bytes&p){return hex(p.data(),p.size());}
static std::string quote(const std::string&s){std::string r="\"";for(unsigned char c:s){if(c=='"'||c=='\\'){r+='\\';r+=char(c);}else if(c<32){r+=' ';}else{r+=char(c);}}return r+'"';}
static const char* read_name(pupper_bno::ReadResult r){switch(r){case pupper_bno::ReadResult::packet:return "packet";case pupper_bno::ReadResult::empty:return "empty";case pupper_bno::ReadResult::invalid:return "invalid";case pupper_bno::ReadResult::timed_out:return "timed_out";}return "unknown";}
struct IO {
 std::function<ssize_t(uint8_t*,size_t)> read;
 std::function<ssize_t(const uint8_t*,size_t)> write;
 std::function<double()> now;
 std::function<void()> wait;
};
struct Capture {
 IO io;std::ostream& reads;std::ostream& writes;std::ostream& packets;std::ostream& state;
 uint8_t txseq=0,commandseq=0;size_t read_index=0,attempt=0,products=0;
 bool permit_set=false,restoring=false,write_uncertain=false,logging_failed=false;
 double deadline=0,hard_deadline=0,last_control=-1;std::map<uint8_t,diagnostic::Feature> observed,original;
 std::set<uint8_t> seen_command_sequences;std::vector<uint8_t> dirty;std::vector<diagnostic::Bytes> controls;
 Capture(IO in,std::ostream&r,std::ostream&w,std::ostream&p,std::ostream&s):io(std::move(in)),reads(r),writes(w),packets(p),state(s){}
 void event(const std::string&name,const std::string&extra=""){
  try{state<<"{\"event\":"<<quote(name)<<",\"monotonic_s\":"<<io.now()<<",\"wall_s\":"<<wall_s()<<extra<<"}\n";state.flush();}
  catch(...){logging_failed=true;if(!restoring)throw;}
 }
 void check(){if(io.now()>=deadline)throw std::runtime_error("overall deadline reached");if(stopped&&!restoring)throw std::runtime_error("interrupted");}
 void send(const diagnostic::Bytes&p){
  check();if(!diagnostic::allowed_request(p,permit_set))throw std::runtime_error("request denied by diagnostic allowlist");
  diagnostic::Bytes wire{uint8_t(p.size()+4),0,2,txseq++};wire.insert(wire.end(),p.begin(),p.end());
  event("write_intent",",\"payload_hex\":"+quote(hex(p))+",\"restoring\":"+(restoring?"true":"false"));
  const double start=io.now();errno=0;const ssize_t n=io.write(wire.data(),wire.size());const int e=errno;
  try{writes<<"{\"monotonic_start_s\":"<<start<<",\"monotonic_end_s\":"<<io.now()<<",\"wall_s\":"<<wall_s()<<",\"requested\":"<<wire.size()<<",\"returned\":"<<n<<",\"errno\":"<<(n<0?e:0)<<",\"wire_hex\":"<<quote(hex(wire))<<",\"payload_hex\":"<<quote(hex(p))<<"}\n";writes.flush();}catch(...){logging_failed=true;if(!restoring)throw;}
  if(n!=static_cast<ssize_t>(wire.size())){write_uncertain=true;throw std::runtime_error("short or failed I2C write; delivery uncertain");}
 }
 void poll(){
  check();const double start=io.now(),wall=wall_s();const size_t first=read_index;
  pupper_bno::Packet packet;
  auto result=pupper_bno::read_packet(packet,[&](uint8_t*p,size_t n)->ssize_t{
   if((stopped&&!restoring)||io.now()>=deadline)return -1;
   double before=io.now();errno=0;ssize_t got=io.read(p,n);int e=errno;
   try{reads<<"{\"index\":"<<read_index++<<",\"monotonic_start_s\":"<<before<<",\"monotonic_end_s\":"<<io.now()<<",\"wall_s\":"<<wall_s()<<",\"requested\":"<<n<<",\"returned\":"<<got<<",\"errno\":"<<(got<0?e:0)<<",\"hex\":"<<quote(got>0?hex(p,size_t(got)):"")<<"}\n";}catch(...){logging_failed=true;if(!restoring)throw;}
   return got;
  },[&](){return uint64_t(io.now()*1e6);},io.wait);
  std::ostringstream line;line<<std::setprecision(17)<<"{\"attempt\":"<<attempt++<<",\"monotonic_start_s\":"<<start<<",\"monotonic_end_s\":"<<io.now()<<",\"wall_start_s\":"<<wall<<",\"first_read\":"<<first<<",\"read_count\":"<<read_index-first<<",\"result\":"<<quote(read_name(result));
  if(result==pupper_bno::ReadResult::packet){
   line<<",\"header_hex\":"<<quote(hex(packet.header.data(),4))<<",\"channel\":"<<unsigned(packet.header[2])<<",\"payload_hex\":"<<quote(hex(packet.payload));
   if(packet.header[2]==2){
    last_control=io.now();std::vector<diagnostic::Bytes> records;bool ok=diagnostic::control_records(packet.payload,records);line<<",\"control_parse_ok\":"<<(ok?"true":"false");
    if(ok){for(auto&record:records){controls.push_back(record);if(record[0]==0xf1){seen_command_sequences.insert(record[3]);}if(record[0]==0xfc&&diagnostic::monitored_id(record[1]))observed[record[1]]=diagnostic::feature_response(record);}}
   }
   if(packet.header[2]==3){
    std::vector<diagnostic::Report> records;bool ok=diagnostic::reports(packet.payload,records);line<<",\"parse_ok\":"<<(ok?"true":"false")<<",\"reports\":[";bool comma=false;
    for(const auto&r:records){if(comma){line<<',';}comma=true;
     line<<"{\"id\":"<<unsigned(r.id)<<",\"sequence\":"<<unsigned(r.sequence)<<",\"status_raw\":"<<unsigned(r.status_raw)<<",\"accuracy\":"<<unsigned(r.accuracy)<<",\"delay_ticks\":"<<r.delay_ticks<<",\"reference_delta_ticks\":"<<r.reference_delta_ticks<<",\"timing_valid\":"<<(r.timing_valid?"true":"false")<<",\"relative_ticks\":"<<r.reference_delta_ticks+r.delay_ticks<<",\"values_raw\":[";
     for(unsigned k=0;k<6;++k){if(k){line<<',';}line<<r.values[k];}line<<']';
     if(r.id==2||r.id==7){line<<",\"gyro_sensor_rad_s\":["<<r.values[0]/512.<<','<<r.values[1]/512.<<','<<r.values[2]/512.<<']';}
     if(r.id==7){line<<",\"estimated_bias_rad_s\":["<<r.values[3]/512.<<','<<r.values[4]/512.<<','<<r.values[5]/512.<<']';}
     if(r.id==5||r.id==8){line<<",\"quaternion_sensor_xyzw\":["<<r.values[0]/16384.<<','<<r.values[1]/16384.<<','<<r.values[2]/16384.<<','<<r.values[3]/16384.<<']';}
     if(r.id==5){const int signed_accuracy=r.rotation_accuracy>=32768?int(r.rotation_accuracy)-65536:int(r.rotation_accuracy);line<<",\"rotation_accuracy_raw\":"<<r.rotation_accuracy<<",\"rotation_accuracy_q12_signed\":"<<signed_accuracy<<",\"rotation_accuracy_rad\":";if(signed_accuracy<0){line<<"null";}else{line<<signed_accuracy/4096.;}}
     if(r.id==0x13){line<<",\"stability\":"<<unsigned(r.stability);}line<<'}';
    }line<<']';
   }
  }
  line<<"}\n";try{packets<<line.str();if(attempt%100==0){packets.flush();reads.flush();}}catch(...){logging_failed=true;if(!restoring)throw;}
  io.wait();
 }
 void drain_controls(){
  const double start=io.now(),until=std::min(deadline,start+1.0);last_control=start;
  while(io.now()<until&&io.now()-last_control<.2){poll();}
  controls.clear();if(io.now()-last_control<.2)throw std::runtime_error("control channel did not become quiet; ownership unknown");
 }
 std::vector<diagnostic::Bytes> query(const diagnostic::Bytes&request,const std::function<bool(const diagnostic::Bytes&)>&match,double seconds=.8,bool collect=false){
  controls.clear();send(request);const double until=std::min(deadline,io.now()+seconds);std::vector<diagnostic::Bytes> found;
  while(io.now()<until){poll();for(const auto&r:controls){if(match(r))found.push_back(r);}controls.clear();if(!found.empty()&&!collect)break;}
  if(found.empty()) { throw std::runtime_error("required readback missing or unmatched"); }
  return found;
 }
 diagnostic::Feature feature(uint8_t id){
  auto x=query(diagnostic::feature_query(id),[&](const diagnostic::Bytes&r){return r.size()==17&&r[0]==0xfc&&r[1]==id;});
  auto f=diagnostic::feature_response(x.back());event("feature_readback",",\"sensor_id\":"+std::to_string(id)+",\"raw_hex\":"+quote(hex(x.back())));return f;
 }
 void inspect(){
  drain_controls();auto p=query(diagnostic::product_query(),[](const diagnostic::Bytes&r){return r.size()==16&&r[0]==0xf8;},1.,true);
  for(const auto&r:p){++products;event("product_record",",\"raw_hex\":"+quote(hex(r))+",\"software_major\":"+std::to_string(r[2])+",\"software_minor\":"+std::to_string(r[3])+",\"software_patch\":"+std::to_string(pupper_bno::u16(r.data()+12))+",\"software_part_number\":"+std::to_string(diagnostic::u32(r.data()+4))+",\"software_build_number\":"+std::to_string(diagnostic::u32(r.data()+8)));}
  commandseq=uint8_t(uint64_t(io.now()*1e6));while(seen_command_sequences.count(commandseq)){++commandseq;if(seen_command_sequences.size()==256)throw std::runtime_error("no unused command sequence");}
  const uint8_t seq=commandseq++;auto c=query(diagnostic::calibration_query(seq),[&](const diagnostic::Bytes&r){return r.size()==16&&r[0]==0xf1&&r[2]==7&&r[3]==seq;});const auto&r=c.back();
  event("calibration_readback",",\"raw_hex\":"+quote(hex(r))+",\"command_sequence\":"+std::to_string(seq)+",\"status\":"+std::to_string(r[5])+",\"accel\":"+std::to_string(r[6])+",\"gyro\":"+std::to_string(r[7])+",\"mag\":"+std::to_string(r[8])+",\"planar\":"+std::to_string(r[9])+",\"on_table\":"+std::to_string(r[10]));
  if(r[5]!=0)throw std::runtime_error("calibration readback returned failure");
  for(uint8_t id:diagnostic::monitored){original[id]=feature(id);}
 }
 void await_feature(const diagnostic::Feature&expected){
  const double until=std::min(deadline,io.now()+1.);double next_query=io.now();controls.clear();
  while(io.now()<until){
   if(io.now()>=next_query){send(diagnostic::feature_query(expected.id()));next_query=io.now()+.02;}
   poll();bool seen=false,matched=false;
   for(const auto&r:controls){if(r.size()==17&&r[0]==0xfc&&r[1]==expected.id()){
    auto actual=diagnostic::feature_response(r);seen=true;matched=(actual==expected);
    event("feature_readback",",\"sensor_id\":"+std::to_string(expected.id())+",\"raw_hex\":"+quote(hex(r)));
    if(!matched)event("feature_wait_mismatch",",\"sensor_id\":"+std::to_string(expected.id())+",\"raw_hex\":"+quote(hex(r))+",\"expected_set_hex\":"+quote(hex(expected.set_request())));
   }}
   controls.clear();if(seen&&matched)return;
  }
  throw std::runtime_error("effective feature did not match within bounded settling wait");
 }
 void set_feature(const diagnostic::Feature&f){
  if(std::find(dirty.begin(),dirty.end(),f.id())==dirty.end())dirty.push_back(f.id()); // Before the potentially partial write.
  send(f.set_request());await_feature(f);
 }
 bool restore(){
  if(dirty.empty()) { return true; }
  restoring=true;deadline=hard_deadline;bool okay=true;event("restore_start");
  // Reverse direct changes first; later verify every monitored field for coupling.
  for(auto i=dirty.rbegin();i!=dirty.rend();++i){try{send(original.at(*i).set_request());await_feature(original.at(*i));}catch(const std::exception&e){okay=false;event("restore_write_failed",",\"message\":"+quote(e.what()));}}
  std::map<uint8_t,diagnostic::Feature> checkback;
  for(uint8_t id:diagnostic::monitored){try{checkback[id]=feature(id);}catch(const std::exception&e){okay=false;event("restore_readback_failed",",\"sensor_id\":"+std::to_string(id)+",\"message\":"+quote(e.what()));}}
  // One bounded correction for proven coupled changes. Never invent unavailable originals.
  bool corrections=false;
  for(auto i=diagnostic::monitored.rbegin();i!=diagnostic::monitored.rend();++i){auto k=checkback.find(*i);if(std::find(dirty.begin(),dirty.end(),*i)==dirty.end()&&k!=checkback.end()&&!(k->second==original.at(*i))){corrections=true;try{send(original.at(*i).set_request());await_feature(original.at(*i));}catch(...){okay=false;}}}
  if(corrections){checkback.clear();for(uint8_t id:diagnostic::monitored){try{checkback[id]=feature(id);}catch(...){okay=false;}}}
  for(uint8_t id:diagnostic::monitored){auto k=checkback.find(id);if(k==checkback.end()||!(k->second==original.at(id)))okay=false;}
  // A first matching response can precede asynchronous feature application.
  // Observe a later complete round; this never resubmits a Set Feature command.
  const double settle_until=std::min(deadline,io.now()+.02);while(io.now()<settle_until){poll();controls.clear();}
  for(uint8_t id:diagnostic::monitored){try{await_feature(original.at(id));}catch(const std::exception&e){okay=false;event("restore_late_readback_failed",",\"sensor_id\":"+std::to_string(id)+",\"message\":"+quote(e.what()));}}
  event("restore_finished",",\"verified\":"+std::string(okay?"true":"false"));return okay;
 }
 void capture_phase(const std::string&phase,double seconds,const std::map<uint8_t,diagnostic::Feature>&expected){
  for(uint8_t id:diagnostic::monitored){if(!(feature(id)==expected.at(id)))throw std::runtime_error("related feature changed; capture refused");}
  if(io.now()+seconds>deadline)throw std::runtime_error("insufficient time reserved for restoration");
  event("capture_start",",\"phase\":"+quote(phase)+",\"seconds\":"+std::to_string(seconds));const double until=io.now()+seconds;
  while(io.now()<until){poll();controls.clear();for(uint8_t id:diagnostic::monitored){if(!(observed.at(id)==expected.at(id)))throw std::runtime_error("effective configuration changed during capture");}}
  event("capture_finished",",\"phase\":"+quote(phase));
 }
 int run(bool compare,const std::vector<uint8_t>&extras,double seconds,bool enable_baseline=false){
  const double baseline_seconds=enable_baseline?30.:0.;
  permit_set=compare;const double start=io.now();hard_deadline=start+(compare?seconds+baseline_seconds+(enable_baseline?40.:30.):12.);deadline=hard_deadline-(compare?12.:0.);bool complete=false,restored=true;std::string error;
  try{
   if(enable_baseline&&!compare)throw std::runtime_error("baseline enable is allowed only in explicit compare mode");
   inspect();event("inspection_complete",",\"product_set_complete\":null,\"note\":\"Product records collected for bounded1second; no transaction count proof.\"");
   if(compare){
    auto expected=original;
    if(enable_baseline){
     for(uint8_t id:diagnostic::monitored){if(original.at(id).interval_us())throw std::runtime_error("baseline enable requires all monitored reports initially disabled");}
     event("baseline_enable",",\"baseline_seconds\":30,\"interval_us\":10000");
     for(uint8_t id:std::array<uint8_t,2>{{2,5}}){
      auto target=original.at(id);for(unsigned k=0;k<4;++k)target.fields[4+k]=uint8_t(10000u>>(8*k));
      set_feature(target);expected[id]=target;
     }
     capture_phase("baseline_02_05",baseline_seconds,expected);
    }else if(!original.at(2).interval_us()||!original.at(5).interval_us()){
     throw std::runtime_error("compare requires preexisting enabled02 and05; will not enable them without --enable-baseline");
    }
    for(uint8_t id:extras){auto target=diagnostic::diagnostic_feature(id);set_feature(target);expected[id]=target;}
    capture_phase("comparison",seconds,expected);
   }
   complete=true;
  }catch(const std::exception&e){error=e.what();try{event("operation_failed",",\"message\":"+quote(error));}catch(...){}}
  try{restored=restore();}catch(const std::exception&e){restored=false;error+="; restoration: ";error+=e.what();}
  int code=stopped?128+stopped:(!complete||!restored||logging_failed?4:0);
  try{event("finished",",\"complete\":"+std::string(complete?"true":"false")+",\"exit_code\":"+std::to_string(code)+",\"signal\":"+std::to_string(stopped)+",\"restoration_required\":"+(dirty.empty()?"false":"true")+",\"restoration_verified\":"+(restored?"true":"false")+",\"restoration_uncertain\":"+(restored?"false":"true")+",\"write_delivery_uncertain\":"+(write_uncertain?"true":"false")+",\"logging_failed\":"+(logging_failed?"true":"false")+",\"error\":"+quote(error)+",\"elapsed_s\":"+std::to_string(io.now()-start));}catch(...){code=4;}
  return code;
 }
};
#ifndef DIAGNOSTIC_NO_MAIN
// Exact read-only fuser check before opening the device. Advisory locks are not
// relied on. A permission/error warning or nonstandard result is unknown.
static bool exclusive_check(const std::string&dir){
#ifndef __linux__
 (void)dir;std::cerr<<"Real-device diagnostics are Linux-only; use offline tests on this host.\n";return false;
#else
 if(::geteuid()!=0){std::cerr<<"Root access required for a complete owner check; refusing unknown ownership.\n";return false;}
 const std::string output=dir+"/owners.stdout",error=dir+"/owners.stderr";
 const pid_t pid=::fork();if(pid<0)return false;
 if(pid==0){
  int out=::open(output.c_str(),O_CREAT|O_EXCL|O_WRONLY,0600),err=::open(error.c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
  if(out<0||err<0)_exit(126);
  if(::dup2(out,STDOUT_FILENO)<0||::dup2(err,STDERR_FILENO)<0)_exit(126);
  ::close(out);::close(err);::execl("/usr/bin/fuser","fuser","/dev/i2c-1",static_cast<char*>(nullptr));_exit(127);
 }
 int status=0;bool finished=false;const double until=monotonic_s()+3.;
 while(!stopped&&monotonic_s()<until){pid_t got=::waitpid(pid,&status,WNOHANG);if(got==pid){finished=true;break;}if(got<0&&errno!=EINTR)break;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
 if(!finished){::kill(pid,SIGKILL);while(::waitpid(pid,&status,0)<0&&errno==EINTR){}return false;}
 std::ifstream eo(error),oo(output);std::string es((std::istreambuf_iterator<char>(eo)),{}),os((std::istreambuf_iterator<char>(oo)),{});
 return WIFEXITED(status)&&WEXITSTATUS(status)==1&&es.empty()&&os.empty();
#endif
}
int main(int argc,char**argv){
 if(argc<4){std::cerr<<"usage: bno_diagnostic inspect NEWDIR --device-inactive\n       bno_diagnostic compare NEWDIR --device-inactive --enable=07,08,13 --seconds=30 [--enable-baseline]\n";return 2;}
 const bool compare=std::string(argv[1])=="compare";
 if(!compare&&std::string(argv[1])!="inspect")return 2;
 bool acknowledged=false,have_seconds=false,have_enable=false,enable_baseline=false;double seconds=0;std::vector<uint8_t> extras;
 for(int i=3;i<argc;++i){std::string arg=argv[i];
  if(arg=="--device-inactive"&&!acknowledged){acknowledged=true;}
  else if(compare&&arg=="--enable-baseline"&&!enable_baseline){enable_baseline=true;}
  else if(compare&&arg.rfind("--enable=",0)==0&&!have_enable){
   have_enable=true;std::stringstream ss(arg.substr(9));std::string token;std::set<uint8_t> unique;
   while(std::getline(ss,token,',')){uint8_t id=token=="07"?7:token=="08"?8:token=="13"?0x13:0;if(!id||!unique.insert(id).second)return 2;extras.push_back(id);}
   if(extras.empty()||arg.back()==',')return 2;
  }else if(compare&&arg.rfind("--seconds=",0)==0&&!have_seconds){
   have_seconds=true;char*end=nullptr;const std::string n=arg.substr(10);seconds=std::strtod(n.c_str(),&end);
   if(!end||end==n.c_str()||*end||!std::isfinite(seconds)||seconds<1||seconds>60)return 2;
  }else{return 2;}
 }
 if(!acknowledged||(compare&&(!have_enable||!have_seconds)))return 2;
 if(::mkdir(argv[2],0700)!=0){std::cerr<<"Exclusive new output directory required.\n";return 2;}
 const std::string dir=argv[2];std::ofstream reads(dir+"/reads.jsonl"),writes(dir+"/writes.jsonl"),packets(dir+"/packets.jsonl"),state(dir+"/state.jsonl");
 if(!reads||!writes||!packets||!state)return 2;
 for(auto*s:{&reads,&writes,&packets,&state}){s->exceptions(std::ios::failbit|std::ios::badbit);*s<<std::setprecision(17);}
 stopped=0;std::signal(SIGINT,signal_stop);std::signal(SIGTERM,signal_stop);
 try{
  state<<"{\"event\":\"start\",\"wall_s\":"<<wall_s()<<",\"mode\":"<<quote(argv[1])<<",\"address\":75,\"operator_device_inactive_acknowledged\":true,\"reset\":false,\"calibration_setting\":false,\"dcd_command\":false,\"sensor_report_features_may_change\":"<<(compare?"true":"false")<<",\"baseline_enable_requested\":"<<(enable_baseline?"true":"false")<<",\"kernel_io_deadline_guaranteed\":false}\n";state.flush();
  if(!exclusive_check(dir)){state<<"{\"event\":\"owner_check_failed_or_unknown\",\"exit_code\":3}\n";return 3;}
  // Readback requires writing queries, soO_RDWR; no sensor-reset constructor.
  const int fd=::open("/dev/i2c-1",O_RDWR|O_CLOEXEC);if(fd<0){state<<"{\"event\":\"open_failed\",\"errno\":"<<errno<<"}\n";return 3;}
  struct Close{int fd;~Close(){::close(fd);}}closer{fd};
  if(::ioctl(fd,I2C_SLAVE,sensor_address)<0){state<<"{\"event\":\"address_selection_failed\",\"errno\":"<<errno<<"}\n";return 3;}
  IO io{[fd](uint8_t*p,size_t n){return ::read(fd,p,n);},[fd](const uint8_t*p,size_t n){return ::write(fd,p,n);},monotonic_s,[]{std::this_thread::sleep_for(std::chrono::milliseconds(1));}};
  Capture capture(io,reads,writes,packets,state);return capture.run(compare,extras,seconds,enable_baseline);
 }catch(const std::exception&e){std::cerr<<"Diagnostic failed: "<<e.what()<<"; inspect state and restoration records.\n";return 4;}
}
#endif
