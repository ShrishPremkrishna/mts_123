#define DIAGNOSTIC_NO_MAIN
#include "diagnostic.cpp"
#include <cassert>
#include <deque>
#include <iterator>
using diagnostic::Bytes;
static Bytes bytes(const std::string&s){Bytes p;for(size_t i=0;i<s.size();i+=2)p.push_back(uint8_t(std::stoul(s.substr(i,2),nullptr,16)));return p;}
static unsigned checks=0;
static void expect(bool value,const char*name){if(!value){std::cerr<<"FAILED "<<name<<'\n';std::abort();}++checks;}
struct Fake {
 double time=100.;uint8_t sequence=0,channel=2;std::deque<Bytes> queue;Bytes current;size_t cursor=0;bool header_read=false;
 std::map<uint8_t,diagnostic::Feature> features,before;std::vector<Bytes> sent;
 int absent_id=-1;bool wrong_sequence=false,cal_failure=false,short_set=false,interrupt_set=false,operation_timeout=false,hard_timeout=false,fail_restore=false,coupling=false,log_failure=false;
 unsigned set_count=0;std::ostream*broken=nullptr;
 Fake(){for(auto id:diagnostic::monitored){Bytes p(17,0);p[0]=0xfc;p[1]=id;if(id==2||id==5)diagnostic::put32(p,5,10000);features[id]=diagnostic::feature_response(p);}features[7]=diagnostic::feature_response(bytes("fc070d0201204e000050c3000078563412"));before=features;}
 ssize_t read(uint8_t*p,size_t n){
  time+=.0001;if(current.empty()&&!queue.empty()){current=queue.front();queue.pop_front();cursor=0;header_read=false;}
  std::fill(p,p+n,0);if(current.empty())return ssize_t(n);
  const size_t remaining=current.size()-cursor;size_t len=remaining+4;p[0]=uint8_t(len);p[1]=uint8_t(len>>8);p[2]=channel;
  if(n==4&&!header_read){p[3]=sequence;header_read=true;return 4;}
  assert(header_read&&n>=4&&n-4<=remaining);p[1]|=0x80;p[3]=++sequence;std::copy(current.begin()+cursor,current.begin()+cursor+n-4,p+4);cursor+=n-4;
  if(cursor==current.size()){current.clear();header_read=false;++sequence;}
  return ssize_t(n);
 }
 ssize_t write(const uint8_t*p,size_t n){
  time+=.0001;assert(n>=6&&p[2]==2);Bytes z(p+4,p+n);sent.push_back(z);
  if(z[0]==0xf9){queue.push_back(bytes("f800010a96a49800940100000a000000f8000507a7a698003f01000011000000f8000503b5a69800e20000000e000000"));}
  else if(z[0]==0xf2){Bytes r=bytes("f1000700000001000100010000000000");r[3]=uint8_t(z[1]+(wrong_sequence?1:0));r[5]=cal_failure?1:0;queue.push_back(r);}
  else if(z[0]==0xfe){if(z[1]!=absent_id){auto r=features.at(z[1]).set_request();r[0]=0xfc;queue.push_back(r);}}
  else if(z[0]==0xfd){
   ++set_count;if(fail_restore&&set_count>1){errno=EIO;return -1;}
   auto r=z;r[0]=0xfc;features[z[1]]=diagnostic::feature_response(r);
   if(set_count==1){
    if(coupling){auto f=features[2].set_request();f[0]=0xfc;diagnostic::put32(f,5,5000);features[2]=diagnostic::feature_response(f);queue.push_back(f);}
    if(interrupt_set)stopped=SIGTERM;
    if(operation_timeout)time=119.1;
    if(hard_timeout)time=132.;
    if(log_failure&&broken)broken->setstate(std::ios::badbit);
    if(short_set)return ssize_t(n-1);
   }
  }else{std::abort();}
  return ssize_t(n);
 }
};
static void lifecycle(){
 auto run=[](const char*name,const std::function<void(Fake&)>&setup,bool compare,bool should_succeed,bool restored){
  Fake f;setup(f);std::ostringstream reads,writes,packets,state;for(auto*s:{&reads,&writes,&packets,&state})s->exceptions(std::ios::badbit|std::ios::failbit);f.broken=&packets;
  IO io{[&](uint8_t*p,size_t n){return f.read(p,n);},[&](const uint8_t*p,size_t n){return f.write(p,n);},[&](){return f.time;},[&](){f.time+=.005;}};
  stopped=0;Capture c(io,reads,writes,packets,state);int code=c.run(compare,{7,8,0x13},1.);
  expect((code==0)==should_succeed,name);if(restored)expect(f.features==f.before,"full original configuration preserved");
  expect(state.str().find("\"restoration_uncertain\":"+std::string(restored?"false":"true"))!=std::string::npos,"restoration result honest");
  for(auto&r:f.sent)expect(diagnostic::allowed_request(r,compare),"only allowed query/feature bytes");
  if(!compare)expect(f.set_count==0,"inspect never changes features");
  if(f.absent_id>=0||f.wrong_sequence||f.cal_failure)expect(f.set_count==0,"missing invalid preflight never mutates");
  if(f.operation_timeout)expect(f.time<131.&&f.set_count>1,"reserved cleanup remains after operation expiry");
  if(f.interrupt_set)expect(code==143&&f.set_count>1,"SIGTERM restores before143exit");
  std::cout<<name<<": PASS\n";stopped=0;
 };
 run("inspect queries only",[](Fake&){},false,true,true);
 run("compare restores all fields",[](Fake&){},true,true,true);
 run("unavailable FC is unknown",[](Fake&f){f.absent_id=7;},true,false,true);
 run("wrong command sequence refused",[](Fake&f){f.wrong_sequence=true;},true,false,true);
 run("ME failure refused",[](Fake&f){f.cal_failure=true;},true,false,true);
 run("partial set delivery restored",[](Fake&f){f.short_set=true;},true,false,true);
 run("signal after dirty write restored",[](Fake&f){f.interrupt_set=true;},true,false,true);
 run("operation deadline reserves cleanup",[](Fake&f){f.operation_timeout=true;},true,false,true);
 run("hard deadline restoration uncertain",[](Fake&f){f.hard_timeout=true;},true,false,false);
 run("failed restore remains uncertain",[](Fake&f){f.short_set=true;f.fail_restore=true;},true,false,false);
 run("coupled original feature restored",[](Fake&f){f.coupling=true;},true,false,true);
 run("log failure after dirty write restores",[](Fake&f){f.log_failure=true;},true,false,true);
}
#include "generated_fixture_checks.hpp"
int main(){(void)&monotonic_s;(void)&signal_stop;fixture_checks();lifecycle();std::cout<<"checks="<<checks<<"\n";}
