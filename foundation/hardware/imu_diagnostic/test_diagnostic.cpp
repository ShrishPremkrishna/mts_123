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
 std::map<uint8_t,diagnostic::Feature> features,before;std::vector<Bytes> sent;std::vector<double> sent_times;
 int absent_id=-1;bool wrong_sequence=false,cal_failure=false,short_set=false,interrupt_set=false,operation_timeout=false,hard_timeout=false,fail_restore=false,coupling=false,log_failure=false;
 unsigned set_count=0,short_set_at=0,interrupt_set_at=0;std::ostream*broken=nullptr;
 double feature_delay=0.;bool ignore_enable=false,late_restore_diverge=false,signal_while_wait=false;
 std::vector<std::pair<double,diagnostic::Feature>> pending_features;
 void apply_due(){for(auto i=pending_features.begin();i!=pending_features.end();){if(time>=i->first){features[i->second.id()]=i->second;i=pending_features.erase(i);}else{++i;}}}
 Fake(){for(auto id:diagnostic::monitored){Bytes p(17,0);p[0]=0xfc;p[1]=id;if(id==2||id==5)diagnostic::put32(p,5,10000);features[id]=diagnostic::feature_response(p);}features[7]=diagnostic::feature_response(bytes("fc070d0201204e000050c3000078563412"));before=features;}
 ssize_t read(uint8_t*p,size_t n){
  time+=.0001;apply_due();if(signal_while_wait&&set_count==1)stopped=SIGTERM;if(current.empty()&&!queue.empty()){current=queue.front();queue.pop_front();cursor=0;header_read=false;}
  std::fill(p,p+n,0);if(current.empty())return ssize_t(n);
  const size_t remaining=current.size()-cursor;size_t len=remaining+4;p[0]=uint8_t(len);p[1]=uint8_t(len>>8);p[2]=channel;
  if(n==4&&!header_read){p[3]=sequence;header_read=true;return 4;}
  assert(header_read&&n>=4&&n-4<=remaining);p[1]|=0x80;p[3]=++sequence;std::copy(current.begin()+cursor,current.begin()+cursor+n-4,p+4);cursor+=n-4;
  if(cursor==current.size()){current.clear();header_read=false;++sequence;}
  return ssize_t(n);
 }
 ssize_t write(const uint8_t*p,size_t n){
  time+=.0001;apply_due();assert(n>=6&&p[2]==2);Bytes z(p+4,p+n);sent.push_back(z);sent_times.push_back(time);
  if(z[0]==0xf9){queue.push_back(bytes("f800010a96a49800940100000a000000f8000507a7a698003f01000011000000f8000503b5a69800e20000000e000000"));}
  else if(z[0]==0xf2){Bytes r=bytes("f1000700000001000100010000000000");r[3]=uint8_t(z[1]+(wrong_sequence?1:0));r[5]=cal_failure?1:0;queue.push_back(r);}
  else if(z[0]==0xfe){if(z[1]!=absent_id){auto r=features.at(z[1]).set_request();r[0]=0xfc;queue.push_back(r);}}
  else if(z[0]==0xfd){
   ++set_count;if(fail_restore&&set_count>1){errno=EIO;return -1;}
   auto r=z;r[0]=0xfc;auto target=diagnostic::feature_response(r);
   if(!(ignore_enable&&target.interval_us())){
    if(feature_delay>0.)pending_features.emplace_back(time+feature_delay,target);else{features[z[1]]=target;}
   }
   if(late_restore_diverge&&z[1]==2&&!target.interval_us()){diagnostic::put32(r,5,10000);pending_features.emplace_back(time+.04,diagnostic::feature_response(r));}
   if(set_count==interrupt_set_at)stopped=SIGTERM;
   if(set_count==short_set_at)return ssize_t(n-1);
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
static void baseline_lifecycle(){
 auto run=[](const char*name,const std::function<void(Fake&)>&setup,bool compare,bool optin,bool success,bool restored,bool interrupt_baseline=false){
  Fake f;for(auto id:diagnostic::monitored){auto p=f.features[id].set_request();p[0]=0xfc;diagnostic::put32(p,5,0);p[2]=id;p[3]=uint8_t(id+1);p[9]=uint8_t(id+2);p[13]=uint8_t(id+3);f.features[id]=diagnostic::feature_response(p);}f.before=f.features;setup(f);
  std::ostringstream reads,writes,packets,state;for(auto*s:{&reads,&writes,&packets,&state})s->exceptions(std::ios::badbit|std::ios::failbit);f.broken=&packets;
  IO io{[&](uint8_t*p,size_t n){return f.read(p,n);},[&](const uint8_t*p,size_t n){return f.write(p,n);},[&](){return f.time;},[&](){f.time+=.005;if(interrupt_baseline&&f.set_count==2&&f.time>106.)stopped=SIGTERM;}};
  stopped=0;Capture c(io,reads,writes,packets,state);int code=c.run(compare,{7,8,0x13},60.,optin);
  expect((code==0)==success,name);if(restored)expect(f.features==f.before,"baseline exact original fields restored");
  expect(state.str().find("\"restoration_uncertain\":"+std::string(restored?"false":"true"))!=std::string::npos,"baseline restoration result honest");
  for(const auto&r:f.sent)expect(diagnostic::allowed_request(r,compare),"baseline only permitted query/feature writes");
  if(!optin||!compare||f.absent_id>=0||f.before.at(7).interval_us())expect(f.set_count==0,"baseline refusal writes no features");
  if(!compare&&optin)expect(f.sent.empty(),"inspect cannot opt into baseline writes");
  if(interrupt_baseline||f.interrupt_set_at)expect(code==143,"baseline signal preserved");
  if(success){
   expect(c.hard_deadline==230.&&f.time<230.,"30 plus60 capture has130 second hard bound");
   std::vector<size_t> sets;for(size_t i=0;i<f.sent.size();++i)if(f.sent[i][0]==0xfd)sets.push_back(i);
   expect(sets.size()==10,"two baseline plus three extra writes and five restores");
   const std::array<uint8_t,10> order{{2,5,7,8,0x13,0x13,8,7,5,2}};
   for(size_t i=0;i<order.size();++i)expect(f.sent[sets[i]][1]==order[i],"baseline write and reverse restoration order");
   for(size_t i=0;i<2;++i){auto wanted=f.before.at(order[i]).set_request();diagnostic::put32(wanted,5,10000);expect(f.sent[sets[i]]==wanted,"baseline changes only report interval");}
   expect(f.sent_times[sets[2]]-f.sent_times[sets[1]]>=30.,"no extras until complete30second baseline");
   expect(f.sent_times[sets[5]]-f.sent_times[sets[4]]>=60.,"no restore until complete60second comparison");
   expect(state.str().find("\"phase\":\"baseline_02_05\",\"seconds\":30.000000")!=std::string::npos,"baseline phase labeled");
   expect(state.str().find("\"phase\":\"comparison\",\"seconds\":60.000000")!=std::string::npos,"comparison phase labeled");
  }
  std::cout<<name<<": PASS\n";stopped=0;
 };
 run("disabled baseline optin captures and restores",[](Fake&){},true,true,true,true);
 run("disabled reports without optin refused",[](Fake&){},true,false,false,true);
 run("inspect baseline optin refused",[](Fake&){},false,true,false,true);
 run("baseline missing original refused",[](Fake&f){f.absent_id=0x13;},true,true,false,true);
 run("baseline active extra refused",[](Fake&f){auto p=f.features[7].set_request();p[0]=0xfc;diagnostic::put32(p,5,20000);f.features[7]=diagnostic::feature_response(p);f.before=f.features;},true,true,false,true);
 run("baseline first partial write restored",[](Fake&f){f.short_set_at=1;},true,true,false,true);
 run("baseline second partial write restored",[](Fake&f){f.short_set_at=2;},true,true,false,true);
 run("baseline extra partial write restored",[](Fake&f){f.short_set_at=3;},true,true,false,true);
 run("baseline phase signal restores disabled originals",[](Fake&){},true,true,false,true,true);
 run("baseline extra signal restores disabled originals",[](Fake&f){f.interrupt_set_at=3;},true,true,false,true);
 run("baseline restore failure remains uncertain",[](Fake&f){f.short_set_at=1;f.fail_restore=true;},true,true,false,false);
 run("baseline coupled readback mismatch restores",[](Fake&f){f.coupling=true;},true,true,false,true);
}
static void settling_lifecycle(){
 for(unsigned scenario=0;scenario<5;++scenario){
  Fake f;for(auto id:diagnostic::monitored){Bytes p(17,0);p[0]=0xfc;p[1]=id;f.features[id]=diagnostic::feature_response(p);}f.before=f.features;
  f.feature_delay=scenario==4?0.:.06;f.ignore_enable=scenario==1;f.signal_while_wait=scenario==2;f.late_restore_diverge=scenario==4;
  std::ostringstream reads,writes,packets,state;
  IO io{[&](uint8_t*p,size_t n){return f.read(p,n);},[&](const uint8_t*p,size_t n){return f.write(p,n);},[&](){return f.time;},[&](){f.time+=.005;}};
  stopped=0;Capture c(io,reads,writes,packets,state);c.permit_set=true;c.original=f.before;c.deadline=scenario==3?100.045:105.;c.hard_deadline=115.;
  auto expected=diagnostic::feature_response(bytes("fc02000000102700000000000000000000"));
  bool accepted=true;try{c.set_feature(expected);}catch(const std::exception&){accepted=false;}
  expect(accepted==(scenario==0||scenario==4),"settling acceptance follows exact effective fields");
  expect(f.set_count==1,"settling sends one FD only");
  if(scenario!=4&&scenario!=2)expect(state.str().find("feature_wait_mismatch")!=std::string::npos,"old zero or mismatched FC retained");
  double prior=-1.;for(size_t i=0;i<f.sent.size();++i){if(f.sent[i][0]==0xfe){if(prior>=0)expect(f.sent_times[i]-prior>=.02,"FE queries spaced at least20ms");prior=f.sent_times[i];}}
  if(scenario==0)expect(f.time>=100.06&&f.time<101.03,"delayed exact FC accepted within one second");
  if(scenario==1)expect(f.time>=101.&&f.time<101.03,"persistent mismatch fails at bounded deadline");
  if(scenario==2)expect(stopped==SIGTERM,"signal during settling propagates");
  if(scenario==3)expect(f.time<100.075,"shorter operation deadline honored");
  bool restored=c.restore();
  expect(restored==(scenario!=4),"late complete restoration verification is truthful");
  expect(f.set_count==2,"delayed readback never resends FD, including restore");
  if(restored)expect(f.features==f.before,"delayed restoration returns exact disabled originals");
  if(scenario==4)expect(state.str().find("restore_late_readback_failed")!=std::string::npos,"late divergence cannot masquerade as restoration");
  expect(f.time<115.,"settling stays within restoration deadline");stopped=0;
 }
 std::cout<<"asynchronous feature and restoration settling: PASS\n";
}
#include "generated_fixture_checks.hpp"
int main(){(void)&monotonic_s;(void)&signal_stop;fixture_checks();lifecycle();baseline_lifecycle();settling_lifecycle();std::cout<<"checks="<<checks<<"\n";}
