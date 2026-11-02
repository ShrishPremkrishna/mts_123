#pragma once
#include "bno_protocol.hpp"
#include <array>
#include <cstdint>
#include <vector>
#include <stdexcept>
namespace diagnostic {
using Bytes=std::vector<uint8_t>;
constexpr std::array<uint8_t,5> monitored{{2,5,7,8,0x13}};
inline bool monitored_id(uint8_t id){return std::find(monitored.begin(),monitored.end(),id)!=monitored.end();}
inline bool supported_id(uint8_t id){return monitored_id(id)||id==0x15;}
inline bool extra_id(uint8_t id){return id==7||id==8||id==0x13||id==0x15;}
inline uint32_t u32(const uint8_t*p){return uint32_t(p[0])|(uint32_t(p[1])<<8)|(uint32_t(p[2])<<16)|(uint32_t(p[3])<<24);}
inline void put32(Bytes &p,size_t i,uint32_t v){for(unsigned k=0;k<4;++k)p[i+k]=uint8_t(v>>(8*k));}
inline Bytes product_query(){return {0xf9,0};}
inline Bytes calibration_query(uint8_t seq){Bytes p(12,0);p[0]=0xf2;p[1]=seq;p[2]=7;p[6]=1;return p;}
inline Bytes feature_query(uint8_t id){if(!supported_id(id))throw std::invalid_argument("unsupported feature");return {0xfe,id};}
struct Feature {
  std::array<uint8_t,16> fields{};
  uint8_t id()const{return fields[0];}
  uint32_t interval_us()const{return u32(fields.data()+4);}
  Bytes set_request()const{Bytes p{0xfd};p.insert(p.end(),fields.begin(),fields.end());return p;}
  bool operator==(const Feature&b)const{return fields==b.fields;}
};
inline Feature feature_response(const Bytes&p){
 if(p.size()!=17||p[0]!=0xfc||!supported_id(p[1]))throw std::invalid_argument("invalid feature response");
 Feature f;std::copy(p.begin()+1,p.end(),f.fields.begin());return f;
}
inline Feature diagnostic_feature(uint8_t id){
 if(!extra_id(id))throw std::invalid_argument("not a diagnostic feature");
 Bytes p(17,0);p[0]=0xfc;p[1]=id;put32(p,5,id==0x13?1000000:10000);return feature_response(p);
}
inline bool allowed_request(const Bytes&p,bool permit_set){
 if(p==product_query())return true;
 if(p.size()==2&&p[0]==0xfe&&supported_id(p[1]))return true;
 if(p.size()==12&&p[0]==0xf2&&p==calibration_query(p[1]))return true;
 return permit_set&&p.size()==17&&p[0]==0xfd&&supported_id(p[1]);
}
// Entire control batch, no scanning past unknown or partial records.
inline bool control_records(const Bytes&data,std::vector<Bytes>&out){
 out.clear();std::vector<Bytes> pending;
 for(size_t i=0;i<data.size();){unsigned n=data[i]==0xf8||data[i]==0xf1?16:data[i]==0xfc?17:0;
  if(!n||i+n>data.size())return false;
  pending.emplace_back(data.begin()+i,data.begin()+i+n);i+=n;
 }
 out=std::move(pending);return !out.empty();
}
struct Report {
 uint8_t id=0,sequence=0,status_raw=0,accuracy=0;
 uint16_t delay_ticks=0;int64_t reference_delta_ticks=0;bool timing_valid=false;
 std::array<int16_t,6> values{};uint16_t rotation_accuracy=0;uint8_t stability=0;
 uint32_t raw_gyro_timestamp_us=0; // 0x15 only; physical ADC xyz/temp in values[0..3].
};
// Diagnostic-only extensions: same atomic timestamp/boundary rules, plus07/15.
// SH-2 v1.9 section6.5.12:15 is16B, signed ADC xyz/temp and uint32 hub time(us).
inline bool reports(const Bytes&data,std::vector<Report>&out){
 out.clear();std::vector<Report> pending;size_t cursor=0;int64_t reference=0;bool have=false,valid=false;
 while(cursor<data.size()){
  uint8_t id=data[cursor];unsigned n=id==0xfb||id==0xfa?5:id==2?10:id==5?14:id==7?16:id==8?12:id==0x13?6:id==0x15?16:0;
  if(!n||cursor+n>data.size())return false;
  const uint8_t*r=data.data()+cursor;
  if(id==0xfb){int32_t d=pupper_bno::i32(r+1);reference=-int64_t(d);have=true;valid=d!=0x7fffffff;}
  else if(id==0xfa){if(!have)return false;reference+=pupper_bno::i32(r+1);}
  else{
   if(!have) { return false; }
   Report z;z.id=id;z.sequence=r[1];z.status_raw=r[2];z.accuracy=r[2]&3;
   z.delay_ticks=uint16_t(r[2]&0xfc)<<6|r[3];z.reference_delta_ticks=reference;z.timing_valid=valid;
   if(id==0x13)z.stability=r[4];else{
    const unsigned count=id==7?6:id==2?3:4;
    for(unsigned k=0;k<count;++k)z.values[k]=pupper_bno::i16(r+4+2*k);
    if(id==5)z.rotation_accuracy=pupper_bno::u16(r+12);
    if(id==0x15)z.raw_gyro_timestamp_us=u32(r+12);
   }
   pending.push_back(z);
  }
  cursor+=n;
 }
 out=std::move(pending);return have&&!out.empty();
}
} // namespace diagnostic
