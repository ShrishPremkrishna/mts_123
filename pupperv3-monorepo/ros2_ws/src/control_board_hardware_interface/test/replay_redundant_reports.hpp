// Included by the intercepted BNO test so the archived payload, receipt time,
// sequence trackers and bias conversion run through the same native reader.
void replay_archive(const std::string &path,const std::string &scenario) {
  using namespace std::chrono;
  struct Row {int64_t time;std::vector<uint8_t> payload;};
  std::vector<Row> rows;
  std::ifstream source(path);
  std::string hex;
  int64_t now,first_aux=0,last_aux=0;
  auto record_length=[](int id)->unsigned{
    switch(id) {case 0xfb:case 0xfa:return 5;case 2:return 10;case 5:return 14;
      case 7:return 16;case 8:return 12;case 0x13:return 6;default:throw std::runtime_error("unknown archive ID");}
  };
  while(source>>now>>hex) {
    Row row{now,{}};
    for(size_t k=0;k<hex.size();k+=2) row.payload.push_back(std::stoul(hex.substr(k,2),nullptr,16));
    for(size_t k=0;k<row.payload.size();k+=record_length(row.payload.at(k)))
      if(row.payload[k]==7) {if (!first_aux) first_aux=now;last_aux=now;}
    rows.push_back(std::move(row));
  }
  require(first_aux>0,"missing archive or missing uncalibrated gyro");
  const int64_t second=1000000000,drop=first_aux+5*second;
  const bool total=scenario=="total";
  const bool normal=scenario=="baseline";
  const bool gyro=scenario=="gyro1";
  const bool long_quat=scenario=="quat2_5";
  const bool quat=scenario=="quat2" || long_quat;
  require(normal || total || gyro || quat || scenario=="both4","bad archive case");
  const int64_t length=long_quat?2500000000LL:(gyro?1:quat?2:4)*second;
  BNO055 imu(1,10000);imu.enable_redundant_reports(10000);
  BNO055::Output raw, selected;
  pupper_imu::RedundantImu selector;
  bool held=false,restored=false,terminal=false;
  int64_t lost_reference=drop,held_ns=0,result_ns=0;
  size_t row_index=0;
  const char *reason=nullptr;
  const int64_t stop=normal?last_aux:drop+5*second;
  size_t observations=0,qualified_ticks=0;
  int64_t first_qualified=0,max_not_ready=0,unqualified_since=first_aux;
  for (int64_t tick=first_aux;tick<stop;tick+=3846154) {
    while(row_index<rows.size() && rows[row_index].time<=tick) {
      const auto &row=rows[row_index++];
      if (row.time<first_aux-second) continue;
      const bool missing=!normal && row.time>=drop && row.time<drop+length;
      std::vector<uint8_t> payload;
      for (size_t i=0;i<row.payload.size();) {
        const auto id=row.payload[i];
        const unsigned size=record_length(id);
        require(i+size<=row.payload.size(),"truncated source record");
        if (!missing || (!total && (id>=0xfa || (!(id==2 && !quat) && !(id==5 && !gyro)))))
          payload.insert(payload.end(),row.payload.begin()+i,row.payload.begin()+i+size);
        i+=size;
      }
      if (payload.empty()) continue;
      replay_time=Clock::time_point(nanoseconds(row.time));
      load(packet(std::move(payload)));raw=imu.sample();
      if (row.time<drop && !normal) {
        auto q=raw.quaternion_receipt_time-raw.quaternion_minimum_age_at_receipt;
        auto g=raw.gyro_receipt_time-raw.gyro_minimum_age_at_receipt;
        lost_reference=duration_cast<nanoseconds>((gyro?g:quat?q:std::min(q,g)).time_since_epoch()).count();
      }
    }
    replay_time=Clock::time_point(nanoseconds(tick));
    const char *error=selector.select(raw,replay_time,tick>=drop,selected);
    ++observations;
    if (selector.qualified) {
      ++qualified_ticks;
      if (!first_qualified) first_qualified=tick;
      if (unqualified_since) max_not_ready=std::max(max_not_ready,tick-unqualified_since);
      unqualified_since=0;
    } else if (!unqualified_since) unqualified_since=tick;
    if(tick<drop) continue;
    if(error) {reason=error;terminal=true;result_ns=tick;break;}
    if(selector.mode==2 && !held) {held=true;held_ns=tick;}
    if(selector.mode==3) {restored=true;result_ns=tick;break;}
  }
  replay_time={};
  auto ms=[&](int64_t t){return double(t-lost_reference)/1e6;};
  std::cout<<"archive="<<scenario<<" qualified="<<selector.qualified<<" held_ms="<<(held?ms(held_ns):-1)
           <<" restored_or_terminal_ms="<<(result_ns?ms(result_ns):-1)<<" reason="<<(reason?reason:"none")<<std::endl;
  if(normal) {
    std::cout<<"observations="<<observations<<" qualified_ticks="<<qualified_ticks
             <<" span_s="<<double(stop-first_aux)/1e9<<" first_qualified_ms="<<double(first_qualified-first_aux)/1e6
             <<" longest_completed_requalification_ms="<<double(max_not_ready)/1e6<<std::endl;
    require(qualified_ticks>0 && !held && !terminal,"baseline failed");
  }
  else if(gyro || quat) require(held && restored && !terminal,"selected partial outage could not recover");
  else require(terminal && (total?!held:held),"outage incorrectly kept authority");
}
