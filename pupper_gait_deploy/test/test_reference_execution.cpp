#include "neural_controller/reference_execution.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using namespace neural_controller;
int main() {
  const std::string id(32,'a'),other(32,'b');
  assert(reference_request_id(std::string(kReferenceRequestPrefix)+id)==id);
  for(const std::string& bad:{std::string{},std::string(31,'a'),std::string(32,'A'),std::string(32,'g'),id+" x"})
    assert(!valid_reference_id(bad));
  ReferenceExecution s;s.accept(id,2.,.8);
  assert(s.running && s.duplicate(id) && !s.duplicate(other));
  s.complete_after_apply(2.79);assert(s.phase=="playing");
  s.reject(other,"busy");assert(s.running && s.status_id==other && s.phase=="rejected");
  s.complete_after_apply(2.8);assert(!s.running && s.status_id==id && s.phase=="completed");
  assert(s.message()==std::string(kReferenceStatusProtocol)+" "+id+" completed ok");
  assert(s.pending.size()==3);
  assert(s.next_message().find(" playing ")!=std::string::npos);s.published();
  assert(s.next_message()==std::string(kReferenceStatusProtocol)+" "+other+" rejected busy");s.published();
  assert(s.next_message()==std::string(kReferenceStatusProtocol)+" "+id+" completed ok");s.published();
  assert(s.next_message()==s.message()); // terminal receipt repeats after delivery
  s.accept(other,3.,1.);s.interrupt("estop");s.complete_after_apply(5.);
  assert(s.phase=="interrupted" && s.reason=="estop");
  s.reset();assert(s.message().empty() && !s.running && !s.duplicate(other));
  s.accept("",2.,1.);assert(s.message().empty()); // legacy request still executes
  bool caught=false;try{s.accept(id,0.,std::numeric_limits<double>::quiet_NaN());}catch(const std::invalid_argument&){caught=true;}
  assert(caught);
  std::cout<<"Reference execution status tests passed\n";
}
