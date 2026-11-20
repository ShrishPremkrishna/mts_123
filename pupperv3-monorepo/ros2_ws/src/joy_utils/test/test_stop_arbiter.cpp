#include "joy_utils/stop_arbiter.hpp"
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
using namespace pupper_stop;
static void check(bool condition, const char *message) { if (!condition) throw std::runtime_error(message); }
struct FakePort : Port {
  bool available = true, throw_start = false, throw_poll = false, throw_after_send = false, refuse_start = false;
  unsigned emergency_count = 0;
  uint64_t next = 0;
  std::vector<Request> calls;
  std::map<uint64_t, std::optional<bool>> replies;
  std::vector<uint64_t> forgotten;
  std::set<std::string> remote_active;
  void emergency() override { ++emergency_count; }
  bool ready() override { return available; }
  uint64_t start(const Request &r) override {
    if (r.ensure_active && refuse_start) return 0;
    if (throw_start) throw std::runtime_error("injected transport failure");
    calls.push_back(r); replies[++next] = std::nullopt;
    if (throw_after_send) throw std::runtime_error("submission outcome ambiguous");
    return next;
  }
  std::optional<bool> poll(uint64_t id) override { if (throw_poll) throw std::runtime_error("ambiguous future"); return replies.at(id); }
  void forget(uint64_t id) noexcept override { forgotten.push_back(id); replies.erase(id); }
  void answer(bool ok = true) {
    check(!replies.empty(), "expected outstanding request");
    auto it=replies.begin();
    if(ok) {
      const auto &request=calls.at(it->first-1);
      for(const auto &name:request.deactivate) remote_active.erase(name);
      for(const auto &name:request.activate) remote_active.insert(name);
    }
    it->second=ok;
  }
};
struct Rig {
  std::shared_ptr<FakePort> port = std::make_shared<FakePort>();
  StopArbiter worker{port, {"default","three_legged","original","student","gemini_walk","trick_sit","walk_spider"}, {0,1,2,3}};
  StopArbiter::Time now{};
  void joy(std::initializer_list<size_t> pressed) { std::vector<int32_t> b(13); for(auto n:pressed)b[n]=1; worker.joy(b); }
  void tick(int ms=1) { now+=std::chrono::milliseconds(ms); worker.tick(now); }
  void complete() { port->answer(); tick(); }
  void stopped() { joy({12}); tick(); check(port->calls.back().stop,"stop request"); complete(); joy({}); }
};
static void simultaneous_and_held_stop() {
  Rig r; r.joy({0,1,2,3,9,12});r.tick();
  check(r.port->calls.size()==1 && r.port->calls[0].stop,"stop must win all-buttons frame");
  r.complete();r.joy({12});r.joy({1,9,12});r.tick();
  check(r.port->calls.size()==1,"held stop blocks activation");
  r.joy({1,9});r.tick();check(r.port->calls.size()==1,"held edge not synthesized after stop release");
  r.joy({});r.joy({9});r.tick();
  check(r.port->calls.size()==2 && r.port->calls.back().activate==std::vector<std::string>{"default"},"fresh release activates remembered default");
}
static void stale_queued_activation() {
  Rig r;r.port->available=false;r.joy({2});r.tick();r.joy({12});
  r.port->available=true;r.tick();check(r.port->calls.size()==1 && r.port->calls.back().stop,"stale queued actor discarded");
  r.complete();r.joy({});r.joy({0});r.tick();check(r.port->calls.size()==1,"face ignored while latched");
  r.joy({});r.joy({9});r.tick();check(r.port->calls.back().activate==std::vector<std::string>{"original"},"release remembers last accepted face selection");
}
static void delayed_activation_then_corrective_stop() {
  Rig r;r.joy({1});r.tick();r.joy({12});r.tick();
  check(r.port->emergency_count>=1 && r.port->calls.size()==1,"immediate signal; one outstanding activation");
  r.joy({});r.joy({9});r.tick();check(r.worker.status().latched,"early release cannot defer rearm");
  r.port->answer();
  check(r.port->remote_active.count("three_legged")==1 && r.worker.status().latched,
        "in-flight remote activation can complete while latched: explicit safety boundary");
  r.tick();check(r.port->calls.size()==2 && r.port->calls.back().stop,"late activation followed by corrective deactivation");
  r.complete();check(r.port->remote_active.empty(),"corrective acknowledged deactivation clears simulated controller state");r.tick();check(r.port->calls.size()==2,"early release edge consumed");
  r.joy({});r.joy({9});r.tick();check(r.port->calls.back().activate==std::vector<std::string>{"three_legged"},"fresh release allowed after confirmed cleanup");
}
static void timeout_quarantine_and_late_cleanup() {
  Rig r;r.joy({0});r.tick();r.tick(1600);
  check(r.worker.status().fault && r.worker.status().latched,"timeout quarantines");
  for(int n=0;n<1000;++n){r.joy({});r.joy({9});r.tick(20);}
  check(r.port->calls.size()==1 && r.port->replies.size()==1,"bounded unresolved request, no retry activation");
  r.port->answer();r.tick();check(r.port->calls.size()==2 && r.port->calls.back().stop,"late timedout activation still cleaned up");
  r.complete();r.joy({});r.joy({9});r.tick();check(r.port->calls.size()==2 && r.worker.status().fault,"timeout fault never automatically rearmed");
}
static void repeated_stop_epoch() {
  Rig r;r.joy({12});r.tick();r.joy({});r.joy({12});r.complete();
  check(r.port->calls.size()==2 && r.port->calls.back().stop,"new stop edge cannot be acknowledged by old epoch");
  r.complete();check(!r.worker.status().stop_required,"latest stop acknowledged");
}
static void service_failures_are_bounded() {
  Rig r;r.joy({12});r.tick();
  for(int n=0;n<3;++n){r.port->answer(false);r.tick();}
  check(r.port->calls.size()==3 && r.worker.status().fault,"failed stop retries bounded at three");
  r.joy({});r.joy({9});r.tick();check(r.port->calls.size()==3,"failed stop cannot release");
  Rig t;t.port->throw_start=true;t.joy({1});t.tick();
  check(t.worker.status().fault && t.worker.status().latched,"transport exception quarantines");
}
static void shutdown_pending_response_is_bounded_and_lifetime_safe() {
  auto port=std::make_shared<FakePort>();
  { StopArbiter worker(port,{"default","gemini_walk"},{0});
    std::vector<int32_t> b(13);b[0]=1;worker.joy(b);worker.tick(StopArbiter::Time{});
    worker.shutdown();worker.shutdown();worker.joy(b);worker.tick(StopArbiter::Time{});
    check(port->calls.size()==1 && port->forgotten.size()==1 && port->replies.empty(),"shutdown removes local tracking exactly once");
  }
  check(port->calls.size()==1,"destruction launches no new async work");
}
static void ambiguous_future_retained_for_late_cleanup() {
  Rig r;r.joy({1});r.tick();r.port->throw_poll=true;
  for(int n=0;n<100;++n)r.tick();
  check(r.worker.status().fault && r.worker.status().in_flight && r.port->calls.size()==1,
        "future exceptions retain ambiguous request and quarantine");
  r.port->throw_poll=false;r.port->answer();r.tick();
  check(r.port->calls.size()==2 && r.port->calls.back().stop,
        "recovered late outcome still receives corrective stop after repeated ambiguity");
  r.complete();check(r.port->remote_active.empty() && r.worker.status().fault,"cleanup does not auto-rearm");
}
static void ambiguous_submission_cannot_retry_or_rearm() {
  Rig r;r.port->throw_after_send=true;r.joy({0});r.tick();
  check(r.worker.status().unknown_remote_submission && r.worker.status().fault && !r.worker.status().in_flight,
        "submission exception is explicitly untracked remote uncertainty");
  for(int n=0;n<30;++n){r.joy({});r.joy({9});r.tick(100);}
  check(r.port->calls.size()==1 && r.port->emergency_count>=30,
        "unknown submission gets bounded stop reassertion, no unordered RPC retries or fresh release");
  check(r.port->replies.size()==1,"remote unresolved operation remains an explicit operator recovery gap");
}
static Request external(std::vector<std::string> activate, std::vector<std::string> deactivate={}, int strictness=2) {
  Request request{false,0,std::move(activate),std::move(deactivate)};request.strictness=strictness;return request;
}
static void gateway_external_epoch_and_completion() {
  Rig r;auto id=r.worker.submit(external({"gemini_walk"},{"default"}),r.now);check(id!=0,"valid motion accepted");
  r.joy({12});auto rejected=r.worker.take_completions();
  check(rejected.size()==1 && rejected[0].command_id==id && !rejected[0].ok,"stop invalidates accepted queued motion");
  r.tick();check(r.port->calls.size()==1 && r.port->calls[0].stop,"no queued activation sent after stop");
  Rig t;auto active=t.worker.submit(external({"gemini_walk"},{"default"},2),t.now);t.tick();
  check(t.port->calls[0].strictness==2,"STRICT retained in backend request");
  t.joy({12});t.port->answer();t.tick();auto late=t.worker.take_completions();
  check(late.size()==1 && late[0].command_id==active && !late[0].ok,"overlapping stop prevents successful activation acknowledgement");
  check(t.port->calls.size()==2 && t.port->calls.back().stop,"late external activation receives corrective stop");
}
static void gateway_feedback_and_startup_latch() {
  auto port=std::make_shared<FakePort>();StopArbiter worker(port,{"default","gemini_walk"},{0},12,9,std::chrono::milliseconds(1500),true);
  StopArbiter::Time now{};check(worker.status().latched && worker.status().epoch==1,"startup requires deliberate rearm");
  check(worker.submit(external({"gemini_walk"}),now)==0,"motion cannot bypass startup latch");
  auto feedback=worker.submit(external({"joint_state_broadcaster","imu_sensor_broadcaster"}),now);
  check(feedback!=0,"known feedback broadcasters allowed while latched");
  worker.tick(now);check(port->calls.size()==1 && port->calls[0].stop,"initial cleanup precedes queued feedback");
  port->answer();worker.tick(now);check(port->calls.size()==2 && !port->calls.back().motion,"feedback uses serialized backend");
  port->answer();worker.tick(now);auto done=worker.take_completions();
  check(done.size()==1 && done[0].ok && worker.status().latched,"feedback never silently releases latch");
}
static void gateway_rejects_unlisted_and_ambiguous_requests() {
  Rig r;check(r.worker.submit(external({"unknown"}),r.now)==0,"unlisted actor rejected");
  check(r.worker.submit(external({"default","student"}),r.now)==0,"multiple motion actors rejected");
  check(r.worker.submit(external({"default"},{"default"}),r.now)==0,"overlap rejected");
  check(r.worker.submit(external({"default","default"}),r.now)==0,"duplicates rejected");
  r.port->throw_after_send=true;auto id=r.worker.submit(external({"gemini_walk"}),r.now);r.tick();
  auto done=r.worker.take_completions();check(done.size()==1 && done[0].command_id==id && !done[0].ok,"unknown submission returns failure once");
  check(r.worker.status().unknown_remote_submission,"untrackable outcome remains permanent quarantine");
}
static void gateway_queue_deadline_prevents_late_dispatch() {
  Rig r;r.port->available=false;auto id=r.worker.submit(external({"gemini_walk"}),r.now);
  r.tick(1600);auto done=r.worker.take_completions();
  check(done.size()==1 && done[0].command_id==id && !done[0].ok,"undispatched request expires explicitly");
  r.port->available=true;r.tick();check(r.port->calls.empty(),"expired activation never dispatches on service recovery");
}
static void gateway_timeout_single_response_and_safe_deactivation() {
  Rig r;auto id=r.worker.submit(external({"gemini_walk"}),r.now);r.tick();r.tick(1600);
  auto timeout=r.worker.take_completions();check(timeout.size()==1 && timeout[0].command_id==id && !timeout[0].ok,"timeout delivers failure once");
  r.port->answer();r.tick();check(r.worker.take_completions().empty(),"late outcome cannot issue second gateway response");
  check(r.port->calls.back().stop,"late timedout gateway operation cleaned up");
  Rig t;t.stopped();auto stop=t.worker.submit(external({}, {"default","gemini_walk"}),t.now);
  check(stop!=0,"deactivation remains available while latched");t.tick();t.complete();
  auto done=t.worker.take_completions();check(done.size()==1 && done[0].ok && t.worker.status().latched,"deactivation acknowledgement cannot rearm");
}
static uint64_t start(Rig &r, int lifetime_ms=500) {
  const auto state=r.worker.status();
  return r.worker.start_motion(state.epoch,state.last_face,r.now,r.now+std::chrono::milliseconds(lifetime_ms));
}
static void deliberate_start_and_duplicate_pending() {
  Rig r;r.stopped();const auto epoch=r.worker.status().epoch;
  auto id=start(r);check(id!=0 && r.worker.status().latched,"queued Start preserves ordinary latch");
  check(start(r)==0,"pending Start cannot be duplicated");r.tick();
  check(!r.worker.status().latched && r.port->calls.back().ensure_active && r.port->calls.back().activate==std::vector<std::string>{"default"},"dispatch releases latch and remembers Options controller");
  r.complete();auto done=r.worker.take_completions();
  check(done.size()==1 && done[0].command_id==id && done[0].ok && r.worker.status().epoch==epoch,"single correlated Start acknowledgement");
}
static void start_rejections_are_inert() {
  Rig r;r.stopped();const auto before=r.worker.status();
  check(r.worker.start_motion(before.epoch-1,before.last_face,r.now,r.now+std::chrono::milliseconds(500))==0,"old epoch refused");
  check(r.worker.start_motion(before.epoch,"student",r.now,r.now+std::chrono::milliseconds(500))==0,"old selection refused");
  check(start(r,0)==0 && start(r,501)==0,"expired and overlong Start refused");
  const auto after=r.worker.status();check(after.latched && after.epoch==before.epoch && !after.in_flight && !after.fault,"rejected Start cannot mutate state");
  r.joy({12});r.tick();r.complete();check(start(r)==0,"held Stop refuses even after cleanup");
  r.joy({});check(start(r)!=0,"fresh deliberate Start after Stop release");r.tick();r.port->answer(false);r.tick();
  check(start(r)==0 && r.worker.status().fault,"fault never cleared by Start");
}
static void start_expiry_restores_latch_without_retry() {
  Rig r;r.stopped();r.port->available=false;auto id=start(r);r.tick(500);
  auto done=r.worker.take_completions();check(done.size()==1 && done[0].command_id==id && !done[0].ok,"queued Start expires once");
  check(r.worker.status().latched && !r.worker.status().fault,"undispatched expiry restores ordinary Stop");
  const auto count=r.port->calls.size();r.port->available=true;r.tick();
  check(r.port->calls.size()==count,"service recovery after expiry submits no mutation");
  r.tick();check(r.worker.status().latched && r.port->calls.size()==count,"fresh Start required after expiry");
}
static void start_stop_races_and_shutdown() {
  Rig r;r.stopped();auto id=start(r);r.joy({12});
  auto queued=r.worker.take_completions();check(queued.size()==1 && queued[0].command_id==id && !queued[0].ok,"Stop cancels queued Start");
  Rig t;t.stopped();auto active=start(t);t.tick();t.joy({12});t.port->answer();t.tick();
  auto late=t.worker.take_completions();check(late.size()==1 && late[0].command_id==active && !late[0].ok,"Stop wins late activation response");
  check(t.port->calls.back().stop,"late Start activation cleaned up");t.complete();check(t.worker.status().latched,"late completion never rearms");
  Rig u;u.stopped();auto waiting=start(u);u.worker.shutdown();auto closed=u.worker.take_completions();
  check(closed.size()==1 && closed[0].command_id==waiting && !closed[0].ok && start(u)==0,"shutdown cancels Start and refuses restart");
}
static void expired_ready_check_preserves_running_controller() {
  Rig r;r.joy({9});r.tick();r.complete();r.joy({});
  const auto before=r.worker.status();const auto calls=r.port->calls.size();const auto emergencies=r.port->emergency_count;
  r.port->available=false;auto id=start(r);r.tick(500);auto done=r.worker.take_completions();
  check(done.size()==1 && done[0].command_id==id && !done[0].ok,"expired ready check reports failure once");
  check(!r.worker.status().latched && !r.worker.status().stop_required && r.worker.status().epoch==before.epoch,
        "expired ready check must preserve running state");
  r.port->available=true;r.tick();check(r.port->calls.size()==calls && r.port->emergency_count==emergencies,
        "expired ready check must not stop or deactivate a running robot");
  check(r.port->remote_active==std::set<std::string>{"default"},"healthy controller remains active");
}
static void queued_start_grants_no_face_authority() {
  Rig r;r.stopped();const auto count=r.port->calls.size();r.port->available=false;auto id=start(r);
  r.joy({1});auto done=r.worker.take_completions();
  check(done.size()==1 && done[0].command_id==id && !done[0].ok,"face edge cancels stopped queued Start");
  r.tick(5000);r.port->available=true;r.tick();
  check(r.worker.status().latched && r.worker.status().last_face=="default" && r.port->calls.size()==count,
        "cancelled queued Start cannot authorize delayed different controller");
  // Options is a separate deliberate release, even while a Start is queued.
  r.joy({});r.port->available=false;auto another=start(r);r.joy({9});auto replaced=r.worker.take_completions();
  check(replaced.size()==1 && replaced[0].command_id==another && !replaced[0].ok,"Options explicitly replaces queued Start");
  r.port->available=true;r.tick();check(!r.worker.status().latched && !r.port->calls.back().ensure_active,
        "separate Options intent keeps its normal release behavior");
}
static void known_start_refusal_is_inert() {
  for(bool running:{false,true}){
    Rig r;if(running){r.joy({9});r.tick();r.complete();r.joy({});}else r.stopped();
    const auto before=r.worker.status();const auto calls=r.port->calls.size();const auto emergencies=r.port->emergency_count;
    r.port->refuse_start=true;const auto id=start(r);r.tick();auto done=r.worker.take_completions();
    check(done.size()==1 && done[0].command_id==id && !done[0].ok,"known refusal completes once");
    const auto after=r.worker.status();check(after.latched==before.latched && after.epoch==before.epoch && !after.in_flight && !after.fault && !after.stop_required,"known refusal preserves prior authority");
    check(r.port->calls.size()==calls && r.port->emergency_count==emergencies,"local refusal must not stop or switch");
  }
}
static void software_stop_fences_delayed_queued_and_inflight_start() {
  Rig delayed; delayed.stopped(); const auto old_epoch=delayed.worker.status().epoch;
  delayed.worker.request_stop();
  check(delayed.worker.status().epoch>old_epoch && delayed.worker.status().latched,
        "software Stop must advance an already-latched epoch");
  check(delayed.worker.start_motion(old_epoch,"default",delayed.now,
        delayed.now+std::chrono::milliseconds(200))==0,"software Stop must refuse an older ticket");
  const auto next_epoch=delayed.worker.status().epoch;
  delayed.worker.request_stop();
  check(delayed.worker.status().epoch>next_epoch,"repeated software Stop cannot retain earlier tickets");

  Rig queued; queued.stopped(); queued.port->available=false; const auto queued_id=start(queued);
  queued.worker.request_stop(); const auto cancelled=queued.worker.take_completions();
  check(cancelled.size()==1 && cancelled[0].command_id==queued_id && !cancelled[0].ok,
        "software Stop cancels a queued Start exactly once");
  const auto count=queued.port->calls.size(); queued.port->available=true; queued.tick();
  check(queued.port->calls.size()==count+1 && queued.port->calls.back().stop,
        "only cleanup can dispatch after software Stop");

  Rig active; active.stopped(); const auto active_id=start(active); active.tick();
  active.worker.request_stop();
  check(active.worker.status().latched && active.worker.status().in_flight,
        "software Stop latch acknowledgement must not imply in-flight cleanup finished");
  active.port->answer(); active.tick(); const auto late=active.worker.take_completions();
  check(late.size()==1 && late[0].command_id==active_id && !late[0].ok,
        "late Start response cannot acknowledge success after software Stop");
  check(active.port->calls.back().stop,"late software-stopped activation needs corrective deactivation");
  active.complete(); check(active.port->remote_active.empty() && active.worker.status().latched,
        "corrective acknowledgement removes active controllers without releasing latch");
}
int main(){
  try {
    simultaneous_and_held_stop();stale_queued_activation();delayed_activation_then_corrective_stop();
    timeout_quarantine_and_late_cleanup();repeated_stop_epoch();service_failures_are_bounded();
    shutdown_pending_response_is_bounded_and_lifetime_safe();ambiguous_future_retained_for_late_cleanup();ambiguous_submission_cannot_retry_or_rearm();
    gateway_external_epoch_and_completion();gateway_feedback_and_startup_latch();
    gateway_rejects_unlisted_and_ambiguous_requests();gateway_queue_deadline_prevents_late_dispatch();
    gateway_timeout_single_response_and_safe_deactivation();
    deliberate_start_and_duplicate_pending();start_rejections_are_inert();
    start_expiry_restores_latch_without_retry();start_stop_races_and_shutdown();
    expired_ready_check_preserves_running_controller();queued_start_grants_no_face_authority();
    known_start_refusal_is_inert();
    software_stop_fences_delayed_queued_and_inflight_start();
    std::cout<<"PASS 22 deterministic scenarios against actual candidate arbiter; no ROS/hardware\n";
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
