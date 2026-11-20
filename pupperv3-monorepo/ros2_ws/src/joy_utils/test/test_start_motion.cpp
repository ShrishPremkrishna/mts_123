#define PUPPER_STOP_TEST
#include "../src/estop_controller.cpp"
#include <atomic>
#include <iostream>
#include <set>
#include <thread>
using namespace pupper_stop;
using Clock=std::chrono::steady_clock;
static void check(bool value,const char *why){if(!value)throw std::runtime_error(why);}
template<class F> static void wait_for(F test,int ms,const char *why){
  auto end=Clock::now()+std::chrono::milliseconds(ms);
  while(!test()){if(Clock::now()>=end)throw std::runtime_error(why);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
}
static std::string field(const std::string &json,const char *name){
  std::smatch match;
  check(std::regex_search(json,match,std::regex(std::string("\"")+name+"\":(?:\"([^\"]*)\"|([0-9]+))")),"missing status field");
  return match[1].matched?match[1].str():match[2].str();
}
int main(int argc,char **argv){
  const bool unknown_owner=argc>1&&std::string(argv[1])=="unknown";
  const char *arguments[]={"start_motion_test","--ros-args","-p","switch_button_indices:=[0,1,2,3]",
    "-p","controller_names:=[default,three_legged,original,student]"};
  rclcpp::init(6,arguments);
  auto manager=std::make_shared<rclcpp::Node>("start_fake_manager");
  auto client_node=std::make_shared<rclcpp::Node>("start_test_client");
  std::mutex mutex;std::set<std::string> active;std::string status;
  std::atomic<int> switches{0},reads{0},reads_finished{0},inventory_delay{0},switch_delay{0};
  auto backend=manager->create_service<RosSwitchPort::Service>(BACKEND_SWITCH,
    [&](RosSwitchPort::Service::Request::SharedPtr request,RosSwitchPort::Service::Response::SharedPtr response){
      ++switches;std::this_thread::sleep_for(std::chrono::milliseconds(switch_delay.load()));
      std::lock_guard<std::mutex> lock(mutex);response->ok=true;
      // A duplicate STRICT activation fails just like the real manager. Start
      // must avoid submitting it; the generic gateway must preserve it.
      if(request->strictness==RosSwitchPort::Service::Request::STRICT)
        for(const auto &name:request->activate_controllers)if(active.count(name))response->ok=false;
      if(response->ok){for(const auto &name:request->deactivate_controllers)active.erase(name);for(const auto &name:request->activate_controllers)active.insert(name);}
    });
  auto inventory=manager->create_service<RosSwitchPort::Inventory>("/controller_manager/list_controllers",
    [&](RosSwitchPort::Inventory::Request::SharedPtr,RosSwitchPort::Inventory::Response::SharedPtr response){
      ++reads;std::this_thread::sleep_for(std::chrono::milliseconds(inventory_delay.load()));
      std::lock_guard<std::mutex> lock(mutex);
      for(const auto *name:{"default","three_legged","original","student","joint_state_broadcaster","imu_sensor_broadcaster","unknown"}){
        controller_manager_msgs::msg::ControllerState state;state.name=name;state.state=active.count(name)?"active":"inactive";state.type="neural_controller/NeuralController";
        if(state.name=="joint_state_broadcaster")state.type="joint_state_broadcaster/JointStateBroadcaster";
        if(state.name=="imu_sensor_broadcaster")state.type="imu_sensor_broadcaster/IMUSensorBroadcaster";
        response->controller.push_back(state);
      }
      ++reads_finished;
    });
  auto start_client=client_node->create_client<StopBridge::StartService>(START_MOTION);
  auto generic=client_node->create_client<RosSwitchPort::Service>(PUBLIC_SWITCH);
  auto joy_pub=client_node->create_publisher<sensor_msgs::msg::Joy>("/joy",10);
  auto stop_pub=client_node->create_publisher<std_msgs::msg::Empty>(SOFTWARE_STOP,rclcpp::QoS(10).reliable());
  std::atomic<unsigned> native_stops{0};
  auto native_stop_sub=client_node->create_subscription<std_msgs::msg::Empty>("/emergency_stop",10,
    [&](std_msgs::msg::Empty::ConstSharedPtr){++native_stops;});
  sensor_msgs::msg::Joy current_joy;current_joy.buttons.resize(13);
  std::atomic<bool> repeat_joy{false};
  auto joy_timer=client_node->create_wall_timer(std::chrono::milliseconds(20),[&]{
    if(repeat_joy){std::lock_guard<std::mutex> lock(mutex);joy_pub->publish(current_joy);}
  });
  auto status_sub=client_node->create_subscription<std_msgs::msg::String>("/estop_controller/stop_status",rclcpp::QoS(1).reliable().transient_local(),
    [&](std_msgs::msg::String::ConstSharedPtr msg){std::lock_guard<std::mutex> lock(mutex);status=msg->data;});
  auto node=std::make_shared<EStopController>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);
  executor.add_node(manager);executor.add_node(client_node);executor.add_node(node);
  std::thread thread([&]{executor.spin();});int result=0;
  const auto snapshot=[&]{std::lock_guard<std::mutex> lock(mutex);return status;};
  const auto flag=[&](const char *name,bool value){return snapshot().find(std::string("\"")+name+"\":"+(value?"true":"false"))!=std::string::npos;};
  const auto idle=[&]{return flag("in_flight",false)&&flag("stop_required",false);};
  const auto challenge=[&]{
    const auto json=snapshot();auto request=std::make_shared<StopBridge::StartService::Request>();
    request->instance_id=field(json,"instance_id");request->epoch=std::stoull(field(json,"epoch"));
    request->sequence=std::stoull(field(json,"sequence"));request->selected_controller=field(json,"selected_controller");return request;
  };
  const auto response=[&](auto &future){wait_for([&]{return future.wait_for(std::chrono::milliseconds(0))==std::future_status::ready;},3000,"Start response missing");return future.get()->success;};
  const auto send=[&](auto request){auto future=start_client->async_send_request(request);return response(future);};
  const auto joy=[&](std::initializer_list<size_t> buttons){sensor_msgs::msg::Joy msg;msg.buttons.resize(13);for(auto b:buttons)msg.buttons[b]=1;{std::lock_guard<std::mutex> lock(mutex);current_joy=msg;}joy_pub->publish(msg);};
  try{
    wait_for([&]{return start_client->service_is_ready()&&generic->service_is_ready()&&joy_pub->get_subscription_count()==1&&stop_pub->get_subscription_count()==1&&idle()&&flag("latched",true);},3000,"startup discovery/cleanup");
    check(field(snapshot(),"software_stop_topic")==SOFTWARE_STOP,"software Stop capability missing or mismatched");
    check(!send(challenge())&&switches==0,"Start accepted without Joy evidence");
    repeat_joy=true;joy({});std::this_thread::sleep_for(std::chrono::milliseconds(60));
    auto wrong=challenge();wrong->instance_id="previous-gateway";check(!send(wrong),"old gateway accepted");
    wrong=challenge();++wrong->epoch;check(!send(wrong),"old epoch accepted");
    wrong=challenge();wrong->selected_controller="student";check(!send(wrong),"changed controller accepted");
    auto stale=challenge();std::this_thread::sleep_for(std::chrono::milliseconds(550));check(!send(stale),"stale Start accepted");
    check(switches==0&&flag("latched",true)&&!flag("fault",true),"rejections changed startup state");
    repeat_joy=false;std::this_thread::sleep_for(std::chrono::milliseconds(300));
    check(!send(challenge())&&switches==0,"Start accepted stale Joy evidence");
    sensor_msgs::msg::Joy invalid;joy_pub->publish(invalid);std::this_thread::sleep_for(std::chrono::milliseconds(40));
    check(!send(challenge()),"Start accepted invalid Joy frame");
    repeat_joy=true;joy({});std::this_thread::sleep_for(std::chrono::milliseconds(60));
    if(unknown_owner){
      {std::lock_guard<std::mutex> lock(mutex);active.insert("unknown");}
      check(!send(challenge()),"unknown active owner permitted Start");
      wait_for(idle,2000,"unknown owner refusal");
      check(switches==0&&flag("latched",true)&&!flag("fault",true),"known refusal changed stopped state");
      {std::lock_guard<std::mutex> lock(mutex);active.erase("unknown");}
      check(send(challenge()),"Start after inert refusal failed");wait_for(idle,1000,"running status");
      {std::lock_guard<std::mutex> lock(mutex);active.insert("unknown");}
      const auto before=switches.load();check(!send(challenge()),"running unknown owner accepted");
      check(switches==before&&flag("latched",false)&&!flag("fault",true),"known local refusal stopped healthy controller");
      std::cout<<"PASS Start DDS refuses unknown active owner without backend mutation\n";
    }else{
    // Options already activates the remembered controller. Software Start must
    // acknowledge fresh inventory without the redundant STRICT trap.
    joy({9});wait_for([&]{return idle()&&flag("latched",false);},2000,"Options activation");joy({});
    const auto base=switches.load(),read_base=reads.load();auto ready=challenge();check(send(ready),"already-ready Start refused");
    check(switches==base&&reads>read_base&&!flag("fault",true),"ready Start mutated manager or skipped fresh inventory");
    check(!send(ready)&&switches==base,"accepted challenge replayed");
    {std::lock_guard<std::mutex> lock(mutex);active.insert("student");}
    check(send(challenge()),"known competing controller cleanup failed");
    {std::lock_guard<std::mutex> lock(mutex);check(active==std::set<std::string>{"default"},"already-ready ignored competing motion actor");}
    inventory_delay=300;const auto running_calls=switches.load(),running_reads=reads.load();
    check(!send(challenge()),"slow already-ready check did not expire");inventory_delay=0;
    wait_for([&]{return idle()&&reads_finished>running_reads;},1500,"running expiry settled");
    check(flag("latched",false)&&!flag("fault",true)&&switches==running_calls,"expired ready check stopped healthy controller");
    joy({12});wait_for([&]{return idle()&&flag("latched",true);},2000,"Stop cleanup");
    check(!send(challenge()),"held Stop released by Start");joy({});std::this_thread::sleep_for(std::chrono::milliseconds(60));
    switch_delay=200;auto current=challenge();auto operation=start_client->async_send_request(current);
    wait_for([&]{return flag("in_flight",true);},500,"Start pending");check(!send(current),"duplicate pending accepted");
    check(response(operation),"ordinary stopped Start failed");switch_delay=0;
    wait_for(idle,2000,"Start completion status");
    joy({12});wait_for([&]{return idle()&&flag("latched",true);},2000,"second Stop cleanup");joy({});std::this_thread::sleep_for(std::chrono::milliseconds(60));
    inventory_delay=650;const auto before_expiry=switches.load(),before_read=reads.load();check(!send(challenge()),"late inventory dispatched expired Start");
    inventory_delay=0;wait_for([&]{return idle()&&flag("latched",true)&&reads_finished>before_read;},3000,"expiry cleanup");
    check(switches==before_expiry&&!flag("fault",true),"expiry retried activation or invented permanent fault");
    // Once dispatched, Stop must still beat the delayed acknowledgement.
    switch_delay=250;auto late=start_client->async_send_request(challenge());
    wait_for([&]{return switches>before_expiry;},1000,"late Start dispatch");joy({12});
    check(!response(late),"Start acknowledged after Stop");switch_delay=0;
    wait_for([&]{return idle()&&flag("latched",true);},3000,"late activation cleanup");
    joy({});std::this_thread::sleep_for(std::chrono::milliseconds(60));check(send(challenge()),"fresh deliberate restart failed");wait_for(idle,1000,"restart status");
    // A software Stop is received by the actual gateway, not just its native
    // damping subscribers. The immediate newer epoch is only a latch ACK.
    const auto software_stop=[&]{
      const auto epoch=std::stoull(field(snapshot(),"epoch"));
      const auto damping_before=native_stops.load(); stop_pub->publish(std_msgs::msg::Empty{});
      wait_for([&]{return flag("latched",true)&&std::stoull(field(snapshot(),"epoch"))>epoch;},1000,"software Stop latch acknowledgement");
      wait_for([&]{return native_stops>damping_before;},1000,"software Stop did not retain native damping request");
    };
    auto before_stop=challenge(); software_stop(); wait_for(idle,2000,"software Stop cleanup");
    check(!send(before_stop),"delayed Start survived software Stop epoch change");
    software_stop(); wait_for(idle,2000,"repeated software Stop cleanup");
    // Hold Start at fresh inventory; a received Stop cancels before dispatch.
    inventory_delay=200;const auto queued_calls=switches.load(),queued_reads=reads.load();
    auto queued_start=start_client->async_send_request(challenge());
    wait_for([&]{return reads>queued_reads;},1000,"queued Start inventory");
    software_stop();check(!response(queued_start),"software Stop did not cancel queued Start");
    inventory_delay=0;wait_for([&]{return idle()&&reads_finished>queued_reads;},2000,"queued cancellation cleanup");
    check(switches==queued_calls,"cancelled queued Start reached backend");
    // Once dispatched, retain its response and perform corrective deactivation.
    switch_delay=250;const auto inflight_calls=switches.load();
    auto inflight_start=start_client->async_send_request(challenge());
    wait_for([&]{return switches>inflight_calls;},1000,"software-cancelled Start dispatch");
    software_stop();check(flag("in_flight",true),"latch acknowledgement falsely implies cleanup completion");
    check(!response(inflight_start),"late Start succeeded after software Stop");switch_delay=0;
    wait_for([&]{return idle()&&flag("latched",true);},3000,"software Stop late activation cleanup");
    {std::lock_guard<std::mutex> lock(mutex);check(active.empty(),"motion actor remained active after corrective acknowledgement");}
    check(send(challenge()),"fresh Start after software Stop cleanup failed");wait_for(idle,1000,"post-software-Stop restart");
    // Generic STRICT continues to fail on a duplicate; new service alone is idempotent.
    auto strict=std::make_shared<RosSwitchPort::Service::Request>();strict->activate_controllers={"default"};strict->strictness=RosSwitchPort::Service::Request::STRICT;
    auto duplicate=generic->async_send_request(strict);wait_for([&]{return duplicate.wait_for(std::chrono::milliseconds(0))==std::future_status::ready;},2000,"STRICT response");
    check(!duplicate.get()->ok,"generic STRICT semantics changed");wait_for([&]{return flag("fault",true)&&idle();},2000,"fault cleanup");
    check(!send(challenge()),"Start cleared permanent fault");
    std::cout<<"PASS Start DDS: stale/epoch/selection/replay rejection, Options idempotence, software Stop delayed/queued/inflight cancellation, held Stop, expiry, late cleanup, STRICT preservation\n";
    }
  }catch(const std::exception &error){std::cerr<<"FAIL Start DDS: "<<error.what()<<'\n';result=1;}
  node->stop_worker();executor.cancel();thread.join();rclcpp::shutdown();return result;
}
