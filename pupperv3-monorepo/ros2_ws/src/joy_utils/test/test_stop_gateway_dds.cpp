#define PUPPER_STOP_TEST
#include "../src/estop_controller.cpp"
#include <atomic>
#include <iostream>
#include <set>
#include <thread>
using Clock=std::chrono::steady_clock;
using Service=controller_manager_msgs::srv::SwitchController;
static void check(bool value,const char *message){if(!value)throw std::runtime_error(message);}
template<class F> static void wait_for(F test,int ms,const char*message){auto end=Clock::now()+std::chrono::milliseconds(ms);while(!test()){if(Clock::now()>=end)throw std::runtime_error(message);std::this_thread::sleep_for(std::chrono::milliseconds(5));}}
struct Seen{std::vector<std::string> activate,deactivate;int strictness;Clock::time_point time;};
int main(int argc,char **argv){
 const std::string scenario=argc>1?argv[1]:"delayed";
 const int delay=scenario=="timeout"?1800:300;
 const char *arguments[]={"stop_gateway_dds_test","--ros-args","-p","switch_button_indices:=[0,1,2,3]","-p",
  "controller_names:=[default,three_legged,original,student,gemini_walk,trick_sit,walk_spider]","-r",
  "controller_manager:/controller_manager/switch_controller:=/controller_manager/switch_controller_backend"};
 rclcpp::init(8,arguments);
 auto manager=std::make_shared<rclcpp::Node>("controller_manager");
 auto driver=std::make_shared<rclcpp::Node>("neural_controller_default"); // same process; scoped remap MUST NOT bypass gateway
 std::mutex mutex;std::vector<Seen> seen;std::set<std::string> active;std::string status;
 std::atomic<bool> wrong_broadcaster_type{scenario=="broadcaster_type"};
 std::atomic<unsigned> emergency_count{0},completed{0};Clock::time_point first_stop{},activation_return{};
 auto backend=manager->create_service<Service>("/controller_manager/switch_controller",
  [&](rclcpp::Service<Service>::SharedPtr handle,std::shared_ptr<rmw_request_id_t> header,Service::Request::SharedPtr request){
   const bool target=std::find(request->activate_controllers.begin(),request->activate_controllers.end(),"gemini_walk")!=request->activate_controllers.end();
   {std::lock_guard<std::mutex> lock(mutex);seen.push_back({request->activate_controllers,request->deactivate_controllers,request->strictness,Clock::now()});}
   if(target && scenario=="service_loss") return; // deferred response intentionally lost; real client stays unresolved
   if(target)std::this_thread::sleep_for(std::chrono::milliseconds(delay));
   const bool ok=!(target && scenario=="negative");
   {std::lock_guard<std::mutex> lock(mutex);if(ok){for(auto&n:request->deactivate_controllers)active.erase(n);for(auto&n:request->activate_controllers)active.insert(n);}if(target)activation_return=Clock::now();}
   Service::Response response;response.ok=ok;handle->send_response(*header,response);++completed;
  });
 auto inventory=manager->create_service<pupper_stop::RosSwitchPort::Inventory>("/controller_manager/list_controllers",
  [&](pupper_stop::RosSwitchPort::Inventory::Request::SharedPtr,pupper_stop::RosSwitchPort::Inventory::Response::SharedPtr response){
   std::lock_guard<std::mutex> lock(mutex);
   for(const auto *name:{"default","three_legged","original","student","gemini_walk","trick_sit","walk_spider","joint_state_broadcaster","imu_sensor_broadcaster"}){
    controller_manager_msgs::msg::ControllerState state;state.name=name;state.state=active.count(name)?"active":"inactive";state.type="neural_controller/NeuralController";
    if(state.name=="joint_state_broadcaster"&&!wrong_broadcaster_type)state.type="joint_state_broadcaster/JointStateBroadcaster";
    if(state.name=="imu_sensor_broadcaster")state.type="imu_sensor_broadcaster/IMUSensorBroadcaster";
    response->controller.push_back(state);
   }
  });
 auto client=driver->create_client<Service>("/controller_manager/switch_controller");
 check(std::string(backend->get_service_name())==pupper_stop::BACKEND_SWITCH,"manager scoped remap did not reach backend");
 check(std::string(client->get_service_name())==pupper_stop::PUBLIC_SWITCH,"controller-node client inherited backend remap and bypassed gateway");
 auto joy_pub=driver->create_publisher<sensor_msgs::msg::Joy>("/joy",10);
 auto stop_sub=driver->create_subscription<std_msgs::msg::Empty>("/emergency_stop",10,[&](std_msgs::msg::Empty::ConstSharedPtr){std::lock_guard<std::mutex>lock(mutex);if(emergency_count++==0)first_stop=Clock::now();});
 auto status_sub=driver->create_subscription<std_msgs::msg::String>("/estop_controller/stop_status",rclcpp::QoS(1).reliable().transient_local(),[&](std_msgs::msg::String::ConstSharedPtr m){std::lock_guard<std::mutex>lock(mutex);status=m->data;});
 auto node=std::make_shared<EStopController>();
 rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);executor.add_node(manager);executor.add_node(driver);executor.add_node(node);
 std::thread thread([&]{executor.spin();});
 const auto count=[&]{std::lock_guard<std::mutex>lock(mutex);return seen.size();};
 const auto field=[&](const char*key,bool value){std::lock_guard<std::mutex>lock(mutex);return status.find(std::string("\"")+key+"\":"+(value?"true":"false"))!=std::string::npos;};
 const auto joy=[&](std::initializer_list<size_t> buttons){sensor_msgs::msg::Joy m;m.buttons.resize(13);for(auto index:buttons)m.buttons[index]=1;joy_pub->publish(m);};
 const auto send=[&](std::vector<std::string> activate,std::vector<std::string> deactivate={}){auto r=std::make_shared<Service::Request>();r->activate_controllers=std::move(activate);r->deactivate_controllers=std::move(deactivate);r->strictness=Service::Request::STRICT;return client->async_send_request(r);};
 const auto response=[&](auto&future){wait_for([&]{return future.wait_for(std::chrono::milliseconds(0))==std::future_status::ready;},3000,"gateway did not give bounded response");return future.get()->ok;};
 int rc=0;
 try{
  wait_for([&]{return joy_pub->get_subscription_count()==1 && client->service_is_ready();},3000,"gateway/Joy discovery");
  wait_for([&]{return field("latched",true)&&field("stop_required",false)&&field("in_flight",false);},3000,"startup stop not acknowledged");
  const auto startup_calls=count();auto blocked=send({"gemini_walk"});check(!response(blocked)&&count()==startup_calls,"startup latch bypassed");
  if(scenario=="broadcaster_type"){
   auto unsafe=send({"joint_state_broadcaster"});check(!response(unsafe),"misnamed neural plugin bypassed stop latch");
   check(count()==startup_calls&&!field("fault",true),"known type refusal dispatched mutation or invented unknown outcome");
   wrong_broadcaster_type=false;
  }
  auto feedback=send({"joint_state_broadcaster","imu_sensor_broadcaster"});check(response(feedback),"feedback activation blocked by latch");
  check(field("latched",true),"feedback rearmed motion");
  auto unknown=send({"unlisted_controller"});check(!response(unknown),"unlisted actor accepted");
  joy({9});wait_for([&]{return field("latched",false)&&field("in_flight",false);},3000,"fresh release did not arm default");joy({});
  const auto base=count();emergency_count=0;auto operation=send({"gemini_walk"},{"default"});
  wait_for([&]{return count()==base+1;},2000,"public gateway did not forward activation to backend");
  {std::lock_guard<std::mutex>lock(mutex);check(seen.back().strictness==Service::Request::STRICT,"STRICT flag changed at gateway");}
  if(scenario=="shutdown"){
   auto start=Clock::now();node->stop_worker();executor.remove_node(node);node.reset();
   auto ms=std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start).count();check(ms<250,"shutdown blocked on pending response");
   check(!response(operation),"shutdown falsely acknowledged activation");
   wait_for([&]{std::lock_guard<std::mutex>lock(mutex);return activation_return!=Clock::time_point{};},2000,"late backend reply did not finish");
   std::this_thread::sleep_for(std::chrono::milliseconds(200));check(count()==base+1,"RPC sent after gateway shutdown");
   std::cout<<"shutdown_ms="<<ms<<" late_remote_execution_possible=true\n";
  }else if(scenario=="service_loss"){
   executor.remove_node(manager);backend.reset();inventory.reset();manager.reset();
   check(!response(operation),"unanswered operation did not fail closed at local deadline");
   wait_for([&]{return field("fault",true)&&field("latched",true)&&field("in_flight",true);},1000,"service loss did not retain unresolved fault");
   manager=std::make_shared<rclcpp::Node>("controller_manager");
   backend=manager->create_service<Service>("/controller_manager/switch_controller",[&](Service::Request::SharedPtr,Service::Response::SharedPtr reply){reply->ok=true;std::lock_guard<std::mutex>lock(mutex);seen.push_back({{}, {},0,Clock::now()});});executor.add_node(manager);
   joy({});std::this_thread::sleep_for(std::chrono::milliseconds(50));joy({9});auto retry=send({"gemini_walk"});check(!response(retry),"service restart cleared permanent quarantine");
   std::this_thread::sleep_for(std::chrono::milliseconds(200));check(count()==base+1,"new RPC dispatched while old outcome unresolved");
  }else{
   joy({12,9,0,3});wait_for([&]{return emergency_count.load()>0;},200,"stop not observed during delayed RPC");
   check(!response(operation),"overlapping stop/failure returned successful activation ACK");
   wait_for([&]{return count()==base+2&&field("stop_required",false)&&field("in_flight",false);},3000,"corrective stop not confirmed");
   {std::lock_guard<std::mutex>lock(mutex);check(first_stop<activation_return,"observer blocked behind manager");check(seen.back().activate.empty(),"activation followed stop");check(std::set<std::string>(seen.back().deactivate.begin(),seen.back().deactivate.end())==(scenario=="negative"?std::set<std::string>{"default"}:std::set<std::string>{"gemini_walk"}),"stop does not match fresh active covered actors");check(!active.count("gemini_walk")&&!active.count("default"),"motion actor remains active after corrective ACK");}
   joy({12});std::this_thread::sleep_for(std::chrono::milliseconds(50));joy({12,9,1});auto held=send({"gemini_walk"});check(!response(held),"public client bypassed held/latched stop");
   check(count()==base+2,"held stop caused another activation");joy({});std::this_thread::sleep_for(std::chrono::milliseconds(50));joy({9});
   if(scenario=="timeout"||scenario=="negative"){
    std::this_thread::sleep_for(std::chrono::milliseconds(200));check(count()==base+2&&field("fault",true),"fault was cleared by release");
   }else{
    wait_for([&]{return count()==base+3;},1000,"fresh release not processed");
    {std::lock_guard<std::mutex>lock(mutex);check(seen.back().activate==std::vector<std::string>{"default"},"Gemini overwrote remembered face controller");}
   }
  }
  std::cout<<"PASS production gateway actual DDS "<<scenario<<" backend_requests="<<count()<<"\n";
 }catch(const std::exception&e){std::cerr<<"FAIL "<<scenario<<": "<<e.what()<<'\n';rc=1;}
 if(node){node->stop_worker();executor.remove_node(node);node.reset();}executor.cancel();thread.join();rclcpp::shutdown();return rc;
}
