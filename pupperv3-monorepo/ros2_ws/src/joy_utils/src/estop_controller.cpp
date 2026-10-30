#include "joy_utils/stop_arbiter.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include <future>
#include <iomanip>
#include <map>
#include <random>
#include <regex>
#include <sstream>

namespace pupper_stop {
constexpr char PROTOCOL[] = "pupper_stop_v1";
constexpr char PUBLIC_SWITCH[] = "/controller_manager/switch_controller";
constexpr char BACKEND_SWITCH[] = "/controller_manager/switch_controller_backend";

class RosSwitchPort final : public Port {
 public:
  using Service = controller_manager_msgs::srv::SwitchController;
  using Client = rclcpp::Client<Service>;
  using Inventory = controller_manager_msgs::srv::ListControllers;
  using InventoryClient = rclcpp::Client<Inventory>;
  RosSwitchPort(Client::SharedPtr client, InventoryClient::SharedPtr inventory,
                rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher)
      : client_(std::move(client)), inventory_client_(std::move(inventory)), publisher_(std::move(publisher)) {}
  void emergency() override { publisher_->publish(std_msgs::msg::Empty{}); }
  bool ready() override {
    // The manager can advertise services before resources/controllers are ready.
    // Query fresh state before dispatch, without timing startup/homing as a
    // failed motion request. An unanswered read may be retried safely.
    if (!client_->service_is_ready() || !inventory_client_->service_is_ready()) return false;
    const auto now = std::chrono::steady_clock::now();
    if (inventory_ready_) return true;
    try {
      if (!inventory_pending_) {
        inventory_pending_.emplace(inventory_client_->async_send_request(std::make_shared<Inventory::Request>()));
        inventory_deadline_ = now + std::chrono::seconds(1);
        return false;
      }
      // Even a ready response is stale if no consumer polled it until after
      // its deadline (for example a queued command expired while idle).
      if (now >= inventory_deadline_) { clear_inventory(); return false; }
      if (inventory_pending_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return false;
      const auto response = inventory_pending_->get(); clear_inventory();
      if (!response) return false;
      active_.clear(); types_.clear();
      for (const auto &controller : response->controller) {
        types_[controller.name] = controller.type;
        if (controller.state == "active") active_.push_back(controller.name);
      }
      inventory_ready_ = true;
      return true;
    } catch (...) { clear_inventory(); return false; }
  }
  uint64_t start(const Request &command) override {
    if (pending_ || synthetic_.has_value()) throw std::logic_error("only one outstanding switch is permitted");
    if (!inventory_ready_) throw std::logic_error("fresh controller inventory required");
    inventory_ready_ = false;
    token_ = ++next_token_;
    // A familiar name alone cannot exempt a misconfigured motion plugin from
    // the latch. Refuse known type mismatch locally, without a backend mutation.
    for (const auto &name : command.activate) {
      const char *expected = name == "joint_state_broadcaster" ? "joint_state_broadcaster/JointStateBroadcaster"
          : name == "imu_sensor_broadcaster" ? "imu_sensor_broadcaster/IMUSensorBroadcaster" : nullptr;
      if (expected && (types_.find(name) == types_.end() || types_.at(name) != expected)) {
        synthetic_ = false; return token_;
      }
    }
    auto request = std::make_shared<Service::Request>();
    request->activate_controllers = command.activate;
    request->deactivate_controllers = command.deactivate;
    if (command.stop) {
      request->deactivate_controllers.clear();
      for (const auto &name : command.deactivate)
        if (std::find(active_.begin(), active_.end(), name) != active_.end()) request->deactivate_controllers.push_back(name);
      if (request->deactivate_controllers.empty()) { synthetic_ = true; return token_; }
    }
    request->strictness = command.strictness == 0 ? Service::Request::BEST_EFFORT : command.strictness;
    request->timeout.sec = 1; // finite remote budget; cannot prove remote cancellation
    pending_.emplace(client_->async_send_request(request));
    return token_;
  }
  std::optional<bool> poll(uint64_t token) override {
    if (token != token_) throw std::logic_error("response token does not identify outstanding switch");
    if (synthetic_.has_value()) return *synthetic_;
    if (!pending_) throw std::logic_error("missing outstanding switch");
    if (pending_->wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) return std::nullopt;
    auto response = pending_->get();
    if (!response) throw std::runtime_error("missing response is an ambiguous remote outcome");
    return response->ok;
  }
  void forget(uint64_t token) noexcept override {
    if (token == token_) {
      if (pending_) { try { client_->remove_pending_request(pending_->request_id); } catch (...) {} }
      pending_.reset(); synthetic_.reset();
    }
  }
 private:
  void clear_inventory() noexcept {
    if (inventory_pending_) { try { inventory_client_->remove_pending_request(inventory_pending_->request_id); } catch (...) {} }
    inventory_pending_.reset();
  }
  Client::SharedPtr client_;
  InventoryClient::SharedPtr inventory_client_;
  std::optional<InventoryClient::FutureAndRequestId> inventory_pending_;
  std::chrono::steady_clock::time_point inventory_deadline_{};
  std::vector<std::string> active_;
  std::map<std::string,std::string> types_;
  bool inventory_ready_ = false;
  std::optional<bool> synthetic_;
  uint64_t token_ = 0, next_token_ = 0;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  std::optional<Client::FutureAndRequestId> pending_;
};

// Weakly captured by all ROS callbacks. It owns response headers until a bounded
// arbiter completion, and never holds a raw Node pointer or detached thread.
class StopBridge {
 public:
  using Service = RosSwitchPort::Service;
  using Server = rclcpp::Service<Service>;
  StopBridge(std::shared_ptr<StopArbiter> worker, std::string instance,
             rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status)
      : worker_(std::move(worker)), instance_(std::move(instance)), publisher_(std::move(status)) {}
  void joy(const std::vector<int32_t> &buttons) {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_->joy(buttons); finish(); publish();
  }
  void tick() {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_->tick(StopArbiter::Time::clock::now()); finish(); publish();
  }
  void request(Server::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
               Service::Request::SharedPtr request) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (request->strictness != 0 && request->strictness != Service::Request::STRICT &&
        request->strictness != Service::Request::BEST_EFFORT) {
      answer(service, header, false); return;
    }
    Request command{false, 0, request->activate_controllers, request->deactivate_controllers};
    command.strictness = request->strictness;
    const auto id = worker_->submit(std::move(command), StopArbiter::Time::clock::now());
    if (!id) answer(service, header, false);
    else responses_.emplace(id, Response{std::move(service), std::move(header)});
    publish();
  }
  void shutdown() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    worker_->shutdown();
    try { finish(); publish(); } catch (...) {}
    for (auto &entry : responses_) answer(entry.second.service, entry.second.header, false);
    responses_.clear();
  }
 private:
  struct Response { Server::SharedPtr service; std::shared_ptr<rmw_request_id_t> header; };
  static void answer(const Server::SharedPtr &service, const std::shared_ptr<rmw_request_id_t> &header, bool ok) noexcept {
    try { Service::Response response; response.ok = ok; service->send_response(*header, response); } catch (...) {}
  }
  void finish() {
    for (const auto &done : worker_->take_completions()) {
      auto it = responses_.find(done.command_id);
      if (it != responses_.end()) {
        answer(it->second.service, it->second.header, done.ok); responses_.erase(it);
      }
    }
  }
  void publish() {
    const auto state = worker_->status();
    std::ostringstream json;
    json << std::boolalpha << "{\"protocol\":\"" << PROTOCOL << "\",\"instance_id\":\"" << instance_
         << "\",\"sequence\":" << ++sequence_ << ",\"epoch\":" << state.epoch
         << ",\"latched\":" << state.latched << ",\"fault\":" << state.fault
         << ",\"stop_required\":" << state.stop_required << ",\"in_flight\":" << state.in_flight
         << ",\"unknown_remote_submission\":" << state.unknown_remote_submission
         << ",\"shutting_down\":" << state.shutdown << "}";
    std_msgs::msg::String message; message.data = json.str(); publisher_->publish(message);
  }
  std::mutex mutex_;
  std::shared_ptr<StopArbiter> worker_;
  std::string instance_;
  uint64_t sequence_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::map<uint64_t, Response> responses_;
};

static std::string boot_instance() {
  std::random_device entropy;
  std::ostringstream value;
  value << std::hex << std::setfill('0');
  for (int i=0; i<4; ++i) value << std::setw(8) << static_cast<uint32_t>(entropy());
  return value.str();
}

class EStopController : public rclcpp::Node {
 public:
  EStopController() : Node("estop_controller") {
    rcl_interfaces::msg::ParameterDescriptor immutable;
    immutable.read_only = true;
    immutable.description = "Joystick configuration cached at startup; restart to change";
    const int stop = declare_parameter<int>("estop_index", 12, immutable);
    const int release = declare_parameter<int>("estop_release_index", 9, immutable);
    const auto raw_buttons = declare_parameter<std::vector<int64_t>>("switch_button_indices", {0,1}, immutable);
    const auto names = declare_parameter<std::vector<std::string>>("controller_names",
        {"neural_controller", "neural_controller_three_legged"}, immutable);
    if (stop < 0 || release < 0) throw std::invalid_argument("negative stop/release index");
    std::vector<size_t> buttons;
    for (auto button : raw_buttons) {
      if (button < 0) throw std::invalid_argument("negative face-button index");
      buttons.push_back(static_cast<size_t>(button));
    }
    for (const auto &name : names)
      if (!std::regex_match(name, std::regex("[A-Za-z_][A-Za-z0-9_]*"))) throw std::invalid_argument("invalid controller name");
    const auto instance = boot_instance();
    declare_parameter<std::string>("stop_arbitration_protocol", PROTOCOL, immutable, true);
    declare_parameter<std::string>("stop_arbitration_instance_id", instance, immutable, true);
    auto port = std::make_shared<RosSwitchPort>(
        create_client<RosSwitchPort::Service>(BACKEND_SWITCH),
        create_client<RosSwitchPort::Inventory>("/controller_manager/list_controllers"),
        create_publisher<std_msgs::msg::Empty>("/emergency_stop", 10));
    auto worker = std::make_shared<StopArbiter>(port, names, buttons, stop, release,
        std::chrono::milliseconds(1500), true); // restart never silently clears an old stop
    auto status = create_publisher<std_msgs::msg::String>("/estop_controller/stop_status",
        rclcpp::QoS(1).reliable().transient_local());
    bridge_ = std::make_shared<StopBridge>(worker, instance, status);
    std::weak_ptr<StopBridge> weak = bridge_;
    subscription_ = create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
      [weak](sensor_msgs::msg::Joy::ConstSharedPtr message) { if (auto bridge=weak.lock()) bridge->joy(message->buttons); });
    gateway_ = create_service<RosSwitchPort::Service>(PUBLIC_SWITCH,
      [weak](StopBridge::Server::SharedPtr service, std::shared_ptr<rmw_request_id_t> header,
             RosSwitchPort::Service::Request::SharedPtr request) {
        if (auto bridge=weak.lock()) bridge->request(std::move(service),std::move(header),std::move(request));
        else { RosSwitchPort::Service::Response response; response.ok=false; service->send_response(*header,response); }
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [weak]() { if (auto bridge=weak.lock()) bridge->tick(); });
    bridge_->tick();
  }
  ~EStopController() override { stop_worker(); }
  void stop_worker() noexcept {
    try { if (timer_) timer_->cancel(); } catch (...) {}
    subscription_.reset(); gateway_.reset();
    if (bridge_) bridge_->shutdown(); bridge_.reset();
  }
 private:
  std::shared_ptr<StopBridge> bridge_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  StopBridge::Server::SharedPtr gateway_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace pupper_stop
using EStopController = pupper_stop::EStopController;
#ifndef PUPPER_STOP_TEST
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node=std::make_shared<EStopController>(); rclcpp::spin(node);
  node->stop_worker(); node.reset(); rclcpp::shutdown(); return 0;
}
#endif
