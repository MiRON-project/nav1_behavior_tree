#include <nav1_behavior_tree/executor_bt.h>

namespace nav1_behavior_tree {

ExecutorBT::ExecutorBT(const std::shared_ptr<ros::NodeHandle>& nh) :
    nh_(nh), 
    factory_(std::make_shared<BT::BehaviorTreeFactory>()),
    srv_load_path_(nh_->advertiseService("load_behavior", 
      &ExecutorBT::Load, this)),
    srv_unload_(nh_->advertiseService("unload_behavior", 
      &ExecutorBT::Unload, this)),
    status_pub_(nh_->advertise<BtState>("bt_state", 100, true)),
    exit_(true),
    status_(BT::NodeStatus::IDLE),
    tree_id_(0),
    behavior_name_("") {
  
  nh_->param<std::string>("behavior_tree_package", behavior_package_, 
    "nav1_behavior_tree");
  BT::NodeBuilder move_base_builder =
    [this](const std::string& name, const BT::NodeConfiguration& config)
    {
      return std::make_unique<nav1_behavior_tree::MoveBasePlanner>(name, config, 
        nh_);
    };
  factory_->registerBuilder<nav1_behavior_tree::MoveBasePlanner>("MoveBase", 
    move_base_builder);
}

ExecutorBT::~ExecutorBT() {
  exit_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  status_pub_.shutdown();
  srv_load_path_.shutdown();
  srv_unload_.shutdown();
}

BT::NodeStatus ExecutorBT::GetStatus() {
  const std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

bool ExecutorBT::Load(const std::string& tree_xml) {
  LoadTree load_msg;
  load_msg.request.tree_xml_name = tree_xml;
  return Load(load_msg.request, load_msg.response);
}

bool ExecutorBT::Unload() {
  std_srvs::Trigger trigger_msg;
  return Unload(trigger_msg.request, trigger_msg.response);
}

void ExecutorBT::execute(const std::string& tree_xml) {
  tree_id_++;

  BT::Tree tree;
  try {
    tree = std::move(factory_->createTreeFromFile(tree_xml));
  }
  catch(const BT::RuntimeError& e) {
    std::cerr << e.what() << '\n';
    return;
  }
  
  
  BT::StdCoutLogger logger_cout(tree);
  std::string log_name = "ulm_trace_" + std::to_string(tree_id_) + ".fbl";
  BT::FileLogger logger_file(tree, log_name.c_str());
  BT::PublisherZMQ publisher_zmq(tree);

  auto bt_state_to_msg_state = [](const BT::NodeStatus& state) {
    switch (state) {
      case BT::NodeStatus::IDLE:
        return BtState::IDLE;
      case BT::NodeStatus::RUNNING:
        return BtState::RUNNING;
      case BT::NodeStatus::SUCCESS:
        return BtState::SUCCESS;
      case BT::NodeStatus::FAILURE:
        return BtState::FAILURE;
      default:
        return BtState::UNKNOWN;
    }
  };

  BtState bt_status;
  bt_status.behavior_xml = behavior_name_;
  while(!exit_) {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      status_ = tree.tickRoot();
      bt_status.state = bt_state_to_msg_state(status_);
      bt_status.header.stamp = ros::Time::now();
      status_pub_.publish(bt_status);
      if (status_ != BT::NodeStatus::RUNNING)
        exit_ = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  tree.haltTree();
}

bool ExecutorBT::Load(LoadTreeRequest& req, LoadTreeResponse& res) {
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    exit_ = true;
    if (thread_.joinable()) 
      thread_.join();
    exit_ = false;
  }
  
  behavior_name_ = req.tree_xml_name;
  req.tree_xml_name = ros::package::getPath(behavior_package_) + 
    "/trees/" + req.tree_xml_name;
  
  thread_ = std::thread(&ExecutorBT::execute, this, req.tree_xml_name);  
  res.success = true;
  return true;
}

bool ExecutorBT::Unload(std_srvs::TriggerRequest& req, 
    std_srvs::TriggerResponse& res) {
  exit_ = true;
  if (thread_.joinable()) {
    thread_.join();
    res.success = true;
    res.message = "behavior unloaded";
    return true;
  }
  res.success = false;
  res.message = "behavior unload fails";
  return true;
}

} // namespace nav1_behavior_tree