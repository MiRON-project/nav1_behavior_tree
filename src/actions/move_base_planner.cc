#include <nav1_behavior_tree/actions/move_base_planner.h>

namespace nav1_behavior_tree {

MoveBasePlanner::MoveBasePlanner(const std::string &name, 
    const BT::NodeConfiguration &config, 
    const std::shared_ptr<ros::NodeHandle>& nh) : 
  PlannerNode<move_base_msgs::MoveBaseAction, geometry_msgs::PoseStamped>(
    name, config, nh)
{}

BT::PortsList MoveBasePlanner::providedPorts() {
  BT::PortsList addition = {
    BT::OutputPort<nav_msgs::Path>("executed_path"),
    BT::OutputPort<actionlib::SimpleClientGoalState>("planner_result")
  };
  return PlannerNode<move_base_msgs::MoveBaseAction, 
    geometry_msgs::PoseStamped>::providedBasicPorts(addition);
}

void MoveBasePlanner::on_start() {
  this->service_name_ = "move_base";
}

void MoveBasePlanner::on_tick() {
}

void MoveBasePlanner::init(const geometry_msgs::PoseStamped& goal) {
  this->goal_ = goal;
  this->config().blackboard->set("goal", this->goal_);
}

void MoveBasePlanner::doneCb(const actionlib::SimpleClientGoalState& state) {
  setOutput("planner_result", state);
}

void MoveBasePlanner::activeCb() {
}

void MoveBasePlanner::feedbackCb(const 
    move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  executed_path_.poses.push_back(feedback->base_position);
  setOutput("executed_path", executed_path_);
}

void MoveBasePlanner::on_new_goal_received() {
  auto goal_status = action_client_->getState();
  if (goal_status == actionlib::SimpleClientGoalState::ACTIVE || 
      goal_status == actionlib::SimpleClientGoalState::PENDING) {
    action_client_->cancelGoal();
  }
  
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = goal_.header.frame_id;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_.pose;
  action_client_->sendGoal(goal, 
    boost::bind(&MoveBasePlanner::doneCb, this, _1),
    boost::bind(&MoveBasePlanner::activeCb, this),
    boost::bind(&MoveBasePlanner::feedbackCb, this, _1)
  );
}

} // namespace nav1_behavior_tree