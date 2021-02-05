#pragma once

// std
#include <string>
#include <memory>
#include <chrono>

// ros
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/client_helpers.h>

// boost
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

// behaviortree_cpp
#include <behaviortree_cpp_v3/action_node.h>

namespace nav1_behavior_tree {

template<class ActionT, class BtGoalT>
class PlannerNode : public BT::CoroActionNode
{
  public:
    typedef actionlib::SimpleActionClient<ActionT> Client;
    PlannerNode(const std::string& xml_tag_name, 
        const BT::NodeConfiguration& config, 
        const std::shared_ptr<ros::NodeHandle>& nh) : 
      BT::CoroActionNode(xml_tag_name, config),
      node_(nh)
    {
      getInput("server_timeout", server_timeout_);
      getInput("goal", goal_);
    }

    PlannerNode() = delete;

    virtual ~PlannerNode() {}

    void createActionClient() {
      action_client_.reset();
      action_client_ = std::make_shared<Client>(*node_.get(), service_name_, 
        "true");
      if (!action_client_->waitForServer(ros::Duration(
          server_timeout_/1000))) {
        setStatus(BT::NodeStatus::FAILURE);
      }
    }

    static BT::PortsList providedBasicPorts(BT::PortsList addition) {
      BT::PortsList basic = 
      {
        BT::InputPort<double>("server_timeout"),
        BT::InputPort<BtGoalT>("goal")
      };
      basic.insert(addition.begin(), addition.end());
      return basic;
    }

    static BT::PortsList providedPorts() {
      return providedBasicPorts({});
    }

    BT::NodeStatus tick() override {
      on_start();
      if (status() == BT::NodeStatus::FAILURE) {
        return status();
      }
      
      createActionClient();
      if (status() == BT::NodeStatus::FAILURE) {
        return status();
      }
      
      getInput("goal", goal_);
      on_new_goal_received();
      if (status() == BT::NodeStatus::FAILURE) {
        return status();
      }
      
      while (node_->ok()) {
        on_tick();
        if (status() == BT::NodeStatus::FAILURE) {
          return status();
        }
        
        BtGoalT new_goal;
        getInput("goal", new_goal);
        if (new_goal != goal_) {
          on_new_goal_received();
          if (status() == BT::NodeStatus::FAILURE) {
            return status();
          }
          goal_ = new_goal;
        }

        auto goal_status = action_client_->getState();
        if (goal_status == actionlib::SimpleClientGoalState::ACTIVE ||
            goal_status == actionlib::SimpleClientGoalState::PENDING) {
          setStatusRunningAndYield();
          continue;
        }
        
        if (goal_status == actionlib::SimpleClientGoalState::SUCCEEDED) {
          return on_succeed();
        }

        if (goal_status == actionlib::SimpleClientGoalState::LOST ||
            goal_status == actionlib::SimpleClientGoalState::REJECTED ||
            goal_status == actionlib::SimpleClientGoalState::ABORTED) {
          return on_failure();
        }
      }
      return status();
    }

    void halt() override
    {
      if (action_client_){
        auto goal_status = action_client_->getState();
        while (goal_status == actionlib::SimpleClientGoalState::ACTIVE ||
            goal_status == actionlib::SimpleClientGoalState::PENDING) {
          action_client_->cancelAllGoals();
          ros::Duration(1).sleep();
          goal_status = action_client_->getState();
        }
      }
      CoroActionNode::halt();
    }

  protected:
    virtual void on_tick() = 0;
    virtual void on_start() = 0;
    virtual void on_new_goal_received() = 0;
    
    virtual BT::NodeStatus on_succeed() {
      return BT::NodeStatus::SUCCESS;
    }
    
    virtual BT::NodeStatus on_failure() {
      return BT::NodeStatus::FAILURE;
    }

    std::string service_name_;
    std::shared_ptr<Client> action_client_;
    BtGoalT goal_;
    double server_timeout_;
    std::shared_ptr<ros::NodeHandle> node_;
};

} // namespace nav1_behavior_tree
