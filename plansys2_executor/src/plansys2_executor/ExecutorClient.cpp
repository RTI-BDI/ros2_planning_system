// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_msgs/msg/action_execution_info.hpp"

using plansys2_msgs::msg::ActionExecutionInfo;
using plansys2_msgs::msg::Plan;

using plansys2_msgs::srv::GetOrderedSubGoals;
using plansys2_msgs::srv::GetPlan;
using plansys2_msgs::srv::GetUpdatedFeedback;
using plansys2_msgs::srv::EarlyArrestRequest;

namespace plansys2
{

using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecutePlan = plansys2_msgs::action::ExecutePlan;

ExecutorClient::ExecutorClient(const std::string & node_name)
{
  node_ = rclcpp::Node::make_shared(node_name);

  createActionClient();

  get_ordered_sub_goals_client_ = node_->create_client<GetOrderedSubGoals>(
    "executor/get_ordered_sub_goals");
  get_plan_client_ = node_->create_client<GetPlan>("executor/get_plan");
  get_updated_feedback_client_ = node_->create_client<GetUpdatedFeedback>("executor/get_updated_feedback");

  early_arrest_request_client_ = node_->create_client<EarlyArrestRequest>("executor/early_arrest");
}

void
ExecutorClient::createActionClient()
{
  action_client_ = rclcpp_action::create_client<ExecutePlan>(node_, "execute_plan");

  if (!this->action_client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
  }
}

bool
ExecutorClient::start_plan_execution(const Plan & plan)
{
  executing_plan_ = is_plan_executing();
  if (!executing_plan_) {
    createActionClient();
    auto success = on_new_goal_received(plan);

    if (success) {
      executing_plan_ = true;
      return true;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Already executing a plan");
  }

  return false;
}

bool
ExecutorClient::execute_and_check_plan()
{
  executing_plan_ = is_plan_executing();
  if (rclcpp::ok() && !goal_result_available_) {
    rclcpp::spin_some(node_);

    if (!goal_result_available_) {
      return true;  // Plan not finished
    }
  }

  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result_.result == nullptr) {
        RCLCPP_WARN(
          node_->get_logger(), "Plan failed due to a nullptr in the result");
      } else if (result_.result->success) {
        RCLCPP_INFO(node_->get_logger(), "Plan Succeeded");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Plan Failed");
        for (auto msg : result_.result->action_execution_status) {
          //switch (msg.status) {
            if(msg.status == ActionExecutionInfo::SUCCEEDED)
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " succeeded with message_status: " <<
                  msg.message_status);
              //break;
            else if(msg.status == ActionExecutionInfo::FAILED)
              RCLCPP_ERROR_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " failed with message_status: " <<
                  msg.message_status);
              //break;
            else if(msg.status == ActionExecutionInfo::NOT_EXECUTED)
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was not executed");
              //break;
            else if(msg.status == ActionExecutionInfo::CANCELLED)
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was cancelled");
              //break;
            else if(msg.status == ActionExecutionInfo::EXECUTING)
              RCLCPP_WARN_STREAM(
                node_->get_logger(),
                "Action: " <<
                  msg.action_full_name <<
                  " was executing");
          //}
        }
      }
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node_->get_logger(), "Plan Aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Plan Cancelled");
      break;

    default:
      throw std::logic_error("ExecutorClient::executePlan: invalid status value");
  }

  executing_plan_ = false;
  goal_result_available_ = false;

  return false;  // Plan finished
}


bool
ExecutorClient::on_new_goal_received(const Plan & plan)
{
  auto goal = ExecutePlan::Goal();
  goal.plan = plan;

  auto send_goal_options = rclcpp_action::Client<ExecutePlan>::SendGoalOptions();

  send_goal_options.feedback_callback =
    std::bind(&ExecutorClient::feedback_callback, this, _1, _2);

  send_goal_options.result_callback =
    std::bind(&ExecutorClient::result_callback, this, _1);

  auto future_goal_handle = action_client_->async_send_goal(goal, send_goal_options);

  if (rclcpp::spin_until_future_complete(
      node_->get_node_base_interface(), future_goal_handle, 3s) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "send_goal failed");
    return false;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server");
    return false;
  }

  return true;
}

bool
ExecutorClient::should_cancel_goal()
{
  if (!executing_plan_) {
    return false;
  }

  rclcpp::spin_some(node_);
  auto status = goal_handle_->get_status();

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

void
ExecutorClient::cancel_plan_execution()
{
  if (should_cancel_goal()) {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(), future_cancel, 3s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to cancel action server for execute_plan");
    }
  }

  executing_plan_ = false;
  goal_result_available_ = false;
}

ExecutePlan::Feedback 
ExecutorClient::get_updated_feedback_info()
{
  ExecutePlan::Feedback feedback = ExecutePlan::Feedback{};
  if(executing_plan_)
  {
    try{  
      while (!get_updated_feedback_client_->wait_for_service(std::chrono::seconds(1))) {
          if (!rclcpp::ok()) {
              return feedback;
          }
          RCLCPP_ERROR_STREAM(
              node_->get_logger(),
              get_updated_feedback_client_->get_service_name() <<
                  " service client: waiting for service to appear...");
      }

      auto future_result = get_updated_feedback_client_->async_send_request(std::make_shared<GetUpdatedFeedback::Request>());

      if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
          rclcpp::FutureReturnCode::SUCCESS)
      {
          return feedback;
      }

      auto response = future_result.get();
      feedback.action_execution_status = response->action_execution_status;
      feedback.early_abort_accepted = response->early_abort_accepted;
    }
    catch(const rclcpp::exceptions::RCLError& rclerr)
    {
        RCLCPP_ERROR(node_->get_logger(), rclerr.what());
    }
    catch(const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "Response error in while trying to call %s srv", get_updated_feedback_client_->get_service_name());
    }
  }
  return feedback;
}


bool
ExecutorClient::early_arrest_request(const plansys2_msgs::msg::Plan & plan)
{
  executing_plan_ = is_plan_executing();
  if(!executing_plan_)
  {
    return false;
  }
  else
  {
    while (!early_arrest_request_client_->wait_for_service(std::chrono::seconds(5))) {
      if (!rclcpp::ok()) {
        return false;
      }
      RCLCPP_ERROR_STREAM(
        node_->get_logger(),
        early_arrest_request_client_->get_service_name() <<
          " service  client: waiting for service to appear...");
    }

    auto request = std::make_shared<EarlyArrestRequest::Request>();
    request->committed_plan = plan;
    auto future_result = early_arrest_request_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    auto result = *future_result.get();
    return result.accepted;
  }
}

std::vector<plansys2_msgs::msg::Tree> ExecutorClient::getOrderedSubGoals()
{
  std::vector<plansys2_msgs::msg::Tree> ret;

  while (!get_ordered_sub_goals_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return ret;
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_ordered_sub_goals_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<GetOrderedSubGoals::Request>();

  auto future_result = get_ordered_sub_goals_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return ret;
  }

  auto result = *future_result.get();

  if (result.success) {
    ret = result.sub_goals;
  } else {
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      get_ordered_sub_goals_client_->get_service_name() << ": " <<
        result.error_info);
  }

  return ret;
}

std::optional<Plan> ExecutorClient::getPlan()
{
  while (!get_plan_client_->wait_for_service(std::chrono::seconds(5))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_plan_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }

  auto request = std::make_shared<GetPlan::Request>();

  auto future_result = get_plan_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(1)) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    return {};
  }

  auto result = *future_result.get();

  if (result.success) {
    return result.plan;
  } else {
    // RCLCPP_ERROR_STREAM(
    //   node_->get_logger(),
    //   get_plan_client_->get_service_name() << ": " <<
    //     result.error_info);
    return {};
  }
}

void
ExecutorClient::feedback_callback(
  GoalHandleExecutePlan::SharedPtr goal_handle,
  const std::shared_ptr<const ExecutePlan::Feedback> feedback)
{
  feedback_ = *feedback;
}

void
ExecutorClient::result_callback(const GoalHandleExecutePlan::WrappedResult & result)
{
  goal_result_available_ = true;
  result_ = result;
  feedback_ = ExecutePlan::Feedback();
}

std::optional<ExecutePlan::Result>
ExecutorClient::getResult()
{
  if (result_.result != nullptr) {
    return *result_.result;
  } else {
    return {};
  }
}

}  // namespace plansys2
