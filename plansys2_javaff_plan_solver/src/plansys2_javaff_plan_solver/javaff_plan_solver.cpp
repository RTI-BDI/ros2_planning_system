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

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_javaff_plan_solver/javaff_plan_solver.hpp"

namespace plansys2
{

bool isTSPActionLine(const std::string& line)
{
  return line.find(":") != std::string::npos && 
          line.find("(") != std::string::npos && line.find(")") != std::string::npos && 
          line.find("[") != std::string::npos && line.find("]") != std::string::npos;
}

JavaFFPlanSolver::JavaFFPlanSolver()
{
}

void JavaFFPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & lc_node,
  const std::string & plugin_name)
{
  parameter_name_ = plugin_name + ".arguments";
  lc_node_ = lc_node;
  lc_node_->declare_parameter<std::string>(parameter_name_, "");
}

std::optional<plansys2_msgs::msg::Plan>
JavaFFPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return {};
  }

  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  plansys2_msgs::msg::Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  int status = system(
    ("ros2 run javaff javaff_offline /tmp/" + node_namespace + "/domain.pddl /tmp/" + node_namespace +
    "/problem.pddl > /tmp/" + node_namespace + "/plan").c_str());

  if (status == -1) {
    return {};
  }

  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
      if (line.find("Solution Found") != std::string::npos) {
        solution = true;
      }
      } else if (solution && line.front() != ';' && isTSPActionLine(line)) {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par1 = line.find("(");
        size_t colon_par2 = line.find(")");
        size_t colon_bra1 = line.find("[");
        size_t colon_bra2 = line.find("]");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_par1, colon_par2 - colon_par1 + 1);
        std::string duration = line.substr(colon_bra1 + 1, colon_bra2 - colon_bra1 - 1);
        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);

        ret.items.push_back(item);
      }
    }
    plan_file.close();
  }

  if (ret.items.empty()) {
    return {};
  }

  return ret;
}

bool
JavaFFPlanSolver::is_valid_domain(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return {};
  }

  if (node_namespace != "") {
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  int status = system(
    ("ros2 run javaff javaff_offline /tmp/" + node_namespace + "/check_domain.pddl /tmp/" +
    node_namespace + "/check_problem.pddl > /tmp/" + node_namespace + "/check.out").c_str());

  if (status == -1) {
    return {};
  }

  std::ifstream plan_file("/tmp/" + node_namespace + "/check.out");

  std::string result((std::istreambuf_iterator<char>(plan_file)),
    std::istreambuf_iterator<char>());
  std::transform(result.begin(), result.end(), result.begin(), ::tolower);//result output plan to lower case

  return result.find("error") == result.npos;//no "error" string in output from the planner -> domain valid
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::JavaFFPlanSolver, plansys2::PlanSolverBase);
