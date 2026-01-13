#include <print_mtc_solution/print_mtc_solution.hpp>

#include <algorithm>
#include <functional>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/msg/stage_description.hpp>
#include <moveit_task_constructor_msgs/msg/stage_statistics.hpp>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>

namespace print_mtc_solution
{
namespace
{
using moveit_task_constructor_msgs::msg::StageDescription;
using moveit_task_constructor_msgs::msg::StageStatistics;
using moveit_task_constructor_msgs::msg::TaskDescription;
using moveit_task_constructor_msgs::msg::TaskStatistics;
using moveit_task_constructor_msgs::msg::Solution;
using moveit_task_constructor_msgs::srv::GetSolution;
using moveit::task_constructor::TaskPtr;

inline std::string indent(int depth)
{
  return std::string(static_cast<size_t>(std::max(0, depth)) * 2, ' ');
}

inline std::string safeTrunc(const std::string& s, size_t max_len)
{
  if (s.size() <= max_len)
    return s;
  return s.substr(0, max_len) + "...";
}

}  // namespace

PrintMtcSolution::PrintMtcSolution(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList PrintMtcSolution::providedPorts()
{
  return BT::PortsList({
      BT::InputPort<TaskPtr>("task", "{mtc_task}", "MTC task pointer used for introspection."),
      BT::InputPort<int>("max_failed_solutions_per_stage", 10,
                         "Max failed solution IDs to query per stage for failure reasons."),
      BT::InputPort<int>("max_failed_solutions_total", 50,
                         "Max failed solution IDs to query overall for failure reasons."),
      BT::InputPort<int>("max_failed_solution_details", 10,
                         "If >0, prints up to this many failed solutions with sub-trajectory details."),
      BT::InputPort<bool>("return_success", false,
                          "If true, returns SUCCESS after printing. If false, returns FAILURE to preserve upstream."),
      BT::OutputPort<std::string>("debug_report", "Full formatted debug report (also printed)."),
  });
}

BT::KeyValueVector PrintMtcSolution::metadata()
{
  return { { "description",
             "If ticked after a PlanMTCTask FAILURE, this behavior logs additional MTC introspection data." },
           { "subcategory", "User Created Behaviors" } };
}

tl::expected<bool, std::string> PrintMtcSolution::doWork()
{
  const auto task_input = getInput<TaskPtr>("task");
  const int max_failed_per_stage = std::max(0, getInput<int>("max_failed_solutions_per_stage").value_or(10));
  const int max_failed_total = std::max(0, getInput<int>("max_failed_solutions_total").value_or(50));
  const int max_failed_details = std::max(0, getInput<int>("max_failed_solution_details").value_or(10));
  const bool return_success = getInput<bool>("return_success").value_or(false);

  if (!task_input || !task_input.value())
  {
    const std::string msg = "No task pointer provided. Pass {mtc_task} to the 'task' port.";
    shared_resources_->logger->publishFailureMessage(name(), msg);
    return tl::make_unexpected(msg);
  }

  // Gather MTC task data
  const TaskPtr task_ptr = task_input.value();
  TaskStatistics stats;
  TaskDescription desc;
  task_ptr->introspection().fillTaskDescription(desc);
  task_ptr->introspection().fillTaskStatistics(stats);
  const bool have_desc = !desc.stages.empty();
  const std::string task_id = desc.task_id.empty() ? "<unknown>" : desc.task_id;

  // Build maps for names and parent relationships
  std::unordered_map<uint32_t, StageDescription> desc_by_id;
  std::unordered_map<uint32_t, StageStatistics> stats_by_id;

  if (have_desc)
  {
    for (const auto& s : desc.stages)
      desc_by_id[s.id] = s;
  }
  for (const auto& s : stats.stages)
    stats_by_id[s.id] = s;

  auto stageName = [&](uint32_t id) -> std::string {
    auto it = desc_by_id.find(id);
    if (it != desc_by_id.end() && !it->second.name.empty())
      return it->second.name;
    return "<stage " + std::to_string(id) + ">";
  };

  // Children map for tree printing (if we have description)
  std::unordered_map<uint32_t, std::vector<uint32_t>> children;
  std::vector<uint32_t> roots;
  if (have_desc)
  {
    std::set<uint32_t> root_set;
    for (const auto& [id, sd] : desc_by_id)
    {
      if (sd.parent_id == sd.id)
        root_set.insert(id);
      else
        children[sd.parent_id].push_back(id);
    }
    for (auto& [pid, vec] : children)
    {
      std::sort(vec.begin(), vec.end(), [&](uint32_t a, uint32_t b) { return stageName(a) < stageName(b); });
    }
    roots.assign(root_set.begin(), root_set.end());
    std::sort(roots.begin(), roots.end(), [&](uint32_t a, uint32_t b) { return stageName(a) < stageName(b); });
  }

  // Stage order: prefer description order, else stats order
  std::vector<uint32_t> stage_order;
  if (have_desc)
  {
    stage_order.reserve(desc.stages.size());
    for (const auto& s : desc.stages)
      stage_order.push_back(s.id);
  }
  else
  {
    stage_order.reserve(stats.stages.size());
    for (const auto& s : stats.stages)
      stage_order.push_back(s.id);
  }

  std::ostringstream out;
  out << "MTC Introspection Dump\n";
  out << "  task_id: " << task_id << "\n\n";

  if (have_desc)
  {
    out << "Stage Tree (TaskDescription)\n";
    std::function<void(uint32_t, int)> dumpTree = [&](uint32_t id, int depth) {
      const auto it = desc_by_id.find(id);
      out << indent(depth) << "- " << stageName(id) << " (id=" << id;
      if (it != desc_by_id.end())
        out << ", flags=" << it->second.flags << ")";
      else
        out << ")";

      out << "\n";

      if (it != desc_by_id.end() && !it->second.properties.empty())
      {
        out << indent(depth + 1) << "properties:\n";
        for (const auto& p : it->second.properties)
        {
          out << indent(depth + 2) << "- " << p.name;
          if (!p.value.empty())
            out << " = " << safeTrunc(p.value, 200);
          if (!p.type.empty())
            out << " (type=" << p.type << ")";
          out << "\n";
        }
      }

      auto cit = children.find(id);
      if (cit != children.end())
      {
        for (uint32_t c : cit->second)
          dumpTree(c, depth + 1);
      }
    };

    if (roots.empty())
    {
      out << "  (no roots found)\n\n";
    }
    else
    {
      for (uint32_t r : roots)
        dumpTree(r, 1);
      out << "\n";
    }
  }
  else
  {
    out << "(No TaskDescription cached for this task_id; stage tree / properties not available.)\n\n";
  }

  out << "Per-Stage Outcomes (TaskStatistics)\n\n";

  struct Bucket
  {
    std::string comment;
    std::string planner_id;
    size_t count{ 0 };
    std::vector<uint32_t> examples;
  };

  int total_failed_queried = 0;

  for (uint32_t stage_id : stage_order)
  {
    auto it = stats_by_id.find(stage_id);
    if (it == stats_by_id.end())
      continue;

    const auto& st = it->second;

    const size_t solved_n = st.solved.size();
    const uint32_t failed_n = !st.failed.empty() ? static_cast<uint32_t>(st.failed.size()) : st.num_failed;

    out << "- " << stageName(stage_id) << " (id=" << stage_id << ")\n";
    out << "    solved: " << solved_n << "\n";
    out << "    failed: " << failed_n;
    if (st.failed.empty() && st.num_failed > 0)
      out << "  (no failed IDs published; only num_failed)";
    out << "\n";
    out << "    total_compute_time: " << st.total_compute_time << " s\n";

    if (failed_n == 0)
    {
      out << "\n";
      continue;
    }

    if (st.failed.empty())
    {
      out << "    failure reasons: unavailable (no failed[] IDs to query)\n\n";
      continue;
    }

    const int stage_cap = std::min<int>(max_failed_per_stage, static_cast<int>(st.failed.size()));
    const int remaining_global = std::max(0, max_failed_total - total_failed_queried);
    const int cap = std::min(stage_cap, remaining_global);

    if (cap <= 0)
    {
      out << "    (global failure query cap reached; skipping GetSolution calls)\n\n";
      continue;
    }

    std::map<std::pair<std::string, std::string>, Bucket> buckets;

    for (int i = 0; i < cap; ++i)
    {
      const uint32_t sol_id = st.failed[static_cast<size_t>(i)];
      Solution sol;
      std::string err;
      auto req = std::make_shared<GetSolution::Request>();
      auto resp = std::make_shared<GetSolution::Response>();
      req->solution_id = sol_id;
      if (!task_ptr->introspection().getSolution(req, resp))
        err = "GetSolution not found in task introspection";
      else
        sol = resp->solution;

      if (!err.empty())
      {
        auto& b = buckets[{ err, "" }];
        b.comment = err;
        b.count++;
        if (b.examples.size() < 5)
          b.examples.push_back(sol_id);
        continue;
      }

      bool matched = false;
      for (const auto& sub : sol.sub_trajectory)
      {
        const auto& info = sub.info;
        if (info.stage_id != stage_id)
          continue;

        matched = true;
        const std::string comment = info.comment.empty() ? "(no comment)" : info.comment;
        const std::string planner_id = info.planner_id;

        auto& b = buckets[{ comment, planner_id }];
        b.comment = comment;
        b.planner_id = planner_id;
        b.count++;
        if (b.examples.size() < 5)
          b.examples.push_back(sol_id);
      }

      if (!matched)
      {
        auto& b = buckets[{ "(no matching SolutionInfo for this stage_id)", "" }];
        b.comment = "(no matching SolutionInfo for this stage_id)";
        b.count++;
        if (b.examples.size() < 5)
          b.examples.push_back(sol_id);
      }

      total_failed_queried++;
    }

    // Sort buckets by count desc
    std::vector<Bucket> sorted;
    sorted.reserve(buckets.size());
    for (auto& [_, b] : buckets)
      sorted.push_back(b);

    std::sort(sorted.begin(), sorted.end(), [](const Bucket& a, const Bucket& b) { return a.count > b.count; });

    out << "    failure reasons (aggregated):\n";
    for (const auto& b : sorted)
    {
      out << "      - [" << b.count << "x] " << safeTrunc(b.comment, 300);
      if (!b.planner_id.empty())
        out << " (planner_id=" << b.planner_id << ")";
      out << "\n";
      out << "        example_solution_ids: ";
      for (size_t k = 0; k < b.examples.size(); ++k)
      {
        out << b.examples[k] << (k + 1 < b.examples.size() ? ", " : "");
      }
      out << "\n";
    }

    if (static_cast<int>(st.failed.size()) > cap)
    {
      out << "    (omitted " << (st.failed.size() - static_cast<size_t>(cap))
          << " more failed IDs for this stage due to cap)\n";
    }

    out << "\n";
  }

  if (max_failed_details > 0)
  {
    out << "Failed Solution Details\n\n";
    std::set<uint32_t> seen;
    int logged = 0;
    for (uint32_t stage_id : stage_order)
    {
      auto it = stats_by_id.find(stage_id);
      if (it == stats_by_id.end())
        continue;
      for (uint32_t sol_id : it->second.failed)
      {
        if (!seen.insert(sol_id).second)
          continue;

        auto req = std::make_shared<GetSolution::Request>();
        auto resp = std::make_shared<GetSolution::Response>();
        req->solution_id = sol_id;
        if (task_ptr->introspection().getSolution(req, resp))
        {
          const Solution& sol = resp->solution;
          out << "- solution_id=" << sol_id << "\n";
          out << "    sub_trajectory count: " << sol.sub_trajectory.size() << "\n";
          for (size_t i = 0; i < sol.sub_trajectory.size(); ++i)
          {
            const auto& sub = sol.sub_trajectory[i];
            const auto& info = sub.info;

            out << "    [" << i << "] stage=" << stageName(info.stage_id) << " (stage_id=" << info.stage_id << ")\n";
            out << "         cost=" << info.cost << "\n";
            if (!info.planner_id.empty())
              out << "         planner_id=" << info.planner_id << "\n";
            if (!info.comment.empty())
              out << "         comment=" << safeTrunc(info.comment, 300) << "\n";
            out << "         markers=" << info.markers.size() << "\n";
            out << "         trajectory_points=" << sub.trajectory.joint_trajectory.points.size() << "\n";
          }
          out << "\n";
          ++logged;
        }
        else
        {
          out << "- solution_id=" << sol_id << ": GetSolution not found in task introspection\n";
          ++logged;
        }

        if (logged >= max_failed_details)
          break;
      }
      if (logged >= max_failed_details)
        break;
    }
  }
  else
  {
    out << "Failed Solution Details: disabled (max_failed_solution_details=0)\n\n";
  }

  const std::string report = out.str();
  setOutput("debug_report", report);

  // UI + console
  shared_resources_->logger->publishWarnMessage(name(), "MTC dump generated for task_id='" + task_id + "'");
  shared_resources_->logger->publishInfoMessage(name(), report);

  if (return_success)
    return { true };

  // Preserve failure so your Fallback keeps the original semantics.
  return tl::make_unexpected("PlanMTCTask failed; printed MTC introspection (task_id=" + task_id + ")");
}

tl::expected<void, std::string> PrintMtcSolution::doHalt()
{
  // Bounded work only; nothing special to cancel.
  return {};
}

}  // namespace print_mtc_solution
