#include <memory>
#include <stdexcept>

#include "pathfinder.h"

namespace Pathfinding {
auto Pathfinder::FindPath_AStar(Node* from, Node* to) const noexcept -> Result {
  struct NodeData {
    float h_score{};
    float g_score{};

    std::optional<Node*> came_from{};
  };

  std::map<Node*, NodeData> node_data{};
  auto nodes_opened = int{};

  // comparison function for std::push_heap and std::pop_heap
  const auto node_score_cmp = [&node_data](const auto& first,
                                           const auto& second) {
    return (node_data[first].h_score + node_data[first].g_score) >
           (node_data[second].h_score + node_data[second].g_score);
  };

  std::vector<Node*> open_nodes{};

  if (from->GetConnections().size() == 0) {
    return {};
  }

  node_data.insert({from, NodeData{}});
  open_nodes.push_back(from);

  nodes_opened++;

  while (open_nodes.size() > 0) {
    std::pop_heap(open_nodes.begin(), open_nodes.end(), node_score_cmp);
    auto lowest_score_node = std::move(open_nodes.back());
    open_nodes.pop_back();

    if (lowest_score_node == to) {
      auto last_in_path = to;
      auto path = std::vector<Node*>{};

      while (last_in_path != from) {
        path.push_back(last_in_path);
        last_in_path = node_data[last_in_path].came_from.value();
      }

      path.push_back(from);

      auto cost_to_goal = node_data[to].g_score;
      return {{path, cost_to_goal}, nodes_opened};
    }

    for (const auto& connection : lowest_score_node->GetConnections()) {
      if (node_data[lowest_score_node].came_from == connection) {
        continue;
      }
      auto new_g_score = node_data[lowest_score_node].g_score +
                         lowest_score_node->GetExactCostTo(connection);

      if (node_data[connection].g_score < new_g_score) {
        node_data[connection].g_score = new_g_score;
        // TODO: Don't calculate this every time we re-check the node, once is
        // enough
        node_data[connection].h_score = connection->GetEstimatedCostTo(to);

        node_data[connection].came_from = lowest_score_node;
        open_nodes.push_back(connection);
        std::push_heap(std::begin(open_nodes), std::end(open_nodes),
                       node_score_cmp);
        nodes_opened++;
      }
    }
  }

  return {};
}
}  // namespace Pathfinding
