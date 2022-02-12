#include <optional>

#include "pathfinder.h"

class AStar final : public Pathfinder {
 public:
  AStar() = default;
  ~AStar() final = default;

  AStar(const AStar&) = default;
  AStar& operator=(const AStar&) = default;

  AStar(AStar&&) = default;
  AStar& operator=(AStar&&) = default;

  [[nodiscard]] auto FindPath(Node* from, Node* to) const noexcept
      -> std::optional<std::vector<Node*>> final;

 private:
  struct NodeData {
    int h_score{};
    int g_score{};

    std::optional<Node*> came_from{};
  };
};
