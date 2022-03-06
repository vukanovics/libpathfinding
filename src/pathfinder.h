#pragma once

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace Pathfinding {
class Pathfinder {
 public:
  virtual ~Pathfinder() = default;

  Pathfinder() = default;

  Pathfinder(const Pathfinder&) = default;
  Pathfinder& operator=(const Pathfinder&) = default;

  Pathfinder(Pathfinder&&) = default;
  Pathfinder& operator=(Pathfinder&&) = default;

  enum class Backend { AStar };
  //!
  //! Point with connections to other points.
  //!
  //! Stored by Pathfinder in the nodes vector.
  //!
  //! Destructor will delete connections from other node instances to this one.
  //! Cannot be copied nor moved.
  //!
  class Node {
   public:
    Node() = default;

    virtual ~Node() = default;

    Node(const Node&) = delete;
    Node(Node&&) = delete;

    Node& operator=(const Node&) = delete;
    Node& operator=(Node&&) = delete;

    //!
    //! Get node's connections.
    //!
    //! @return Const reference to a vector of pointers to connected
    //! nodes.
    //!
    [[nodiscard]] inline auto GetConnections() const noexcept -> const auto& {
      return connections;
    }
    //!
    //! Add a connection to this node.
    //!
    //! @param to Pointer to the other node.
    //!
    inline void AddConnection(Node* to) noexcept { connections.push_back(to); }
    //!
    //! Remove a connection from this node.
    //!
    //! @param to Pointer to the other node.
    //!
    inline void RemoveConnection(Node* to) noexcept {
      connections.erase(
          std::remove(std::begin(connections), std::end(connections), to),
          std::end(connections));
    }
    //!
    //! Returns the exact cost to get from this node to the one given.
    //!
    //! @return Cost from this -> to
    //!
    [[nodiscard]] virtual auto GetExactCostTo(Node* to) const -> float = 0;
    //!
    //! Returns the estimated cost to get from this node to the one given.
    //!
    //! @return Cost from this -> to
    //!
    [[nodiscard]] virtual auto GetEstimatedCostTo(Node* to) const -> float = 0;

   private:
    std::vector<Node*> connections;
  };

  struct Path {
    Path(std::vector<Node*> nodes, float length)
        : nodes(std::move(nodes)), length(length) {}
    std::vector<Node*> nodes = {};
    float length = 0.0f;
  };

  struct Result {
    Result() = default;
    Result(Path path, int nodes_opened)
        : path(std::move(path)), nodes_opened(nodes_opened) {}
    std::optional<Path> path{};
    int nodes_opened = 0;
  };

  //!
  //! Add a node to the nodes vector.
  //!
  //! @return Pointer to the newly added node.
  //!
  [[nodiscard]] inline auto AddNode(std::unique_ptr<Node> node) noexcept
      -> Node* {
    nodes.push_back(std::move(node));
    return nodes.back().get();
  };
  //!
  //! Tries to find a path between two nodes.
  //!
  //! @param from Pointer to the starting Node.
  //! @param to Pointer to the goal Node.
  //! @param backend Which backend to use from the Backend enum.
  //!
  //! @return Result of the pathfinding attempt.
  //!
  [[nodiscard]] auto FindPath(Node* from, Node* to,
                              Backend backend) const noexcept -> Result;

  //!
  //! Returns a const ref to the vector of nodes.
  //!
  //! @return Const ref to the vector of node unique pointers.
  //!
  [[nodiscard]] inline const auto& GetNodes() const noexcept { return nodes; }

  //!
  //! Removes a node matching the given node pointer from the pathfinder.
  //!
  //! @param node Node to remove.
  //!
  inline void RemoveNode(Node* node) noexcept {
    // remove all connections to this node
    std::for_each(std::begin(nodes), std::end(nodes),
                  [&node](auto& n) { n->RemoveConnection(node); });
    // and then remove it from storage
    nodes.erase(
        std::remove_if(std::begin(nodes), std::end(nodes),
                       [&node](auto const& e) { return e.get() == node; }),
        std::end(nodes));
  }

 private:
  [[nodiscard]] auto FindPath_AStar(Node* from, Node* to) const noexcept
      -> Result;
  std::vector<std::unique_ptr<Node>> nodes{};
};
}  // namespace Pathfinding
