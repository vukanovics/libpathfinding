#include <gtest/gtest.h>

#include <cmath>

#include "../src/pathfinder.h"

using namespace Pathfinding;

class TestNode : public Pathfinder::Node {
 public:
  TestNode(std::pair<float, float> position) : position(std::move(position)) {}

  [[nodiscard]] auto GetExactCostTo(Node* to) const -> float final {
    if (to == nullptr) {
      throw std::invalid_argument("GetExactCostTo: 'to' is nullptr");
    }
    auto to_cast = dynamic_cast<TestNode*>(to);
    if (to_cast == nullptr) {
      throw std::invalid_argument("GetExactCostTo: 'to' isn't a TestNode*");
    }

    auto pos1 = position;
    auto pos2 = to_cast->GetPosition();

    constexpr float exp = 2.0f;

    return sqrtf(powf(pos1.first - pos2.first, exp) +
                 powf(pos1.second - pos2.second, exp));
  }

  [[nodiscard]] auto GetEstimatedCostTo(Node* to) const -> float final {
    return GetExactCostTo(to);
  };

  [[nodiscard]] std::pair<float, float> GetPosition() const noexcept {
    return position;
  }

 private:
  std::pair<float, float> position{};
};

TEST(AStar, TestNode) {
  {
    auto node1 = TestNode({0.0f, 0.0f});

    ASSERT_FLOAT_EQ(node1.GetPosition().first, 0.0f);
    ASSERT_FLOAT_EQ(node1.GetPosition().second, 0.0f);
  }
  {
    auto node1 = TestNode({5.0f, 2.5f});

    ASSERT_FLOAT_EQ(node1.GetPosition().first, 5.0f);
    ASSERT_FLOAT_EQ(node1.GetPosition().second, 2.5f);
  }
  {
    auto node1 = TestNode({0.0f, 0.0f});
    auto node2 = TestNode({-5.0f, 0.0f});
    auto cost = node1.GetExactCostTo(&node2);

    ASSERT_FLOAT_EQ(cost, 5.0f);
  }
  {
    auto node1 = TestNode({0.0f, 0.0f});
    auto node2 = TestNode({0.0f, 0.0f});
    auto cost = node1.GetExactCostTo(&node2);

    ASSERT_FLOAT_EQ(cost, 0.0f);
  }
  {
    auto node1 = TestNode({2.5f, 2.5f});
    auto node2 = TestNode({0.0f, 0.0f});
    auto cost = node1.GetExactCostTo(&node2);

    ASSERT_FLOAT_EQ(cost, 3.53553339f);
  }
}
TEST(AStar, SimplePath) {
  {
    auto node1 = std::make_unique<TestNode>(std::make_pair(0.0f, 0.0f));
    auto node2 = std::make_unique<TestNode>(std::make_pair(5.0f, 0.0f));
    auto node3 = std::make_unique<TestNode>(std::make_pair(10.0f, 0.0f));

    auto expected_length =
        node1->GetExactCostTo(node2.get()) + node2->GetExactCostTo(node3.get());

    auto pathfinder = Pathfinder{};
    auto node1_ptr = pathfinder.AddNode(std::move(node1));
    auto node2_ptr = pathfinder.AddNode(std::move(node2));
    auto node3_ptr = pathfinder.AddNode(std::move(node3));

    node1_ptr->AddConnection(node2_ptr);
    node2_ptr->AddConnection(node1_ptr);

    node2_ptr->AddConnection(node3_ptr);
    node3_ptr->AddConnection(node2_ptr);

    auto result =
        pathfinder.FindPath(node1_ptr, node3_ptr, Pathfinder::Backend::AStar);

    ASSERT_TRUE(result.path.has_value());

    auto path = result.path.value();
    auto nodes = path.nodes;

    ASSERT_EQ(nodes.size(), 3);
    ASSERT_EQ(result.nodes_opened, 3);

    ASSERT_FLOAT_EQ(path.length, expected_length);

    ASSERT_EQ(nodes[0], node3_ptr);
    ASSERT_EQ(nodes[1], node2_ptr);
    ASSERT_EQ(nodes[2], node1_ptr);
  }
}

TEST(AStar, BranchingPath) {
  {
    auto node1 = std::make_unique<TestNode>(std::make_pair(0.0f, 0.0f));
    auto node2 = std::make_unique<TestNode>(std::make_pair(1.0f, 1.0f));
    auto node3 = std::make_unique<TestNode>(std::make_pair(2.0f, 1.0f));
    auto node4 = std::make_unique<TestNode>(std::make_pair(3.0f, 0.0f));
    auto node5 = std::make_unique<TestNode>(std::make_pair(2.0f, -4.0f));

    auto expected_length = node1->GetExactCostTo(node2.get()) +
                           node2->GetExactCostTo(node3.get()) +
                           node3->GetExactCostTo(node4.get());

    auto pathfinder = Pathfinder{};
    auto node1_ptr = pathfinder.AddNode(std::move(node1));
    auto node2_ptr = pathfinder.AddNode(std::move(node2));
    auto node3_ptr = pathfinder.AddNode(std::move(node3));
    auto node4_ptr = pathfinder.AddNode(std::move(node4));
    auto node5_ptr = pathfinder.AddNode(std::move(node5));

    node1_ptr->AddConnection(node2_ptr);
    node2_ptr->AddConnection(node1_ptr);

    node2_ptr->AddConnection(node3_ptr);
    node3_ptr->AddConnection(node2_ptr);

    node3_ptr->AddConnection(node4_ptr);
    node4_ptr->AddConnection(node3_ptr);

    node1_ptr->AddConnection(node5_ptr);
    node5_ptr->AddConnection(node1_ptr);

    node5_ptr->AddConnection(node4_ptr);
    node4_ptr->AddConnection(node5_ptr);

    auto result =
        pathfinder.FindPath(node1_ptr, node4_ptr, Pathfinder::Backend::AStar);

    ASSERT_TRUE(result.path.has_value());

    auto path = result.path.value();
    auto nodes = path.nodes;

    ASSERT_EQ(nodes.size(), 4);
    ASSERT_EQ(result.nodes_opened, 5);

    ASSERT_FLOAT_EQ(path.length, expected_length);

    ASSERT_EQ(nodes[0], node4_ptr);
    ASSERT_EQ(nodes[1], node3_ptr);
    ASSERT_EQ(nodes[2], node2_ptr);
    ASSERT_EQ(nodes[3], node1_ptr);
  }
}
