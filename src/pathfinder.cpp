#include "pathfinder.h"

namespace Pathfinding {
auto Pathfinder::FindPath(Node* from, Node* to, Backend backend) const noexcept
    -> Result {
  switch (backend) {
    case Backend::AStar:
      return FindPath_AStar(from, to);
      break;
  }
}

};  // namespace Pathfinding
