set(CMAKE_CXX_CLANG_TIDY "clang-tidy;-header-filter=./src/*.h;--extra-arg-before=-std=c++17")

add_library(
  libpathfinding
  astar.cpp
)
