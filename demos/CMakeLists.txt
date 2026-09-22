find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})

add_executable(delaunay_demo
        delaunay/DelaunayWidget.cpp
        delaunay/main.cpp
)
target_link_libraries(delaunay_demo PRIVATE Qt6::Widgets)
target_link_libraries(delaunay_demo PRIVATE ${LIB_NAME})

add_executable(voronoi_demo
        voronoi/VoronoiWidget.cpp
        voronoi/main.cpp
)
target_link_libraries(voronoi_demo PRIVATE Qt6::Widgets)
target_link_libraries(voronoi_demo PRIVATE ${LIB_NAME})

add_executable(convex_hull_2d_demo
        convex_hull_2d/ConvexHullWidget.cpp
        convex_hull_2d/main.cpp
)
target_link_libraries(convex_hull_2d_demo PRIVATE Qt6::Widgets)
target_link_libraries(convex_hull_2d_demo PRIVATE ${LIB_NAME})

add_executable(circuit_quad_tree_demo
        circuit_quad_tree/CircuitQuadTreeWidget.cpp
        circuit_quad_tree/CircuitSegmentValidator.cpp
        circuit_quad_tree/main.cpp
)
target_link_libraries(circuit_quad_tree_demo PRIVATE Qt6::Widgets)
target_link_libraries(circuit_quad_tree_demo PRIVATE ${LIB_NAME})