find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BIN_PATH})

add_executable(delaunay_demo
        delaunay/DelaunayWidget.cpp
        delaunay/main.cpp
)
target_link_libraries(delaunay_demo PRIVATE Qt6::Widgets)
target_link_libraries(delaunay_demo PRIVATE ${LIB_NAME})