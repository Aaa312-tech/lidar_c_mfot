add_library(picpr_route_lidar)

target_sources(picpr_route_lidar PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_bitmap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_python_set.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_astar.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_mfot.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/picdb_lidar_view.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_drc.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/lidar_router.cpp
)

target_include_directories(picpr_route_lidar PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(picpr_route_lidar PUBLIC
  picpr_db
  picpr_utility
)
