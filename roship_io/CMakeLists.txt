cmake_minimum_required(VERSION 3.8)

################################
##  CONFIGURE THIS FILE HERE  ##
################################

set(ROS_DEPENDS
    ament_cmake
    rclcpp
    std_msgs
    io_interfaces
  )

set(EXTERNAL_DEPENDS

  )

# Setup PkgConfig Modules
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)
pkg_check_modules(LIBSERIALPORT REQUIRED libserialport)
pkg_check_modules(MOSQUITTO REQUIRED libmosquitto)
include_directories(${LIBMODBUS_INCLUDE_DIRS} ${LIBSERIALPORT_INCLUDE_DIRS} ${MOSQUITTO_INCLUDE_DIRS})

set(EXTERNAL_DEPENDS_LIB
  ${LIBMODBUS_LIBRARIES}
  ${LIBSERIALPORT_LIBRARIES}
  ${MOSQUITTO_LIBRARIES}
)

###
###  Avoid editing below this line
###

get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message("Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in

add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
)

###########
## Build ##
###########

set(EXECUTABLES "")  # for installing later

message("buliding ${PROJECT_NAME} node: ")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  message("  - " ${NODE_NAME})
  list(APPEND EXECUTABLES ${NODE_NAME})
  add_executable(${NODE_NAME} ${NODE_FILE} ${HDRS} ${SRCS})
  target_include_directories(${NODE_NAME} PRIVATE "include/${PROJECT_NAME}/")
  target_link_libraries(${NODE_NAME} ${EXTERNAL_DEPENDS_LIB} )
  ament_target_dependencies(${NODE_NAME} ${ROS_DEPENDS})
endforeach(NODE_FILE)

message("  Installing: " ${EXECUTABLES})
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)
install(PROGRAMS
  scripts/replay_pcap.sh
  scripts/test_mqtt.sh
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


#############
## library ##
#############

include_directories(include/${PROJECT_NAME}  ${LIBMODBUS_LIBRARIES} ${LIBSERIALPORT_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${SRCS} ${HDRS})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDS})
ament_export_dependencies(${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
ament_export_include_directories(include)

target_link_libraries(${PROJECT_NAME} ${EXTERNAL_DEPENDS_LIB})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
