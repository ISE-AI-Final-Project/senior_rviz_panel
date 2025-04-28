cmake_minimum_required(VERSION 3.8)
project(senior_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)

# Separate moc wrapping
qt5_wrap_cpp(COMMAND_PANEL_MOC_FILES include/senior_rviz_panel/command_panel.hpp)
qt5_wrap_cpp(LABEL_PANEL_MOC_FILES include/senior_rviz_panel/label_panel.hpp)

# Command Panel
add_library(command_panel src/command_panel.cpp ${COMMAND_PANEL_MOC_FILES})
target_include_directories(command_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(command_panel
  pluginlib
  rviz_common
)

# Label Panel
add_library(label_panel src/label_panel.cpp ${LABEL_PANEL_MOC_FILES})
target_include_directories(label_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(label_panel
  pluginlib
  rviz_common
)

# Install targets
install(TARGETS command_panel label_panel
        EXPORT export_senior_rviz_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/
        DESTINATION include
)

install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

install(FILES icons/classes/DemoPanel.png
        DESTINATION share/${PROJECT_NAME}/icons/classes
)

ament_export_include_directories(include)
ament_export_targets(export_senior_rviz_panel)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()
