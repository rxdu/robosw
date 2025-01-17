#add_executable(app_swerve_motion_visualizer
#    app_swerve_motion_visualizer.cpp
#    info_planel.cpp
#    motion_drawer.cpp
#    swerve_motion_visualizer.cpp)
#target_link_libraries(app_swerve_motion_visualizer PRIVATE
#    swervebot imview logging
#)

add_executable(app_sbot_controller app_sbot_controller.cpp)
target_link_libraries(app_sbot_controller PRIVATE
    swervebot logging
)
