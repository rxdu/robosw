## Add source directories
# Robots
add_subdirectory(platform)

# Modules
add_subdirectory(common)
add_subdirectory(interface)
add_subdirectory(driver)
#add_subdirectory(mapping)
#add_subdirectory(estimation)
add_subdirectory(control)
#add_subdirectory(planning)

if (ENABLE_VISUALIZATION)
  option(QUICKVIZ_DEV_MODE "Build quickviz in dev mode" ON)
  option(BUILD_QUICKVIZ_APP "Build quickviz application" ON)
  add_subdirectory(quickviz)
endif ()
