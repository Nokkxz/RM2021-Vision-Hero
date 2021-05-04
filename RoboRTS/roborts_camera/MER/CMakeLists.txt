project(MER_driver)

#MER_driver
add_library(MER_driver
  SHARED
  MER_driver.cpp
  GxCamera.cpp
)

target_link_libraries(MER_driver
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  libgxiapi.so  
)


### TODO:
target_include_directories(MER_driver
  PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRECTORIES}
)


### TODO: modified it
add_library(driver::MER_driver ALIAS MER_driver)
