add_executable(shell-app main.cpp PIDController.cpp Ackermann.cpp)
target_link_libraries(shell-app ${Boost_LIBRARIES})
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
)