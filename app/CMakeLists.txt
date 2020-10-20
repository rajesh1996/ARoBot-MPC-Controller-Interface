add_executable(shell-app main.cpp PIDController.cpp Ackermann.cpp)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)
