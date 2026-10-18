cmake_minimum_required(VERSION 3.8)
project(r1_description)

find_package(ament_cmake REQUIRED)

# 원본: third_party/unitree_ros/robots/r1_description (env/clone_third_party.sh 로 받음, 수정하지 않음)
# 전신 R1 (다리 12, 허리 roll/yaw, 팔 5x2, 머리 2). 관절 이름·범위는 시뮬 모델과 일치 (docs/r1_model.md).
set(R1_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/unitree_ros/robots/r1_description)
if(NOT EXISTS ${R1_SRC}/R1.urdf)
  message(FATAL_ERROR "R1.urdf 없음: ${R1_SRC}\n먼저 bash env/clone_third_party.sh 를 실행하세요.")
endif()

# 메시 경로를 ROS 패키지 경로로 바꿔 빌드 디렉토리에 생성
file(READ ${R1_SRC}/R1.urdf URDF_TEXT)
string(REPLACE "filename=\"meshes/" "filename=\"package://r1_description/meshes/" URDF_TEXT "${URDF_TEXT}")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/r1.urdf "${URDF_TEXT}")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${R1_SRC}/R1.urdf)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/r1.urdf DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY ${R1_SRC}/meshes/ DESTINATION share/${PROJECT_NAME}/meshes)

ament_package()
