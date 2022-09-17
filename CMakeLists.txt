cmake_minimum_required(VERSION 3.0.2)
project(action_runtime)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

add_message_files(FILES Action.msg Condition.msg)

add_service_files(
  FILES
  ActionTrigger.srv
  CreatePlan.srv
  GetPlan.srv
  LoadPDDL.srv
  Observation.srv
  RegisterAction.srv
  RegisterCondition.srv
  RunAction.srv
  RunPlan.srv)

generate_messages(DEPENDENCIES)

catkin_package(CATKIN_DEPENDS message_generation rospy)

include_directories(${catkin_INCLUDE_DIRS})
