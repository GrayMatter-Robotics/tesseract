find_package(opw_kinematics REQUIRED)

add_library(${PROJECT_NAME}_opw_roe src/opw_roe_inv_kin.cpp)
target_link_libraries(
  ${PROJECT_NAME}_opw_roe
  PUBLIC ${PROJECT_NAME}_core
         Eigen3::Eigen
         tesseract::tesseract_scene_graph
         opw_kinematics::opw_kinematics
         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_opw_roe PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_opw_roe PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw_roe PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw_roe ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw_roe PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw_roe
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_opw_roe PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

add_library(${PROJECT_NAME}_opw_roe_factory src/opw_roe_factory.cpp)
target_link_libraries(${PROJECT_NAME}_opw_roe_factory PUBLIC ${PROJECT_NAME}_opw_roe tesseract::tesseract_scene_graph
                                                         console_bridge::console_bridge)
target_compile_options(${PROJECT_NAME}_opw_roe_factory PRIVATE ${TESSERACT_COMPILE_OPTIONS_PRIVATE})
target_compile_options(${PROJECT_NAME}_opw_roe_factory PUBLIC ${TESSERACT_COMPILE_OPTIONS_PUBLIC})
target_compile_definitions(${PROJECT_NAME}_opw_roe_factory PUBLIC ${TESSERACT_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_opw_roe_factory ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_opw_roe_factory PUBLIC VERSION ${TESSERACT_CXX_VERSION})
target_code_coverage(
  ${PROJECT_NAME}_opw_roe_factory
  PRIVATE
  ALL
  EXCLUDE ${COVERAGE_EXCLUDE}
  ENABLE ${TESSERACT_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_opw_roe_factory PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                              "$<INSTALL_INTERFACE:include>")

# Add factory library so kinematic_factory can find these factories by defauult
set(KINEMATICS_PLUGINS ${KINEMATICS_PLUGINS} "${PROJECT_NAME}_opw_roe_factory" PARENT_SCOPE)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  COMPONENT opw_roe
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")

# Plugin should not be export only installed
install(
  TARGETS ${PROJECT_NAME}_opw_roe_factory
  COMPONENT opw_roe
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

configure_component(
  COMPONENT opw_roe
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME}_opw_roe
  DEPENDENCIES opw_kinematics "tesseract_kinematics COMPONENTS core")

if(TESSERACT_PACKAGE)
  cpack_component(
    COMPONENT opw_roe
    VERSION ${pkg_extracted_version}
    DESCRIPTION "Tesseract Kinematics opw implementation"
    COMPONENT_DEPENDS core
    LINUX_DEPENDS "${TESSERACT_PACKAGE_PREFIX}opw-kinematics"
    WINDOWS_DEPENDS "${TESSERACT_PACKAGE_PREFIX}opw-kinematics")
endif()
