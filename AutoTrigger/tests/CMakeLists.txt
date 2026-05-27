# Collect all test source files
file(GLOB TEST_SOURCES "test_*.cpp")

foreach(TEST_SRC ${TEST_SOURCES})
  get_filename_component(TEST_NAME ${TEST_SRC} NAME_WE)
  add_executable(${TEST_NAME} ${TEST_SRC})
  target_link_libraries(${TEST_NAME} PRIVATE autotrigger_core gtest_main gmock_main)
  target_include_directories(${TEST_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
  )
  gtest_discover_tests(${TEST_NAME})
endforeach()
