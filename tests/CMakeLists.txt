cmake_minimum_required(VERSION 3.21)

find_package(Qt6 REQUIRED COMPONENTS Test)

# Shared cube builders used by the tests.
add_library(meshbench_test_utils STATIC test_utils.cpp)
target_include_directories(meshbench_test_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(meshbench_test_utils PUBLIC CGAL::CGAL)

function(add_meshbench_test name)
  add_executable(${name} ${name}.cpp)
  target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/src)
  target_link_libraries(${name} PRIVATE
    meshbench_test_utils
    Qt6::Test
    CGAL::CGAL
  )
  if(MESHBENCH_GMSH_LIB)
    target_link_libraries(${name} PRIVATE ${MESHBENCH_GMSH_LIB})
    target_include_directories(${name} PRIVATE ${MESHBENCH_GMSH_INCLUDE})
    target_compile_definitions(${name} PRIVATE MESHBENCH_HAVE_GMSH=1)
  endif()
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${name} PRIVATE OpenMP::OpenMP_CXX)
  endif()
  # LeakFix.cpp / MeshOps.cpp are compiled into each test binary.
  target_sources(${name} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/LeakFix.cpp
    ${CMAKE_SOURCE_DIR}/src/MeshOps.cpp
    ${CMAKE_SOURCE_DIR}/src/Shrinkwrap.cpp
  )
  add_test(NAME ${name} COMMAND ${name})
  # GUI-free: run offscreen so tests pass on headless machines.
  set_tests_properties(${name} PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

add_meshbench_test(tst_leakfix)
add_meshbench_test(tst_meshops)
add_meshbench_test(tst_shrinkwrap)
