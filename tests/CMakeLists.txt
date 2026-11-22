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
    ${MESHBENCH_GMSH_LIB}
  )
  target_include_directories(${name} PRIVATE ${MESHBENCH_GMSH_INCLUDE})
  if(OpenMP_CXX_FOUND)
    target_link_libraries(${name} PRIVATE OpenMP::OpenMP_CXX)
  endif()
  # LeakFix.cpp / MeshOps.cpp are compiled into each test binary.
  target_sources(${name} PRIVATE
    ${CMAKE_SOURCE_DIR}/src/Commands.cpp
    ${CMAKE_SOURCE_DIR}/src/GmshCadSession.cpp
    ${CMAKE_SOURCE_DIR}/src/LeakFix.cpp
    ${CMAKE_SOURCE_DIR}/src/MeshDocument.cpp
    ${CMAKE_SOURCE_DIR}/src/MeshOps.cpp
    ${CMAKE_SOURCE_DIR}/src/Shrinkwrap.cpp
  )
  if(name STREQUAL "tst_mcp")
    target_sources(${name} PRIVATE
      ${CMAKE_SOURCE_DIR}/src/McpHttpServer.cpp
      ${CMAKE_SOURCE_DIR}/src/McpProtocol.cpp
    )
    target_link_libraries(${name} PRIVATE Boost::headers)
  endif()
  if(name STREQUAL "tst_sceneexport")
    target_sources(${name} PRIVATE ${CMAKE_SOURCE_DIR}/src/SceneExport.cpp)
    target_link_libraries(${name} PRIVATE ${VTK_LIBRARIES})
    vtk_module_autoinit(TARGETS ${name} MODULES ${VTK_LIBRARIES})
  endif()
  add_test(NAME ${name} COMMAND ${name})
  # GUI-free: run offscreen so tests pass on headless machines.
  set_tests_properties(${name} PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

add_meshbench_test(tst_leakfix)
add_meshbench_test(tst_meshdocument)
add_meshbench_test(tst_meshops)
add_meshbench_test(tst_shrinkwrap)
add_meshbench_test(tst_commands)
add_meshbench_test(tst_gmshcad)
add_meshbench_test(tst_mcp)
add_meshbench_test(tst_sceneexport)
