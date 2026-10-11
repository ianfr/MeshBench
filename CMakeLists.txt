cmake_minimum_required(VERSION 3.21)
project(MeshBench VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

option(MESHBENCH_ENABLE_GMSH "Enable Gmsh-based operations (requires Gmsh C++ API)" ON)
option(MESHBENCH_BUILD_TESTS "Build unit tests" ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGLWidgets)
find_package(VTK REQUIRED COMPONENTS
  CommonCore
  CommonDataModel
  FiltersSources
  FiltersCore
  IOGeometry
  RenderingCore
  RenderingOpenGL2
  RenderingAnnotation
  InteractionStyle
  GUISupportQt
  RenderingQt
)
find_package(CGAL REQUIRED)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  message(STATUS "MeshBench: OpenMP enabled (${OpenMP_CXX_VERSION})")
endif()

# NB: we link the libgmsh file directly rather than the gmsh::shared imported
# target, whose INTERFACE_LINK_LIBRARIES names .so dev symlinks (OpenCASCADE
# TK*, metis, GLU) that are not installed here. The versioned runtime libs
# those names resolve to are present, so direct linking works.
if(MESHBENCH_ENABLE_GMSH)
  find_package(gmsh CONFIG QUIET)
  find_library(MESHBENCH_GMSH_LIB NAMES gmsh)
  find_path(MESHBENCH_GMSH_INCLUDE NAMES gmsh.h)
endif()
if(MESHBENCH_GMSH_LIB AND MESHBENCH_GMSH_INCLUDE)
  message(STATUS "MeshBench: Gmsh support enabled (${MESHBENCH_GMSH_LIB})")
else()
  message(STATUS "MeshBench: Gmsh library/headers not found, support disabled")
  unset(MESHBENCH_GMSH_LIB CACHE)
  set(MESHBENCH_GMSH_LIB "")
endif()

set(MESHBENCH_SOURCES
  src/main.cpp
  src/MainWindow.cpp
  src/MeshDocument.cpp
  src/MeshOps.cpp
  src/LeakFix.cpp
  src/Shrinkwrap.cpp
  src/VtkView.cpp
)

add_executable(meshbench ${MESHBENCH_SOURCES})
target_include_directories(meshbench PRIVATE src)
target_link_libraries(meshbench PRIVATE
  Qt6::Widgets
  Qt6::OpenGLWidgets
  ${VTK_LIBRARIES}
  CGAL::CGAL
)
if(MESHBENCH_GMSH_LIB)
  target_link_libraries(meshbench PRIVATE ${MESHBENCH_GMSH_LIB})
  target_include_directories(meshbench PRIVATE ${MESHBENCH_GMSH_INCLUDE})
  target_compile_definitions(meshbench PRIVATE MESHBENCH_HAVE_GMSH=1)
endif()
if(OpenMP_CXX_FOUND)
  target_link_libraries(meshbench PRIVATE OpenMP::OpenMP_CXX)
endif()

vtk_module_autoinit(
  TARGETS meshbench
  MODULES ${VTK_LIBRARIES}
)

if(MESHBENCH_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
