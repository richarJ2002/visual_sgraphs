set(g2o_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

SET(G2O_LIB_TYPE SHARED)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Set the output directory for the build executables and libraries
SET(g2o_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# activate warnings !!!
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Generate config.h
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(${g2o_SOURCE_DIR}/config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/g2o_generated/vs_graphs_g2o/config.h)

# Set up the top-level include directories
INCLUDE_DIRECTORIES(
${g2o_SOURCE_DIR}/vs_graphs_g2o/core
${g2o_SOURCE_DIR}/vs_graphs_g2o/types
${g2o_SOURCE_DIR}/vs_graphs_g2o/stuff 
${G2O_EIGEN3_INCLUDE})

# Include the subdirectories
set(VS_GRAPHS_G2O_SOURCES
#types
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_sba.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_six_dof_expmap.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_sba.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_six_dof_expmap.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_seven_dof_expmap.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/types_seven_dof_expmap.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/se3quat.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/se3_ops.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/se3_ops.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/g2o_types_slam3d_api.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/g2o_types_slam3d_addons_api.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/isometry3d_mappings.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/isometry3d_mappings.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/plane3d.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/vertex_plane.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/types/vertex_plane.cpp
#core
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_edge.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_binary_edge.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_graph_action.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_binary_edge.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_graph_action.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_multi_edge.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_graph.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_multi_edge.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_graph.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_unary_edge.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/linear_solver.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_unary_edge.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/marginal_covariance_cholesky.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_vertex.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/marginal_covariance_cholesky.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/base_vertex.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/matrix_structure.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/batch_stats.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/matrix_structure.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/batch_stats.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/openmp_mutex.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/block_solver.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/block_solver.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/parameter.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/parameter.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/cache.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/cache.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimizable_graph.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimizable_graph.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/solver.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/solver.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/creators.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_factory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/estimate_propagator.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_factory.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/estimate_propagator.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/factory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_property.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/factory.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/sparse_block_matrix.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/sparse_optimizer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/sparse_block_matrix.hpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/sparse_optimizer.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_dijkstra.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/hyper_dijkstra.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/parameter_container.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/parameter_container.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_with_hessian.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_with_hessian.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_levenberg.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_levenberg.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_gauss_newton.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/optimization_algorithm_gauss_newton.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/jacobian_workspace.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/jacobian_workspace.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel_factory.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel_factory.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel_impl.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/core/robust_kernel_impl.h
#stuff
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/string_tools.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/color_macros.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/macros.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/timeutil.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/misc.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/timeutil.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/os_specific.c
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/os_specific.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/string_tools.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/property.cpp
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/property.h
  ${CMAKE_CURRENT_LIST_DIR}/vs_graphs_g2o/stuff/opengl_wrapper.h
)

ADD_LIBRARY(g2o ${G2O_LIB_TYPE}
  ${VS_GRAPHS_G2O_SOURCES}
)

# Preserve runtime source-location labels without restoring old include roots.
set(VS_GRAPHS_G2O_MACRO_PREFIX_MAP
  "-fmacro-prefix-map=${g2o_SOURCE_DIR}/vs_graphs_g2o=${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/g2o")
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g2o PUBLIC
    "$<BUILD_INTERFACE:$<$<COMPILE_LANGUAGE:C>:${VS_GRAPHS_G2O_MACRO_PREFIX_MAP}>>")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(g2o PUBLIC
    "$<BUILD_INTERFACE:$<$<COMPILE_LANGUAGE:CXX>:${VS_GRAPHS_G2O_MACRO_PREFIX_MAP}>>")
endif()
