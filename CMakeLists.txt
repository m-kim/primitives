##=============================================================================
##
##  Copyright (c) Kitware, Inc.
##  All rights reserved.
##  See LICENSE.txt for details.
##
##  This software is distributed WITHOUT ANY WARRANTY; without even
##  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
##  PURPOSE.  See the above copyright notice for more information.
##
##  Copyright 2015 Sandia Corporation.
##  Copyright 2015 UT-Battelle, LLC.
##  Copyright 2015 Los Alamos National Security.
##
##  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
##  the U.S. Government retains certain rights in this software.
##  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
##  Laboratory (LANL), the U.S. Government retains certain rights in
##  this software.
##
##=============================================================================
cmake_minimum_required(VERSION 3.3)
project(csg_rendering)


#Find the VTK-m package
find_package(VTKm REQUIRED
PATHS "/home/mkim/local/vtkm/release/lib"
  OPTIONAL_COMPONENTS Serial TBB CUDA Rendering
  )


if(VTKm_Rendering_FOUND)

  set(PARAMETRIC_SRC
    main.cpp
    MapperRayTracer.cxx
  )
  set(PARAMETRIC_HDR
    Camera.h
    Tree.h
    TreeIntersector.h
    Ray.h
    TriangleIntersector.h
    RayTracer.h
  )
  add_executable(parametric_rendering_Serial ${PARAMETRIC_SRC} ${PARAMETRIC_HDR})

  target_include_directories(parametric_rendering_Serial PRIVATE 	${VTKm_INCLUDE_DIRS})
  target_link_libraries(parametric_rendering_Serial ${VTKm_LIBRARIES})
  target_compile_options(parametric_rendering_Serial PRIVATE ${VTKm_COMPILE_OPTIONS})
  target_compile_definitions(parametric_rendering_Serial PRIVATE
                         "VTKM_DEVICE_ADAPTER=VTKM_DEVICE_ADAPTER_SERIAL")

  if(VTKm_CUDA_FOUND)
    set(old_nvcc_flags ${CUDA_NVCC_FLAGS})
    set(old_cxx_flags ${CMAKE_CXX_FLAGS})
    vtkm_setup_nvcc_flags( old_nvcc_flags old_cxx_flags)
    vtkm_disable_troublesome_thrust_warnings()

    # Cuda compiles do not respect target_include_directories
    cuda_include_directories(${VTKm_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

    set (cudaSource  "${CMAKE_CURRENT_BINARY_DIR}/MapperRayTracer.cu" "${CMAKE_CURRENT_BINARY_DIR}/main.cu")
    configure_file(MapperRayTracer.cxx "${CMAKE_CURRENT_BINARY_DIR}/MapperRayTracer.cu" COPYONLY)
    configure_file(main.cpp "${CMAKE_CURRENT_BINARY_DIR}/main.cu" COPYONLY)
    cuda_add_executable(parametric_rendering_CUDA ${cudaSource})
    target_include_directories(parametric_rendering_CUDA PRIVATE ${VTKm_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(parametric_rendering_CUDA PRIVATE ${VTKm_LIBRARIES})
    target_compile_options(parametric_rendering_CUDA PRIVATE ${VTKm_COMPILE_OPTIONS})
     set(CUDA_NVCC_FLAGS ${old_nvcc_flags})
    set(CMAKE_CXX_FLAGS ${old_cxx_flags})
  endif()

  if(VTKm_TBB_FOUND)
      add_executable(parametric_rendering_TBB ${PARAMETRIC_SRC} ${PARAMETRIC_HDR})
      target_include_directories(parametric_rendering_TBB PRIVATE ${VTKm_INCLUDE_DIRS})
      target_link_libraries(parametric_rendering_TBB PRIVATE ${VTKm_LIBRARIES})
      target_compile_options(parametric_rendering_TBB PRIVATE ${VTKm_COMPILE_OPTIONS})
      target_compile_definitions(parametric_rendering_TBB PRIVATE "VTKM_DEVICE_ADAPTER=VTKM_DEVICE_ADAPTER_TBB")

  endif()
endif()


