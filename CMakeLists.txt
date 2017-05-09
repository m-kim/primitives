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
PATHS "/home/mkim/local/vtkm/debug//lib"
  OPTIONAL_COMPONENTS Rendering Serial 
  )

if(VTKm_Rendering_FOUND)
  add_executable(parametric_rendering main.cpp Camera.h Ray.h TriangleIntersector.h RayTracer.h MapperRayTracer.cxx)
  target_include_directories(parametric_rendering PRIVATE 	${VTKm_INCLUDE_DIRS})
  target_link_libraries(parametric_rendering ${VTKm_LIBRARIES})
  target_compile_options(parametric_rendering PRIVATE ${VTKm_COMPILE_OPTIONS})
endif()
