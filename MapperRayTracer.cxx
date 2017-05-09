//============================================================================
//  Copyright (c) Kitware, Inc.
//  All rights reserved.
//  See LICENSE.txt for details.
//  This software is distributed WITHOUT ANY WARRANTY; without even
//  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
//  PURPOSE.  See the above copyright notice for more information.
//
//  Copyright 2016 Sandia Corporation.
//  Copyright 2016 UT-Battelle, LLC.
//  Copyright 2016 Los Alamos National Security.
//
//  Under the terms of Contract DE-AC04-94AL85000 with Sandia Corporation,
//  the U.S. Government retains certain rights in this software.
//
//  Under the terms of Contract DE-AC52-06NA25396 with Los Alamos National
//  Laboratory (LANL), the U.S. Government retains certain rights in
//  this software.
//============================================================================

#include "MapperRayTracer.h"

#include <vtkm/cont/TryExecute.h>

#include <vtkm/cont/internal/SimplePolymorphicContainer.h>

#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/rendering/internal/RunTriangulator.h>
#include "RayTracer.h"
#include <vtkm/rendering/raytracing/Camera.h>


struct MapperRayTracer::InternalsType
{
  vtkm::rendering::CanvasRayTracer *Canvas;
  std::shared_ptr<vtkm::cont::internal::SimplePolymorphicContainerBase>
      RayTracerContainer;

  std::shared_ptr<vtkm::cont::internal::SimplePolymorphicContainerBase>
      TreeContainer;

  VTKM_CONT_EXPORT
  InternalsType()
    : Canvas(nullptr)
  {  }

  template<typename Device>
  VTKM_CONT_EXPORT
  RayTracer<Device> *GetRayTracer(Device)
  {
    VTKM_IS_DEVICE_ADAPTER_TAG(Device);

    typedef RayTracer<Device> RayTracerType;
    typedef vtkm::cont::internal::SimplePolymorphicContainer<RayTracerType>
        ContainerType;
    RayTracerType *tracer = NULL;
    if (this->RayTracerContainer)
    {
      ContainerType *container =
          dynamic_cast<ContainerType *>(this->RayTracerContainer.get());
      if (container)
      {
        tracer = &container->Item;
      }
    }

    if (tracer == NULL)
    {
      ContainerType *container
          = new vtkm::cont::internal::SimplePolymorphicContainer<RayTracerType>;
      tracer = &container->Item;
      this->RayTracerContainer.reset(container);
    }

    return tracer;
  }

  template<typename DeviceAdapter>
  VTKM_CONT_EXPORT
  Tree<DeviceAdapter> *GetTree(DeviceAdapter)
  {
    VTKM_IS_DEVICE_ADAPTER_TAG(DeviceAdapter);

    typedef Tree<DeviceAdapter> TreeType;
    typedef vtkm::cont::internal::SimplePolymorphicContainer<TreeType>
        ContainerType;
    TreeType *treePtr = NULL;
    if (this->TreeContainer)
    {
      ContainerType *container =
          dynamic_cast<ContainerType *>(this->TreeContainer.get());
      if (container)
      {
        treePtr = &container->Item;
      }
    }

    if (treePtr == NULL)
    {
      ContainerType *container
          = new vtkm::cont::internal::SimplePolymorphicContainer<TreeType>;
      treePtr = &container->Item;
      this->TreeContainer.reset(container);
    }

    return treePtr;
  }

};

MapperRayTracer::MapperRayTracer(const vtkm::cont::DynamicCellSet &cells,
                                 vtkm::cont::ArrayHandle<vtkm::UInt32 > &cntArray,
                                 vtkm::cont::ArrayHandle<vtkm::UInt32 > &idxArray,
                                 vtkm::cont::ArrayHandle<vtkm::UInt32 > &vtxArray
                                 )

  : Internals(new InternalsType),
    Cells(cells),
    CntArray(cntArray),
    IdxArray(idxArray),
    VtxArray(vtxArray)
{  }

MapperRayTracer::~MapperRayTracer()
{  }

void MapperRayTracer::SetCanvas(vtkm::rendering::Canvas *canvas)
{
  if(canvas != nullptr)
  {
    this->Internals->Canvas = dynamic_cast<vtkm::rendering::CanvasRayTracer*>(canvas);
    if(this->Internals->Canvas == nullptr)
    {
      throw vtkm::cont::ErrorBadType(
        "Ray Tracer: bad canvas type. Must be CanvasRayTracer");
    }
  }
  else
  {
    this->Internals->Canvas = nullptr;
  }
}

struct MapperRayTracer::RenderFunctor
{
  MapperRayTracer *Self;
  vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Id, 4> > TriangleIndices;
  vtkm::Id NumberOfTriangles;
  vtkm::cont::CoordinateSystem Coordinates;
  vtkm::cont::Field ScalarField;

  vtkm::rendering::Camera Camera;
  vtkm::Range ScalarRange;
  vtkm::cont::DynamicCellSet Cells;

  vtkm::cont::ArrayHandle<vtkm::UInt32 > CntArray;
  vtkm::cont::ArrayHandle<vtkm::UInt32 > IdxArray;
  vtkm::cont::ArrayHandle<vtkm::UInt32 > VtxArray;
  VTKM_CONT_EXPORT
  RenderFunctor(MapperRayTracer *self,
                const vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Id,4> > &indices,
                vtkm::Id numberOfTriangles,
                const vtkm::cont::CoordinateSystem &coordinates,
                const vtkm::cont::Field &scalarField,

                const vtkm::cont::DynamicCellSet &cells,
                const vtkm::rendering::Camera &camera,
                const vtkm::Range &scalarRange,
                vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray,
                vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray,
                vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray)
    : Self(self),
      TriangleIndices(indices),
      NumberOfTriangles(numberOfTriangles),
      Coordinates(coordinates),
      ScalarField(scalarField),
      Camera(camera),
      ScalarRange(scalarRange),
      Cells(cells),
      CntArray(cntArray),
      IdxArray(idxArray),
      VtxArray(vtxArray)
  {  }

  template<typename Device>
  bool operator()(Device)
  {
    VTKM_IS_DEVICE_ADAPTER_TAG(Device);

    RayTracer<Device> *tracer =
        this->Self->Internals->GetRayTracer(Device());

    Tree<Device> *treePtr = this->Self->Internals->GetTree(Device());
    tracer->GetCamera().SetParameters(this->Camera,
                                      *this->Self->Internals->Canvas);

    vtkm::Bounds dataBounds = this->Coordinates.GetBounds(Device());

    treePtr->SetCnt(CntArray);
    treePtr->SetIdx(IdxArray);
    treePtr->SetVtx(VtxArray);
    tracer->SetData(this->Coordinates.GetData(),
                    this->TriangleIndices,
                    this->ScalarField,
                    this->NumberOfTriangles,
                    this->ScalarRange,
                    dataBounds,
                    this->Cells,
                    treePtr);
    tracer->SetColorMap(this->Self->ColorMap);
    tracer->SetBackgroundColor(
          this->Self->Internals->Canvas->GetBackgroundColor().Components);
    tracer->Render(this->Self->Internals->Canvas);

    return true;
  }
};

void MapperRayTracer::RenderCells(
    const vtkm::cont::DynamicCellSet &cellset,
    const vtkm::cont::CoordinateSystem &coords,
    const vtkm::cont::Field &scalarField,
    const vtkm::rendering::ColorTable &vtkmNotUsed(colorTable),
    const vtkm::rendering::Camera &camera,
    const vtkm::Range &scalarRange)
{
  vtkm::cont::ArrayHandle< vtkm::Vec<vtkm::Id, 4> >  indices;
  vtkm::Id numberOfTriangles;
  vtkm::rendering::internal::RunTriangulator(
        cellset, indices, numberOfTriangles);

  RenderFunctor functor(this,
                        indices,
                        numberOfTriangles,
                        coords,
                        scalarField,

                        this->Cells,
                        camera,
                        scalarRange,
                        CntArray,
                        IdxArray,
                        VtxArray);
  vtkm::cont::TryExecute(functor);
}

void MapperRayTracer::StartScene()
{
  // Nothing needs to be done.
}

void MapperRayTracer::EndScene()
{
  // Nothing needs to be done.
}

vtkm::rendering::Mapper *MapperRayTracer::NewCopy() const
{
  return new MapperRayTracer(*this);
}

vtkm::rendering::Canvas *
MapperRayTracer::GetCanvas() const
{
  return this->Internals->Canvas;
}
