#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetFieldAdd.h>
#include <vtkm/rendering/CanvasRayTracer.h>
#include <vtkm/io/writer/VTKDataSetWriter.h>
#include <vtkm/io/reader/VTKDataSetReader.h>

#include "MapperRayTracer.h"
#include "Tree.h"

#include <vtkm/rendering/View3D.h>
#include <vtkm/rendering/ColorTable.h>
#include <iostream>
#include <fstream>
#

typedef VTKM_DEFAULT_DEVICE_ADAPTER_TAG DeviceAdapter;

std::shared_ptr<vtkm::cont::DataSetBuilderExplicitIterative> dataSetBuilder;
vtkm::cont::DataSet csg;
std::vector<vtkm::UInt32> child_idx, child_cnt, child_vtx, child_idx_vtx;
std::shared_ptr<Tree<DeviceAdapter>> treePtr;
vtkm::cont::DataSetFieldAdd dataSetFieldAdd;
vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray, vtxArray, idxArray;
vtkm::rendering::View3D *view;

void load(const char *fn)
{
	vtkm::io::reader::VTKDataSetReader reader(fn);
	reader.ReadDataSet();
	csg = reader.GetDataSet();
	std::cout << "finished reader" << std::endl;
	std::ifstream cntf;
	cntf.open("count.dat", std::ios::binary);
	size_t size = 0;
	cntf.read((char*)&size, sizeof(size_t));
	child_cnt.resize(size);
	cntf.read((char*)&child_cnt[0], sizeof(vtkm::UInt32) * size);

	std::ifstream idxf;
	idxf.open("index.dat", std::ios::binary);
	size = 0;
	idxf.read((char*)&size, sizeof(size_t));
	child_idx.resize(size);
	idxf.read((char*)&child_idx[0], sizeof(vtkm::UInt32) * size);

	std::ifstream vtxf;
	vtxf.open("vertex.dat", std::ios::binary);
	size = 0;
	vtxf.read((char*)&size, sizeof(size_t));
	child_vtx.resize(size);
	vtxf.read((char*)&child_vtx[0], sizeof(vtkm::UInt32) * size);
	

	cntArray = vtkm::cont::make_ArrayHandle(&child_cnt[0], child_cnt.size());
    vtxArray = vtkm::cont::make_ArrayHandle(&child_vtx[0], child_vtx.size());
    idxArray = vtkm::cont::make_ArrayHandle(&child_idx[0], child_idx.size());


    treePtr->SetCnt(cntArray);
    treePtr->SetVtx(vtxArray);
    treePtr->SetIdx(idxArray);

}

int main(int argc, char **argv)
  {

    treePtr = std::shared_ptr<Tree<DeviceAdapter>>(new Tree<DeviceAdapter>());
	std::cout << "start" << std::endl;
    load("dataset.vtk");
	std::cout << "done load" << std::endl;

	std::cout << "done setting the table" << std::endl;

    vtkm::rendering::Color bg(0.2f, 0.2f, 0.2f, 1.0f);
    vtkm::rendering::CanvasRayTracer canvas(512,512);
    MapperRayTracer mapper(csg.GetCellSet(), cntArray, idxArray, vtxArray );

    vtkm::rendering::Scene scene;
    scene.AddActor(vtkm::rendering::Actor(csg.GetCellSet(),
                                          csg.GetCoordinateSystem(),
                                          csg.GetField("radius"),
                                          vtkm::rendering::ColorTable("thermal")));

    //Create vtkm rendering stuff.
    //vtkm::rendering::Camera new_cam;
    //vtkm::Bounds spatialBounds = scene.GetSpatialBounds();
    //new_cam.SetPosition(vtkm::Vec<vtkm::Float32, 3>(0,1,0));
    //new_cam.SetViewUp(vtkm::Vec<vtkm::Float32,3>(0,0,1));
    //new_cam.ResetToBounds(spatialBounds);
    //vtkm::Vec<vtkm::Float32, 3> pos = new_cam.GetPosition();
    //pos[1] *= 1;
    //pos[2] += pos[2] * 4.0;
    //new_cam.SetPosition(pos);

    vtkm::Vec<vtkm::Float32,3> wtf(2,2,2);
	view = new vtkm::rendering::View3D(scene, mapper, canvas, bg);

    view->Initialize();
    //glutMainLoop();
	std::cout << "paint" << std::endl;
    view->Paint();
    view->SaveAs("reg3D.pnm");
  }
