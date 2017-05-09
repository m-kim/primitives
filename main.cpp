#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/testing/MakeTestDataSet.h>
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
std::vector<vtkm::Float32> radii;
vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray, vtxArray,idxArray;
vtkm::rendering::View3D *view;

void load(const char *fn)
{
    vtkm::io::reader::VTKDataSetReader reader(fn);
    reader.ReadDataSet();
    csg = reader.GetDataSet();

    std::ifstream cntf;
    cntf.open("count.dat");
    size_t size = 0;
    cntf.read((char*)&size, sizeof(size_t));
    child_cnt.resize(size);
    cntf.read((char*)&child_cnt[0], sizeof(size_t) * size);

    std::ifstream idxf;
    idxf.open("index.dat");
    size = 0;
    idxf.read((char*)&size, sizeof(size_t));
    child_idx.resize(size);
    idxf.read((char*)&child_idx[0], sizeof(size_t) * size);

    std::ifstream vtxf;
    vtxf.open("vertex.dat");
    size = 0;
    vtxf.read((char*)&size, sizeof(size_t));
    child_vtx.resize(size);
    vtxf.read((char*)&child_vtx[0], sizeof(size_t) * size);

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
    load("dataset.vtk");
    dataSetFieldAdd.AddPointField(csg, "radius", radii);


  //    glutInit(&argc, argv);
  //    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  //    glutInitWindowSize(W,H);
  //    glutCreateWindow("VTK-m Rendering");
  //    glutDisplayFunc(displayCall);
  //    glutMotionFunc(mouseMove);
  //    glutMouseFunc(mouseCall);
  //    glutReshapeFunc(reshape);

    vtkm::rendering::Color bg(0.2f, 0.2f, 0.2f, 1.0f);
    vtkm::rendering::CanvasRayTracer canvas;
    MapperRayTracer mapper(csg.GetCellSet(), cntArray, idxArray, vtxArray );

    vtkm::rendering::Scene scene;
    scene.AddActor(vtkm::rendering::Actor(csg.GetCellSet(),
                                          csg.GetCoordinateSystem(),
                                          csg.GetField("radius"),
                                          vtkm::rendering::ColorTable("thermal")));

    //Create vtkm rendering stuff.
    vtkm::rendering::Camera new_cam;
    vtkm::Bounds spatialBounds = scene.GetSpatialBounds();
    new_cam.SetPosition(vtkm::Vec<vtkm::Float32, 3>(0,1,0));
    new_cam.SetViewUp(vtkm::Vec<vtkm::Float32,3>(0,0,1));
    new_cam.ResetToBounds(spatialBounds);
    vtkm::Vec<vtkm::Float32, 3> pos = new_cam.GetPosition();
    pos[1] *= 1;
    pos[2] += pos[2] * 4.0;
    new_cam.SetPosition(pos);

#if 0
    treePtr->test(0);
#else
    vtkm::Vec<vtkm::Float32,3> wtf(2,2,2);
    //test_flat(wtf);
    view = new vtkm::rendering::View3D(scene, mapper, canvas, new_cam, bg);
    new_cam.SetPosition(pos);
    view->SetCamera(new_cam);

    view->Initialize();
    //glutMainLoop();
    view->Paint();
    view->SaveAs("reg3D.pnm");
#endif
  //QImage img(canvas.GetWidth(), canvas.GetHeight(), QImage::Format_RGB32);
  //typedef vtkm::cont::ArrayHandle<vtkm::Vec<vtkm::Float32, 4> > ColorBufferType;

  //ColorBufferType::PortalConstControl colorPortal =
  //  canvas.GetColorBuffer().GetPortalConstControl();
  //for (vtkm::Id yIndex = canvas.GetHeight() - 1; yIndex >= 0; yIndex--)
  //{
  //  for (vtkm::Id xIndex = 0; xIndex < canvas.GetWidth(); xIndex++)
  //  {
  //    vtkm::Vec<vtkm::Float32, 4> tuple =
  //      colorPortal.Get(yIndex*canvas.GetWidth() + xIndex);
  //    QRgb value;
  //    value = qRgb(tuple[0]*255, tuple[1]*255, tuple[2]*255);
  //    img.setPixel(xIndex, (canvas.GetHeight() -1) - yIndex, value);
  //  }
  //}

  //QImageWriter writer;
  //writer.setFormat("png");
  //writer.setFileName("reg3D.png");
  //writer.write(img);
  }
