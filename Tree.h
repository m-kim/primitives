#ifndef TREE_H
#define TREE_H


#include <vector>
#include <vtkm/Math.h>
#include <vtkm/cont/DataSet.h>
#include <vtkm/cont/DataSetBuilderExplicit.h>

template<typename DeviceAdapterTag>
class Tree : public vtkm::exec::ExecutionObjectBase
{
private:
  typedef typename vtkm::cont::ArrayHandle<vtkm::UInt32>
      ::template ExecutionTypes<DeviceAdapterTag>::Portal PortalType;

public:
  typedef vtkm::Vec<vtkm::Float32, 3> vec3;

  Tree()
  {
  }

  void vtxPush();
  void idxvtxPush();
  void AddCell(const vtkm::UInt8 shape);
  void AddPoint(vtkm::Float32, vtkm::Float32, vtkm::Float32);
  void AddCellPoint(vtkm::Id idx);
  void AddCellPoint();
  void Create();
  vtkm::UInt32 getCnt(vtkm::Id id) const {return cnt.Get(id);}
  vtkm::UInt32 getIdx(vtkm::Id id) const {return idx.Get(id);}
  vtkm::UInt32 getVtx(vtkm::Id id) const {return vtx.Get(id);}

  void SetCnt(vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray){
    cnt = cntArray.PrepareForInPlace(DeviceAdapterTag());

  }
  void SetVtx(vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray){
      vtx = vtxArray.PrepareForInPlace(DeviceAdapterTag());

  }
  void SetIdx(vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray){
      idx = idxArray.PrepareForInPlace(DeviceAdapterTag());

  }

protected:

  PortalType cnt, vtx, idx;
};



#endif // TREE_H

