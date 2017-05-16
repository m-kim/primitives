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
  VTKM_EXEC_CONT
  Tree()
  {
  }
  //Tree(const Tree &rhs):cnt(rhs.cnt), vtx(rhs.vtx), idx(rhs.idx) {

  //}

  VTKM_EXEC_CONT
  vtkm::UInt32 getCnt(vtkm::Id id) const {return cnt.Get(id);}
  VTKM_EXEC_CONT
  vtkm::UInt32 getIdx(vtkm::Id id) const {return idx.Get(id);}
  VTKM_EXEC_CONT
  vtkm::UInt32 getVtx(vtkm::Id id) const {return vtx.Get(id);}

  VTKM_CONT
  void SetCnt(vtkm::cont::ArrayHandle<vtkm::UInt32 > cntArray){
    cnt = cntArray.PrepareForInPlace(DeviceAdapterTag());

  }
  VTKM_CONT
  void SetVtx(vtkm::cont::ArrayHandle<vtkm::UInt32 > vtxArray){
      vtx = vtxArray.PrepareForInPlace(DeviceAdapterTag());

  }
  VTKM_CONT
  void SetIdx(vtkm::cont::ArrayHandle<vtkm::UInt32 > idxArray){
      idx = idxArray.PrepareForInPlace(DeviceAdapterTag());

  }

protected:

  PortalType cnt, vtx, idx;
};



#endif // TREE_H

