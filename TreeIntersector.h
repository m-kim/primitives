#ifndef TREEINTERSECTOR_H
#define TREEINTERSECTOR_H

#include "Tree.h"

template <typename DeviceAdapterTag>
class TreeIntersector : public vtkm::exec::ExecutionObjectBase
{
public:
  enum CSG_TYPE{
    CYLINDER = vtkm::CELL_SHAPE_TRIANGLE,
    BOX = vtkm::CELL_SHAPE_LINE,
    BOUND = vtkm::CELL_SHAPE_TETRA,
    SPHERE = vtkm::CELL_SHAPE_VERTEX
  };

  typedef typename vtkm::cont::ArrayHandle<vtkm::UInt8>
	  ::template ExecutionTypes<DeviceAdapterTag>::Portal PortalShapeType;

  typedef typename vtkm::cont::ArrayHandle<vtkm::Id>
	  ::template ExecutionTypes<DeviceAdapterTag>::Portal PortalIndexType;


  typedef typename Tree<DeviceAdapterTag>::vec3 vec3;

  VTKM_EXEC_CONT
  TreeIntersector()
  {
  }

  VTKM_EXEC_CONT
	  TreeIntersector(const TreeIntersector &rhs) :ShapesPortal(rhs.ShapesPortal), OffsetsPortal(rhs.OffsetsPortal)
  {

  }

  VTKM_CONT
  void SetOffset(vtkm::cont::ArrayHandle<vtkm::Id> &offsetsPortal)
  {
	  OffsetsPortal = offsetsPortal.PrepareForInPlace(DeviceAdapterTag());
	  
  }

  VTKM_CONT
  void SetShapes(vtkm::cont::ArrayHandle<vtkm::UInt8> &shapesPortal)
  {
	  ShapesPortal = shapesPortal.PrepareForInPlace(DeviceAdapterTag());
  }
  VTKM_EXEC_CONT
  vec3 cylinder(const vec3 &ray_start,
      const vec3 &ray_direction,
      const vec3 &p,
      const vec3 &q,
      float r) const
  {
    float t = 0;
    vec3 ray_end = ray_start + ray_direction * 1000;
    vec3 d = q - p;
    vec3 m = ray_start - p;
    vec3 n = ray_end - ray_start;
    float md = dot(m, d);
    float nd = dot(n, d);
    float dd = dot(d, d);
    if ((md < 0.0f) && (md + nd < 0.0f)) return vec3(0, 0, 0); //segment outside 'p' side of cylinder
    if ((md > dd) && (md + nd  > dd))  return vec3(0, 0, 0); //segment outside 'q' side of cylinder
    float nn = dot(n, n);
    float mn = dot(m, n);
    float a = dd * nn - nd *nd;
    float k = dot(m, m) - r * r;
    float c = dd * k - md * md;

    if (fabs(a) < 1e-6) {
      if (c > 0.0) //'a' and thus the segment lie outside cylinder
        return vec3(0, 0, 0);
      if (md < 0.0f) //intesect segment against 'p' endcap
        t = -mn / nn;
      else if (md > dd) //intersect segment against 'q' endcap
t = (nd - mn) / nn;
      else //else 'a' lies inside cylinder
        t = 0;

        return vec3(1, 2, 0);
    }
    float b = dd * mn - nd * md;
    float discr = b * b - a * c;
    if (discr < 0.0f) //no roots
      return vec3(0, 0, 0);
    t = (-b - sqrt(discr)) / a;
    if (md + t * nd < 0.0f) {
      //intersect outside cylinder on 'p' side
      if (nd <= 0.0f) return vec3(0.0, 0.0, 0);
      t = -md / nd;
      return vec3(k + 2 * t * (mn + t *nn) <= 0.0f, t * 1000, 0);
    }
    else if (md + t * nd > dd) {
      //intersect outside cylinder on 'q' side
      if (nd >= 0.0f) return vec3(0, 0, 0);
      t = (dd - md) / nd;

      return vec3(k + dd - 2 * md + t * (2 * (mn - nd) + t * nn) <= 0.0f, t * 1000, 0);
    }

    return vec3(1, t * 1000, 0);

  }

  VTKM_EXEC
  vec3 box(const vec3 &ray_start,
           const vec3 &ray,
           const vec3 &lb,
           const vec3 &rt) const
  {
   vec3 rayfrac;
   rayfrac[0] = 1.0 / ray[0];
   rayfrac[1] = 1.0 / ray[1];
   rayfrac[2] = 1.0 / ray[2];
   float t1 = (lb[0] - ray_start[0])*rayfrac[0];
   float t2 = (rt[0] - ray_start[0])*rayfrac[0];
   float t3 = (lb[1] - ray_start[1])*rayfrac[1];
   float t4 = (rt[1] - ray_start[1])*rayfrac[1];
   float t5 = (lb[2] - ray_start[2])*rayfrac[2];
   float t6 = (rt[2] - ray_start[2])*rayfrac[2];

   int face = 0;

   //float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
   float tmin = 0;
   if (t1 < t2) {
     tmin = t1;
     face = 0;
   }
   else {
     tmin = t2;
     face = 1;
   }
   if (t3 < t4) {
     if (tmin < t3) {
       tmin = t3;
       face = 2;
     }
   }
   else {
     if (tmin < t4) {
       tmin = t4;
       face = 3;
     }

   }
   if (t5 < t6) {
     if (tmin < t5) {
       tmin = t5;
       face = 4;
     }
   }
   else {
     if (tmin < t6) {
       tmin = t6;
       face = 5;
     }

   }
   float tmax = vtkm::Min(vtkm::Min(vtkm::Max(t1, t2), vtkm::Max(t3, t4)), vtkm::Max(t5, t6));

   // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
   if (tmax < 0)
   {
     return vec3(0, tmax, 0);
   }

   // if tmin > tmax, ray doesn't intersect AABB
   if (tmin > tmax)
   {
     return vec3(0, tmax, 0);
   }

   return vec3(1, tmin, face);
  }

  VTKM_EXEC
  vec3 sphere(const vec3 &ray_start,
      const vec3 &ray_direction,
      const vec3 &center,
      float r) const
  {
    vec3 p = ray_start - center;

    float b = dot(p, ray_direction);
    float c = dot(p, p) - r * r;

    if (c > 0 && b > 0) {
      return vec3(0, 0, 0);
    }

    float discr = b * b - c;

    if (discr < 0.0) {
      return vec3(0, 0, 0);
    }

    float t = -b - sqrt(discr);

    if (t < 0)
      return vec3(0, 0, 0);

    return vec3(1.0, t, 0);
  }

  template<typename PointPortalType, typename ScalarPortalType>
  VTKM_EXEC
	  void query(
            const PointPortalType &points,
            const ScalarPortalType &scalars,
            const vtkm::Vec<vtkm::Float32, 3> &rayOrigin,
      const vtkm::Vec<vtkm::Float32, 3> &rayDir,
            vtkm::UInt8 &fin_type,
            vtkm::UInt8 &face,
            vtkm::Id &fin_offset,
            vec3 &fin_center,
            vtkm::Float32 &minDistance,
            vtkm::Id &hitIndex,
            float &matid,
          const Tree<DeviceAdapterTag> &treePtr) const
  {

    vec3 cyl_top, cyl_bottom;
    vec3 center;
    vtkm::Float32 cyl_radius;
    vec3 box_ll, box_ur;

    int stack[64], s_cnt[64], s_i[64], stack_ptr;
    int _idx = 0;//

    vtkm::Vec<vtkm::Float32, 3> lower, upper;
    int _vtx = treePtr.getVtx(_idx);//child_vtx[_idx];
    vtkm::UInt8 type = ShapesPortal.Get(_vtx);
    vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);
    lower = points.Get(cur_offset);
    upper = points.Get(cur_offset + 1);
    vec3 ret = box(rayOrigin, rayDir, lower, upper);
    if (ret[0] > 0){

  //   recurse(points,
  //           scalars,
  //           rayOrigin,
  //           rayDir,
  //           fin_type,
  //           face,
  //           fin_offset,
  //           fin_center,
  //           minDistance,
  //           hitIndex,
  //           treePtr.child_cnt[_idx],
  //           _idx);

         vtkm::Vec<vtkm::Float32,3> lower, upper;
         //vtkm::Id cur_offset = coords.GetNumberOfValues();
          vec3 ret;
          int _cnt = 0;
         stack_ptr = 0;
         stack[stack_ptr] = _idx;
         _cnt = s_cnt[stack_ptr] = treePtr.getCnt(_idx);//child_cnt[_idx];
         s_i[stack_ptr] = 0;
         _idx = treePtr.getIdx(_idx);//child_idx[_idx];

         stack_ptr++;
         int i = 0;
         do{
           for(;i < _cnt;){
             _vtx = treePtr.getVtx(_idx+i);//child_vtx[_idx + i];
             //lower = wtf.Get(_vtx);
             //upper = wtf.Get( _vtx + 1);
             vtkm::UInt8 type = ShapesPortal.Get(_vtx);
             vtkm::Id cur_offset = OffsetsPortal.Get(_vtx);
             if(type == BOUND){
               lower = points.Get(cur_offset);
               upper = points.Get(cur_offset+1);
               ret = box(rayOrigin, rayDir, lower, upper);
               if (ret[0] > 0){
                 //push onto stack
                 stack[stack_ptr] = _idx;
                 s_cnt[stack_ptr] = _cnt;
                 s_i[stack_ptr] = i;


                 _cnt = treePtr.getCnt(_idx+i);//child_cnt[_idx + i];
                 _idx = treePtr.getIdx(_idx+i);//child_idx[_idx + i];
                 i = 0;
                 stack_ptr++;
               }
               else{
                 i++;
               }
             }
             else{

               //reached a leaf
               switch(type){
               case CYLINDER:
                 cyl_bottom = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));
                 cyl_top = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset + 1));
                 cyl_radius = vtkm::Float32(scalars.Get(cur_offset));
                 //ret = vtkm::Vec<vtkm::Float32, 3>(1,1,1);
                 ret = cylinder(rayOrigin, rayDir, cyl_bottom, cyl_top, cyl_radius);
                 if (ret[0] > 0) {
                   if (ret[1] < minDistance) {
                     matid = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset + 2))[0];
                     minDistance = ret[1];
                     hitIndex = 35;
                     fin_type = type;
                     fin_offset = cur_offset;
                   }
                 }
                 break;

               case BOX:
                 box_ll = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));
                 box_ur = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset+1));
                 ret = box(rayOrigin, rayDir, box_ll, box_ur);
                 if (ret[0] > 0) {
                   if (ret[1] < minDistance) {
                     matid = 0.125;
                     minDistance = ret[1];
                     hitIndex = 35;
                     fin_type = type;
                     fin_offset = cur_offset;
                     face = ret[2];
                   }
                 }
                 break;

               case SPHERE:
                 center = vtkm::Vec<vtkm::Float32, 3>(points.Get(cur_offset));//1.5;
                 ret = sphere(rayOrigin, rayDir, center, vtkm::Float32(scalars.Get(cur_offset)));
                 if (ret[0] > 0) {
                   if (ret[1] < minDistance) {
                     minDistance = ret[1];
                     hitIndex = 35;
                     fin_type = type;
                     fin_offset = cur_offset;
                     fin_center = center;
                   }
                 }
                 break;
               }
               i++;
             }
           }
           if (i >= _cnt ){
             stack_ptr--;
             _idx = stack[stack_ptr];
             _cnt = s_cnt[stack_ptr];
             i = s_i[stack_ptr];
             i++;
           }
         }
         while(stack_ptr > 0);

      }

  }

  PortalShapeType ShapesPortal;
  PortalIndexType OffsetsPortal;

};


#endif
