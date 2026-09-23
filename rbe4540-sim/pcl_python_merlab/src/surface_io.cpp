#include "cloud.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/filter.h>

namespace {
template<class Hull>
py::tuple hull(Hull &algorithm,const PointCloud &c,int dimension) {
  if (dimension!=2 && dimension!=3) throw std::invalid_argument("dimension must be 2 or 3");
  require_points(c,dimension+1); algorithm.setInputCloud(c.data); algorithm.setDimension(dimension);
  PointCloud vertices; std::vector<pcl::Vertices> polygons; algorithm.reconstruct(*vertices.data,polygons);
  std::vector<std::vector<unsigned>> faces;
  for (const auto &polygon:polygons) faces.emplace_back(polygon.vertices.begin(),polygon.vertices.end());
  return py::make_tuple(vertices,faces);
}
PointCloud load(const std::string &path,bool ply) {
  // Read the generic message first so a file without XYZ is not silently
  // converted into an all-zero XYZ cloud by PCL's field mapping.
  pcl::PCLPointCloud2 raw;
  int code=ply ? pcl::io::loadPLYFile(path,raw) : pcl::io::loadPCDFile(path,raw);
  if (code<0) throw std::runtime_error("PCL could not read file: "+path);
  for (const std::string name:{"x","y","z"}) {
    auto field=std::find_if(raw.fields.begin(),raw.fields.end(),[&](const auto &f){ return f.name==name; });
    if (field==raw.fields.end() || field->datatype!=pcl::PCLPointField::FLOAT32 || field->count!=1)
      throw std::invalid_argument("File must contain scalar float32 XYZ fields");
  }
  PointCloud result; pcl::fromPCLPointCloud2(raw,*result.data);
  result.data->is_dense=false; std::vector<int> indices;
  Cloud finite; pcl::removeNaNFromPointCloud(*result.data,finite,indices); *result.data=std::move(finite);
  return result;
}
}
void bind_surface_io(py::module_ &m,CloudClass &cls) {
  m.def("load_pcd",[](const std::string &p){return load(p,false);},py::arg("path"));
  m.def("load_ply",[](const std::string &p){return load(p,true);},py::arg("path"));
  cls.def("save_pcd",[](const PointCloud &c,const std::string &path,bool binary) {
    require_points(c);
    if (pcl::io::savePCDFile(path,*c.data,binary)<0) throw std::runtime_error("PCL could not write file: "+path);
  },py::arg("path"),py::arg("binary")=true)
  .def("save_ply",[](const PointCloud &c,const std::string &path,bool binary) {
    require_points(c);
    if (pcl::io::savePLYFile(path,*c.data,binary)<0) throw std::runtime_error("PCL could not write file: "+path);
  },py::arg("path"),py::arg("binary")=true)
  .def("moving_least_squares",[](const PointCloud &c,float radius,int order) {
    positive(radius,"radius"); if (order<1 || order>5) throw std::invalid_argument("polynomial_order must be between 1 and 5");
    PointCloud out; if (c.data->empty()) return out;
    pcl::MovingLeastSquares<XYZ,XYZ> mls; mls.setInputCloud(c.data); mls.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>());
    mls.setSearchRadius(radius); mls.setPolynomialOrder(order); mls.setComputeNormals(false); mls.process(*out.data);
    return out;
  },py::arg("radius"),py::arg("polynomial_order")=2)
  .def("convex_hull",[](const PointCloud &c,int dimension) {
    pcl::ConvexHull<XYZ> algorithm; return hull(algorithm,c,dimension);
  },py::arg("dimension")=3)
  .def("concave_hull",[](const PointCloud &c,float alpha,int dimension) {
    positive(alpha,"alpha"); pcl::ConcaveHull<XYZ> algorithm; algorithm.setAlpha(alpha); return hull(algorithm,c,dimension);
  },py::arg("alpha"),py::arg("dimension")=3);
}
