#include "cloud.hpp"
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/segmentation/region_growing.h>

void bind_features(CloudClass &cls) {
  cls.def("normals_with_curvature", [](const PointCloud &c, float radius, const Eigen::Vector3f &viewpoint) {
    positive(radius,"radius"); if (!viewpoint.allFinite()) throw std::invalid_argument("viewpoint must be finite");
    pcl::PointCloud<pcl::Normal> normals;
    if (!c.data->empty()) {
      pcl::NormalEstimation<XYZ,pcl::Normal> estimator; estimator.setInputCloud(c.data);
      estimator.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>()); estimator.setRadiusSearch(radius);
      estimator.setViewPoint(viewpoint.x(),viewpoint.y(),viewpoint.z()); estimator.compute(normals);
    }
    Array result({static_cast<py::ssize_t>(normals.size()),py::ssize_t(4)}); auto a=result.mutable_unchecked<2>();
    for (std::size_t i=0;i<normals.size();++i) {
      a(i,0)=normals[i].normal_x; a(i,1)=normals[i].normal_y; a(i,2)=normals[i].normal_z; a(i,3)=normals[i].curvature;
    }
    return result;
  },py::arg("radius"),py::arg("viewpoint")=Eigen::Vector3f::Zero().eval())
  .def("fpfh", [](const PointCloud &c, const Array &array, float radius) {
    positive(radius,"radius"); auto normals=normals_from_array(array,c.data->size());
    pcl::PointCloud<pcl::FPFHSignature33> descriptors;
    if (!c.data->empty()) {
      pcl::FPFHEstimation<XYZ,pcl::Normal,pcl::FPFHSignature33> f; f.setInputCloud(c.data); f.setInputNormals(normals);
      f.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>()); f.setRadiusSearch(radius); f.compute(descriptors);
    }
    Array result({static_cast<py::ssize_t>(descriptors.size()),py::ssize_t(33)}); auto a=result.mutable_unchecked<2>();
    for (std::size_t i=0;i<descriptors.size();++i) for (int j=0;j<33;++j) a(i,j)=descriptors[i].histogram[j];
    return result;
  },py::arg("normals"),py::arg("radius"))
  .def("vfh", [](const PointCloud &c, const Array &array, const Eigen::Vector3f &viewpoint) {
    require_points(c,2); if (!viewpoint.allFinite()) throw std::invalid_argument("viewpoint must be finite");
    auto normals=normals_from_array(array,c.data->size()); pcl::PointCloud<pcl::VFHSignature308> descriptors;
    pcl::VFHEstimation<XYZ,pcl::Normal,pcl::VFHSignature308> f; f.setInputCloud(c.data); f.setInputNormals(normals);
    f.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>()); f.setViewPoint(viewpoint.x(),viewpoint.y(),viewpoint.z()); f.compute(descriptors);
    if (descriptors.empty()) throw std::runtime_error("PCL VFH estimation failed");
    Array result(308); auto a=result.mutable_unchecked<1>(); for (int j=0;j<308;++j) a(j)=descriptors[0].histogram[j]; return result;
  },py::arg("normals"),py::arg("viewpoint")=Eigen::Vector3f::Zero().eval())
  .def("principal_curvatures", [](const PointCloud &c, const Array &array, float radius) {
    positive(radius,"radius"); auto normals=normals_from_array(array,c.data->size());
    pcl::PointCloud<pcl::PrincipalCurvatures> descriptors;
    if (!c.data->empty()) {
      pcl::PrincipalCurvaturesEstimation<XYZ,pcl::Normal,pcl::PrincipalCurvatures> f;
      f.setInputCloud(c.data); f.setInputNormals(normals); f.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>());
      f.setRadiusSearch(radius); f.compute(descriptors);
    }
    Array result({static_cast<py::ssize_t>(descriptors.size()),py::ssize_t(5)}); auto a=result.mutable_unchecked<2>();
    for (std::size_t i=0;i<descriptors.size();++i) {
      a(i,0)=descriptors[i].principal_curvature_x; a(i,1)=descriptors[i].principal_curvature_y;
      a(i,2)=descriptors[i].principal_curvature_z; a(i,3)=descriptors[i].pc1; a(i,4)=descriptors[i].pc2;
    }
    return result;
  },py::arg("normals"),py::arg("radius"))
  .def("region_growing", [](const PointCloud &c, const Array &array, int neighbors, float angle, float curvature, int minimum, int maximum) {
    if (neighbors<1 || minimum<1 || maximum<minimum || !std::isfinite(angle) || angle<=0 || angle>3.141592654f ||
        !std::isfinite(curvature) || curvature<0) throw std::invalid_argument("Invalid region-growing parameters (angle is in radians)");
    if (array.ndim()!=2 || array.shape(1)!=4) throw std::invalid_argument("Region growing needs (N,4) normals including curvature");
    auto normals=normals_from_array(array,c.data->size()); std::vector<std::vector<int>> result;
    if (c.data->empty()) return result;
    pcl::RegionGrowing<XYZ,pcl::Normal> seg; seg.setInputCloud(c.data); seg.setInputNormals(normals);
    seg.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>()); seg.setNumberOfNeighbours(neighbors);
    seg.setSmoothnessThreshold(angle); seg.setCurvatureThreshold(curvature); seg.setMinClusterSize(minimum); seg.setMaxClusterSize(maximum);
    std::vector<pcl::PointIndices> clusters; seg.extract(clusters); for (const auto &v:clusters) result.push_back(v.indices); return result;
  },py::arg("normals"),py::arg("neighbors")=30,py::arg("smoothness_threshold")=0.05235988f,
     py::arg("curvature_threshold")=1.0f,py::arg("min_size")=30,py::arg("max_size")=25000)
  .def("segment_cylinder", [](const PointCloud &c, const Array &array, float distance, float minimum, float maximum, int iterations, float weight) {
    positive(distance,"distance_threshold");
    if (!std::isfinite(minimum)||!std::isfinite(maximum)||minimum<0||maximum<=minimum||iterations<1||!std::isfinite(weight)||weight<0||weight>1)
      throw std::invalid_argument("Invalid cylinder radius limits, iterations, or normal_distance_weight");
    auto normals=normals_from_array(array,c.data->size()); pcl::PointIndices indices; pcl::ModelCoefficients coefficients;
    if (c.data->size()>=3) {
      pcl::SACSegmentationFromNormals<XYZ,pcl::Normal> seg; seg.setInputCloud(c.data); seg.setInputNormals(normals);
      seg.setModelType(pcl::SACMODEL_CYLINDER); seg.setMethodType(pcl::SAC_RANSAC); seg.setOptimizeCoefficients(true);
      seg.setDistanceThreshold(distance); seg.setRadiusLimits(minimum,maximum); seg.setMaxIterations(iterations);
      seg.setNormalDistanceWeight(weight); seg.segment(indices,coefficients);
    }
    return py::make_tuple(indices.indices,coefficients.values);
  },py::arg("normals"),py::arg("distance_threshold"),py::arg("min_radius"),py::arg("max_radius"),
     py::arg("max_iterations")=1000,py::arg("normal_distance_weight")=0.1f);
}
