#include "cloud.hpp"
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/features/moment_of_inertia_estimation.h>

struct KdTree {
  Cloud::Ptr data;
  pcl::search::KdTree<XYZ> tree;
  explicit KdTree(const PointCloud &c) : data(c.data) { if (!data->empty()) tree.setInputCloud(data); }
  py::tuple nearest(const Eigen::Vector3f &query, int k) {
    if (!query.allFinite() || k < 1) throw std::invalid_argument("Query must be finite and k positive");
    std::vector<int> indices; std::vector<float> distances;
    if (!data->empty()) tree.nearestKSearch(XYZ(query.x(),query.y(),query.z()), std::min<std::size_t>(k,data->size()),indices,distances);
    return py::make_tuple(indices, distances);
  }
  py::tuple radius(const Eigen::Vector3f &query, float radius, unsigned maximum) {
    positive(radius,"radius"); if (!query.allFinite()) throw std::invalid_argument("Query must be finite");
    std::vector<int> indices; std::vector<float> distances;
    if (!data->empty()) tree.radiusSearch(XYZ(query.x(),query.y(),query.z()),radius,indices,distances,maximum);
    return py::make_tuple(indices, distances);
  }
};

void bind_geometry(py::module_ &m, CloudClass &cls) {
  py::class_<KdTree>(m,"KdTree")
    .def(py::init<const PointCloud &>())
    .def("nearest_k", &KdTree::nearest, py::arg("query"), py::arg("k")=1,
         "Return (indices, squared_distances), sorted nearest first.")
    .def("radius_search", &KdTree::radius, py::arg("query"), py::arg("radius"), py::arg("max_neighbors")=0,
         "Return (indices, squared_distances); max_neighbors=0 means unlimited.");
  cls.def("centroid", [](const PointCloud &c) -> Eigen::Vector3f {
    require_points(c); Eigen::Vector4f center; pcl::compute3DCentroid(*c.data,center); return center.head<3>();
  }).def("covariance", [](const PointCloud &c) {
    require_points(c); Eigen::Vector4f center; Eigen::Matrix3f covariance;
    pcl::compute3DCentroid(*c.data,center); pcl::computeCovarianceMatrixNormalized(*c.data,center,covariance); return covariance;
  }).def("bounds", [](const PointCloud &c) {
    require_points(c); XYZ low,high; pcl::getMinMax3D(*c.data,low,high);
    return py::make_tuple(Eigen::Vector3f(low.x,low.y,low.z), Eigen::Vector3f(high.x,high.y,high.z));
  }).def("pca", [](const PointCloud &c) {
    require_points(c,3); pcl::PCA<XYZ> pca; pca.setInputCloud(c.data);
    py::dict result; result["centroid"]=Eigen::Vector3f(pca.getMean().head<3>());
    result["eigenvalues"]=Eigen::Vector3f(pca.getEigenValues()); result["eigenvectors"]=Eigen::Matrix3f(pca.getEigenVectors()); return result;
  }).def("oriented_bounding_box", [](const PointCloud &c) {
    require_points(c,3); pcl::MomentOfInertiaEstimation<XYZ> estimator; estimator.setInputCloud(c.data); estimator.compute();
    XYZ low,high,position; Eigen::Matrix3f rotation;
    if (!estimator.getOBB(low,high,position,rotation)) throw std::runtime_error("PCL could not compute bounding box");
    py::dict result; result["minimum"]=Eigen::Vector3f(low.x,low.y,low.z); result["maximum"]=Eigen::Vector3f(high.x,high.y,high.z);
    result["position"]=Eigen::Vector3f(position.x,position.y,position.z); result["rotation"]=rotation; return result;
  }).def("concatenate", [](const PointCloud &c, const PointCloud &other) {
    PointCloud out; *out.data=*c.data+*other.data; return out;
  }, py::arg("other"))
  .def("segment_model", [](const PointCloud &c, const std::string &model, float distance, int iterations, double probability, float min_radius, float max_radius) {
    positive(distance,"distance_threshold");
    if (iterations<1 || !std::isfinite(probability) || probability<=0 || probability>=1 ||
        !std::isfinite(min_radius) || !std::isfinite(max_radius) || min_radius<0 || max_radius<min_radius)
      throw std::invalid_argument("Invalid iteration, probability, or radius limits");
    const std::map<std::string,std::pair<int,int>> models = {{"plane",{pcl::SACMODEL_PLANE,3}}, {"line",{pcl::SACMODEL_LINE,2}},
      {"sphere",{pcl::SACMODEL_SPHERE,4}}, {"circle2d",{pcl::SACMODEL_CIRCLE2D,3}}, {"circle3d",{pcl::SACMODEL_CIRCLE3D,3}}};
    auto chosen=models.find(model); if (chosen==models.end()) throw std::invalid_argument("model must be plane, line, sphere, circle2d, or circle3d");
    pcl::PointIndices indices; pcl::ModelCoefficients coeff;
    if (c.data->size() >= static_cast<std::size_t>(chosen->second.second)) {
      pcl::SACSegmentation<XYZ> seg; seg.setInputCloud(c.data); seg.setModelType(chosen->second.first); seg.setMethodType(pcl::SAC_RANSAC);
      seg.setOptimizeCoefficients(true); seg.setDistanceThreshold(distance); seg.setMaxIterations(iterations);
      seg.setProbability(probability); seg.setRadiusLimits(min_radius,max_radius); seg.segment(indices,coeff);
    }
    return py::make_tuple(indices.indices,coeff.values);
  }, py::arg("model"), py::arg("distance_threshold"), py::arg("max_iterations")=1000, py::arg("probability")=0.99,
     py::arg("min_radius")=0.0f, py::arg("max_radius")=std::numeric_limits<float>::max());
}
