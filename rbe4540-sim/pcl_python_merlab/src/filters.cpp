#include "cloud.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/project_inliers.h>

void bind_filters(CloudClass &cls) {
  cls.def("pass_through", [](const PointCloud &c, const std::string &axis, float low, float high, bool negative) {
    if ((axis != "x" && axis != "y" && axis != "z") || !std::isfinite(low) || !std::isfinite(high) || low > high)
      throw std::invalid_argument("Use axis x/y/z and finite ordered limits");
    PointCloud out; if (c.data->empty()) return out;
    pcl::PassThrough<XYZ> f; f.setInputCloud(c.data); f.setFilterFieldName(axis);
    f.setFilterLimits(low, high); f.setNegative(negative); f.filter(*out.data); return out;
  }, py::arg("axis"), py::arg("minimum"), py::arg("maximum"), py::arg("negative")=false)
  .def("statistical_outlier_removal", [](const PointCloud &c, int k, float multiplier, bool negative) {
    positive(multiplier, "stddev_multiplier");
    if (k < 1) throw std::invalid_argument("mean_k must be positive");
    if (c.data->empty()) return PointCloud();
    if (c.data->size() <= static_cast<std::size_t>(k)) throw std::invalid_argument("mean_k must be smaller than cloud size");
    PointCloud out; pcl::StatisticalOutlierRemoval<XYZ> f;
    f.setInputCloud(c.data); f.setMeanK(k); f.setStddevMulThresh(multiplier);
    f.setNegative(negative); f.filter(*out.data); return out;
  }, py::arg("mean_k")=20, py::arg("stddev_multiplier")=1.0f, py::arg("negative")=false)
  .def("radius_outlier_removal", [](const PointCloud &c, float radius, int neighbors, bool negative) {
    positive(radius, "radius"); if (neighbors < 1) throw std::invalid_argument("min_neighbors must be positive");
    PointCloud out; if (c.data->empty()) return out;
    pcl::RadiusOutlierRemoval<XYZ> f; f.setInputCloud(c.data); f.setRadiusSearch(radius);
    f.setMinNeighborsInRadius(neighbors); f.setNegative(negative); f.filter(*out.data); return out;
  }, py::arg("radius"), py::arg("min_neighbors")=3, py::arg("negative")=false)
  .def("random_sample", [](const PointCloud &c, int count, unsigned seed) {
    if (count < 0 || static_cast<std::size_t>(count) > c.data->size()) throw std::invalid_argument("count must be between zero and cloud size");
    PointCloud out; if (!count) return out;
    pcl::RandomSample<XYZ> f; f.setInputCloud(c.data); f.setSample(count); f.setSeed(seed); f.filter(*out.data); return out;
  }, py::arg("count"), py::arg("seed")=0)
  .def("uniform_sampling", [](const PointCloud &c, float radius) {
    positive(radius, "radius"); PointCloud out; if (c.data->empty()) return out;
    pcl::UniformSampling<XYZ> f; f.setInputCloud(c.data); f.setRadiusSearch(radius); f.filter(*out.data); return out;
  }, py::arg("radius"))
  .def("approximate_voxel_grid", [](const PointCloud &c, float leaf) {
    positive(leaf, "leaf_size"); PointCloud out; if (c.data->empty()) return out;
    pcl::ApproximateVoxelGrid<XYZ> f; f.setInputCloud(c.data); f.setLeafSize(leaf, leaf, leaf); f.filter(*out.data); return out;
  }, py::arg("leaf_size"))
  .def("project_plane", [](const PointCloud &c, const Eigen::Vector4f &plane) {
    if (!plane.allFinite() || plane.head<3>().norm() < 1e-8f) throw std::invalid_argument("Plane must have a finite nonzero normal");
    PointCloud out; if (c.data->empty()) return out;
    Eigen::Vector4f normalized = plane / plane.head<3>().norm();
    auto coeff = pcl::make_shared<pcl::ModelCoefficients>(); coeff->values.assign(normalized.data(), normalized.data()+4);
    pcl::ProjectInliers<XYZ> f; f.setInputCloud(c.data); f.setModelType(pcl::SACMODEL_PLANE);
    f.setModelCoefficients(coeff); f.filter(*out.data); return out;
  }, py::arg("coefficients"));
}
