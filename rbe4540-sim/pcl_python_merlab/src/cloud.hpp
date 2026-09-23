#pragma once
// Copyright 2026 MERLab. SPDX-License-Identifier: Apache-2.0
// A focused Python API delegating algorithms to the system PCL library.
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <set>
#include <map>
#include <algorithm>

namespace py = pybind11;
using XYZ = pcl::PointXYZ;
using Cloud = pcl::PointCloud<XYZ>;
using Array = py::array_t<float, py::array::c_style | py::array::forcecast>;

inline void positive(float value, const char *name) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(std::string(name) + " must be finite and positive");
}

struct PointCloud {
  Cloud::Ptr data = pcl::make_shared<Cloud>();
  PointCloud() = default;
  explicit PointCloud(const Array &xyz) {
    if (xyz.ndim() != 2 || xyz.shape(1) != 3)
      throw std::invalid_argument("XYZ must have shape (N, 3)");
    const auto points = xyz.unchecked<2>();
    data->reserve(points.shape(0));
    for (py::ssize_t i = 0; i < points.shape(0); ++i) {
      for (int j = 0; j < 3; ++j)
        if (!std::isfinite(points(i, j)))
          throw std::invalid_argument("XYZ points must be finite");
      data->push_back(XYZ(points(i, 0), points(i, 1), points(i, 2)));
    }
    data->is_dense = true;
  }
  Array xyz() const {
    Array result({static_cast<py::ssize_t>(data->size()), py::ssize_t(3)});
    auto out = result.mutable_unchecked<2>();
    for (std::size_t i = 0; i < data->size(); ++i) {
      out(i, 0) = (*data)[i].x;
      out(i, 1) = (*data)[i].y;
      out(i, 2) = (*data)[i].z;
    }
    return result;
  }
  PointCloud transform(const Eigen::Matrix4f &matrix) const {
    if (!matrix.allFinite() ||
        !matrix.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)))
      throw std::invalid_argument("Expected a finite homogeneous 4x4 transform");
    PointCloud out;
    pcl::transformPointCloud(*data, *out.data, matrix);
    return out;
  }
  PointCloud voxel_grid(float leaf) const {
    positive(leaf, "leaf_size");
    PointCloud out;
    if (data->empty()) return out;
    pcl::VoxelGrid<XYZ> filter;
    filter.setInputCloud(data);
    filter.setLeafSize(leaf, leaf, leaf);
    filter.filter(*out.data);
    return out;
  }
  PointCloud crop_box(const Eigen::Vector3f &minimum,
                      const Eigen::Vector3f &maximum) const {
    if (!minimum.allFinite() || !maximum.allFinite() ||
        (minimum.array() > maximum.array()).any())
      throw std::invalid_argument("Bounds must be finite with minimum <= maximum");
    PointCloud out;
    if (data->empty()) return out;
    pcl::CropBox<XYZ> filter;
    filter.setInputCloud(data);
    filter.setMin(Eigen::Vector4f(minimum.x(), minimum.y(), minimum.z(), 1));
    filter.setMax(Eigen::Vector4f(maximum.x(), maximum.y(), maximum.z(), 1));
    filter.filter(*out.data);
    return out;
  }
  py::tuple segment_plane(float distance, int iterations) const {
    positive(distance, "distance_threshold");
    if (iterations <= 0) throw std::invalid_argument("max_iterations must be positive");
    pcl::PointIndices indices;
    pcl::ModelCoefficients coefficients;
    if (data->size() >= 3) {
      pcl::SACSegmentation<XYZ> segmentation;
      segmentation.setInputCloud(data);
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setDistanceThreshold(distance);
      segmentation.setMaxIterations(iterations);
      segmentation.segment(indices, coefficients);
    }
    return py::make_tuple(indices.indices, coefficients.values);
  }
  PointCloud extract(const std::vector<int> &indices, bool negative) const {
    auto selected = pcl::make_shared<pcl::PointIndices>();
    for (int i : indices) {
      if (i < 0 || static_cast<std::size_t>(i) >= data->size())
        throw std::out_of_range("Point index outside cloud");
    }
    if (std::set<int>(indices.begin(), indices.end()).size() != indices.size())
      throw std::invalid_argument("Point indices must be unique");
    selected->indices = indices;
    PointCloud out;
    if (data->empty()) return out;
    pcl::ExtractIndices<XYZ> filter;
    filter.setInputCloud(data);
    filter.setIndices(selected);
    filter.setNegative(negative);
    filter.filter(*out.data);
    return out;
  }
  std::vector<std::vector<int>> euclidean_clusters(float tolerance,
                                                  int minimum, int maximum) const {
    positive(tolerance, "tolerance");
    if (minimum < 1 || maximum < minimum)
      throw std::invalid_argument("Require 1 <= min_size <= max_size");
    std::vector<std::vector<int>> result;
    if (data->empty()) return result;
    auto tree = pcl::make_shared<pcl::search::KdTree<XYZ>>();
    pcl::EuclideanClusterExtraction<XYZ> extraction;
    extraction.setInputCloud(data);
    extraction.setSearchMethod(tree);
    extraction.setClusterTolerance(tolerance);
    extraction.setMinClusterSize(minimum);
    extraction.setMaxClusterSize(maximum);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    for (const auto &cluster : clusters) result.push_back(cluster.indices);
    return result;
  }
  Array estimate_normals(float radius, const Eigen::Vector3f &viewpoint) const {
    positive(radius, "radius");
    if (!viewpoint.allFinite()) throw std::invalid_argument("viewpoint must be finite");
    pcl::PointCloud<pcl::Normal> normals;
    if (!data->empty()) {
      pcl::NormalEstimation<XYZ, pcl::Normal> estimator;
      estimator.setInputCloud(data);
      estimator.setSearchMethod(pcl::make_shared<pcl::search::KdTree<XYZ>>());
      estimator.setRadiusSearch(radius);
      estimator.setViewPoint(viewpoint.x(), viewpoint.y(), viewpoint.z());
      estimator.compute(normals);
    }
    Array result({static_cast<py::ssize_t>(normals.size()), py::ssize_t(3)});
    auto out = result.mutable_unchecked<2>();
    for (std::size_t i = 0; i < normals.size(); ++i) {
      out(i, 0) = normals[i].normal_x;
      out(i, 1) = normals[i].normal_y;
      out(i, 2) = normals[i].normal_z;
    }
    return result;
  }
};

using CloudClass = py::class_<PointCloud>;
void bind_filters(CloudClass &);
void bind_geometry(py::module_ &, CloudClass &);
void bind_features(CloudClass &);
void bind_registration(CloudClass &);
void bind_surface_io(py::module_ &, CloudClass &);

inline void require_points(const PointCloud &cloud, std::size_t minimum = 1) {
  if (cloud.data->size() < minimum)
    throw std::invalid_argument("Not enough points for this operation");
}
inline void validate_transform(const Eigen::Matrix4f &matrix) {
  if (!matrix.allFinite() || !matrix.row(3).isApprox(Eigen::RowVector4f(0, 0, 0, 1)))
    throw std::invalid_argument("Expected a finite homogeneous 4x4 transform");
}
inline pcl::PointCloud<pcl::Normal>::Ptr normals_from_array(const Array &array, std::size_t size) {
  if (array.ndim() != 2 || array.shape(0) != static_cast<py::ssize_t>(size) ||
      (array.shape(1) != 3 && array.shape(1) != 4))
    throw std::invalid_argument("Normals must have shape (N,3) or (N,4) including curvature");
  auto normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
  const auto a = array.unchecked<2>();
  for (py::ssize_t i = 0; i < a.shape(0); ++i) {
    Eigen::Vector3f n(a(i,0), a(i,1), a(i,2));
    if (!n.allFinite() || n.norm() < 1e-8f)
      throw std::invalid_argument("Normals must be finite and nonzero; remove invalid points and normals together");
    n.normalize();
    pcl::Normal value;
    value.normal_x = n.x(); value.normal_y = n.y(); value.normal_z = n.z();
    value.curvature = a.shape(1) == 4 ? a(i,3) : 0;
    if (!std::isfinite(value.curvature) || value.curvature < 0)
      throw std::invalid_argument("Curvature must be finite and nonnegative");
    normals->push_back(value);
  }
  return normals;
}
