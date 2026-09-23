#include "cloud.hpp"

PYBIND11_MODULE(pcl_python_merlab, module) {
  module.doc() = "Course-local Python bindings to system PCL; XYZ-only subset.";
  module.attr("pcl_version") = PCL_VERSION_PRETTY;
  module.attr("__version__") = "0.2.0";
  CloudClass cloud(module, "PointCloud");
  cloud
    .def(py::init<>())
    .def(py::init<const Array &>(), py::arg("xyz"))
    .def("__len__", [](const PointCloud &cloud) { return cloud.data->size(); })
    .def_property_readonly("xyz", &PointCloud::xyz, "Return an independent (N, 3) float32 array.")
    .def("transform", &PointCloud::transform, py::arg("matrix"), "PCL transformPointCloud; returns a new cloud.")
    .def("voxel_grid", &PointCloud::voxel_grid, py::arg("leaf_size"), "PCL VoxelGrid centroids; meters.")
    .def("crop_box", &PointCloud::crop_box, py::arg("minimum"), py::arg("maximum"), "PCL CropBox bounds in cloud coordinates.")
    .def("segment_plane", &PointCloud::segment_plane,
         py::arg("distance_threshold"), py::arg("max_iterations") = 100,
         "PCL SACSegmentation/RANSAC; returns (inlier_indices, [a,b,c,d]). Empty lists on failure.")
    .def("extract", &PointCloud::extract, py::arg("indices"), py::arg("negative") = false,
         "PCL ExtractIndices; negative=True removes the selected points.")
    .def("euclidean_clusters", &PointCloud::euclidean_clusters,
         py::arg("tolerance"), py::arg("min_size") = 30, py::arg("max_size") = 25000,
         "PCL EuclideanClusterExtraction; returns lists of indices into this cloud.")
    .def("estimate_normals", &PointCloud::estimate_normals,
         py::arg("radius"), py::arg("viewpoint") = Eigen::Vector3f::Zero().eval(),
         "PCL NormalEstimation; returns (N,3) normals, NaN where neighbors are insufficient.");
  bind_filters(cloud);
  bind_geometry(module, cloud);
  bind_features(cloud);
  bind_registration(cloud);
  bind_surface_io(module, cloud);
}
