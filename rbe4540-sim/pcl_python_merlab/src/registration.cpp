#include "cloud.hpp"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

namespace {
template<class Registration>
py::dict align(Registration &registration, const PointCloud &source, const PointCloud &target,
               const Eigen::Matrix4f &guess, int iterations, double epsilon, double distance) {
  require_points(source,3); require_points(target,3); validate_transform(guess);
  if (iterations<1 || !std::isfinite(epsilon) || epsilon<=0 || !std::isfinite(distance) || distance<=0)
    throw std::invalid_argument("Iterations, transformation_epsilon, and max_correspondence_distance must be positive and finite");
  PointCloud out;
  registration.setInputSource(source.data); registration.setInputTarget(target.data);
  registration.setMaximumIterations(iterations); registration.setTransformationEpsilon(epsilon);
  registration.setMaxCorrespondenceDistance(distance); registration.align(*out.data,guess);
  py::dict result; result["cloud"]=out; result["transformation"]=Eigen::Matrix4f(registration.getFinalTransformation());
  result["converged"]=registration.hasConverged(); result["fitness"]=registration.getFitnessScore(); return result;
}
}

void bind_registration(CloudClass &cls) {
  cls.def("icp", [](const PointCloud &c,const PointCloud &target,const Eigen::Matrix4f &guess,int iterations,double distance,double epsilon) {
    pcl::IterativeClosestPoint<XYZ,XYZ> registration; return align(registration,c,target,guess,iterations,epsilon,distance);
  },py::arg("target"),py::arg("initial_guess")=Eigen::Matrix4f::Identity().eval(),py::arg("max_iterations")=50,
     py::arg("max_correspondence_distance")=0.05,py::arg("transformation_epsilon")=1e-8)
  .def("generalized_icp", [](const PointCloud &c,const PointCloud &target,const Eigen::Matrix4f &guess,int iterations,double distance,double epsilon,int neighbors) {
    if (neighbors<3 || c.data->size()<static_cast<std::size_t>(neighbors) || target.data->size()<static_cast<std::size_t>(neighbors))
      throw std::invalid_argument("correspondence_randomness must be >=3 and <= both cloud sizes");
    pcl::GeneralizedIterativeClosestPoint<XYZ,XYZ> registration; registration.setCorrespondenceRandomness(neighbors);
    return align(registration,c,target,guess,iterations,epsilon,distance);
  },py::arg("target"),py::arg("initial_guess")=Eigen::Matrix4f::Identity().eval(),py::arg("max_iterations")=50,
     py::arg("max_correspondence_distance")=0.05,py::arg("transformation_epsilon")=1e-8,py::arg("correspondence_randomness")=20)
  .def("ndt", [](const PointCloud &c,const PointCloud &target,float resolution,float step,const Eigen::Matrix4f &guess,int iterations,double epsilon) {
    positive(resolution,"resolution"); positive(step,"step_size"); require_points(target,6);
    pcl::NormalDistributionsTransform<XYZ,XYZ> registration; registration.setResolution(resolution); registration.setStepSize(step);
    return align(registration,c,target,guess,iterations,epsilon,std::numeric_limits<double>::max());
  },py::arg("target"),py::arg("resolution"),py::arg("step_size")=0.1f,py::arg("initial_guess")=Eigen::Matrix4f::Identity().eval(),
     py::arg("max_iterations")=50,py::arg("transformation_epsilon")=1e-6);
}
