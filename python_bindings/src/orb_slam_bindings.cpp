#include <pybind11/pybind11.h>
#include "System.h"
#include "Map.h"
#include "ndarray_converter.h"

namespace py = pybind11;
PYBIND11_MODULE(orb_slam2, m) {
  NDArrayConverter::init_numpy();

  m.doc() = R"pbdoc(
        ORB_SLAM bindings
        -----------------------

        .. currentmodule:: orb_slam2

        .. autosummary::
           :toctree: _generate
           slamOnImageFilenamesList
    )pbdoc";

  //'System class'
  py::class_<ORB_SLAM2::System> orb_slam2_system_class(m, "OrbSlamSystem");

  py::enum_<ORB_SLAM2::System::eSensor>(orb_slam2_system_class, "eSensor")
      .value("MONOCULAR", ORB_SLAM2::System::eSensor::MONOCULAR)
      .value("STEREO", ORB_SLAM2::System::eSensor::STEREO)
      .value("RGBD", ORB_SLAM2::System::eSensor::RGBD)
      .export_values();

  orb_slam2_system_class.def(py::init<const string, const string, const ORB_SLAM2::System::eSensor, const bool>(),
          py::arg("strVocFile") = "ORB_SLAM/Vocabulary/ORBvoc.txt",
          py::arg("strSettingsFile") = "python_slam/default_config.yaml",
          py::arg("sensor") = ORB_SLAM2::System::eSensor::MONOCULAR,
          py::arg("bUseViewer") = false);

  orb_slam2_system_class.def("TrackMonocular", &ORB_SLAM2::System::TrackMonocular);
  orb_slam2_system_class.def("Reset", &ORB_SLAM2::System::Reset);
  orb_slam2_system_class.def("GetTrackingState", &ORB_SLAM2::System::GetTrackingState);
  orb_slam2_system_class.def("GetTrackedMapPoints", &ORB_SLAM2::System::GetTrackedMapPoints);
  orb_slam2_system_class.def("GetMap", &ORB_SLAM2::System::GetMap);
  orb_slam2_system_class.def("GetTrackedKeyPointsUn", &ORB_SLAM2::System::GetTrackedKeyPointsUn);

  //'Map class'
  py::class_<ORB_SLAM2::Map> map_class(m, "Map");
  map_class.def("GetAllMapPoints", &ORB_SLAM2::Map::GetAllMapPoints);

  //'MapPoint class'
  py::class_<ORB_SLAM2::MapPoint> map_point_class(m, "MapPoint");
  map_point_class.def("GetWorldPos", &ORB_SLAM2::MapPoint::GetWorldPos);
  map_point_class.def("GetNormal", &ORB_SLAM2::MapPoint::GetNormal);

  //'KeyFrame class'
  //py::class_<ORB_SLAM2::KeyFrame> key_frame_class(m, "KeyFrame");
  //key_frame_class.def("GetNormal", &ORB_SLAM2::MapPoint::GetNormal);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
