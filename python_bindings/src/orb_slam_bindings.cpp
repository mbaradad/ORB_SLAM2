#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
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

  //'Map class'
  py::class_<ORB_SLAM2::Map> map_class(m, "Map");
  map_class.def("GetAllMapPoints", &ORB_SLAM2::Map::GetAllMapPoints);

  //'MapPoint class'
  py::class_<ORB_SLAM2::MapPoint> map_point_class(m, "MapPoint");
  map_point_class.def("GetWorldPos", &ORB_SLAM2::MapPoint::GetWorldPos);
  map_point_class.def("GetNormal", &ORB_SLAM2::MapPoint::GetNormal);

  //'Frame class'
  py::class_<ORB_SLAM2::Frame> frame_class(m, "Frame");
  frame_class.def("GetMVKeys", &ORB_SLAM2::Frame::GetMVKeys);
  frame_class.def("GetMVDescriptors", &ORB_SLAM2::Frame::GetMVDescriptors);
  frame_class.def("DropMVKeys", &ORB_SLAM2::Frame::DropMVKeys);
  frame_class.def("SetMVDescriptors", &ORB_SLAM2::Frame::SetMVDescriptors);

  //'KeyFrame class'
  py::class_<ORB_SLAM2::KeyFrame > key_frame_class(m, "KeyFrame");
  key_frame_class.def("GetTimestamp", &ORB_SLAM2::KeyFrame::GetTimestamp);

  //'TrackerConfig class'
  py::class_<ORB_SLAM2::TrackerConfig> orb_slam2_tracker_config_class(m, "TrackerConfig");
  orb_slam2_tracker_config_class.def(py::init(&ORB_SLAM2::TrackerConfig::defaultSettings));
  //all attributes read/writeable must be explicity said to be so
  orb_slam2_tracker_config_class.def_readwrite("Camera_fx", &ORB_SLAM2::TrackerConfig::Camera_fx);
  orb_slam2_tracker_config_class.def_readwrite("Camera_fy", &ORB_SLAM2::TrackerConfig::Camera_fy);
  orb_slam2_tracker_config_class.def_readwrite("Camera_cx", &ORB_SLAM2::TrackerConfig::Camera_cx);
  orb_slam2_tracker_config_class.def_readwrite("Camera_cy", &ORB_SLAM2::TrackerConfig::Camera_cy);

  orb_slam2_tracker_config_class.def_readwrite("Camera_k1", &ORB_SLAM2::TrackerConfig::Camera_k1);
  orb_slam2_tracker_config_class.def_readwrite("Camera_k2", &ORB_SLAM2::TrackerConfig::Camera_k2);
  orb_slam2_tracker_config_class.def_readwrite("Camera_p1", &ORB_SLAM2::TrackerConfig::Camera_p1);
  orb_slam2_tracker_config_class.def_readwrite("Camera_p2", &ORB_SLAM2::TrackerConfig::Camera_p2);
  orb_slam2_tracker_config_class.def_readwrite("Camera_k3", &ORB_SLAM2::TrackerConfig::Camera_k3);

  orb_slam2_tracker_config_class.def_readwrite("Camera_bf", &ORB_SLAM2::TrackerConfig::Camera_bf);
  orb_slam2_tracker_config_class.def_readwrite("Camera_fps", &ORB_SLAM2::TrackerConfig::Camera_fps);
  orb_slam2_tracker_config_class.def_readwrite("Camera_RGB", &ORB_SLAM2::TrackerConfig::Camera_RGB);

  orb_slam2_tracker_config_class.def_readwrite("ORBextractor_nFeatures", &ORB_SLAM2::TrackerConfig::ORBextractor_nFeatures);
  orb_slam2_tracker_config_class.def_readwrite("ORBextractor_scaleFactor", &ORB_SLAM2::TrackerConfig::ORBextractor_scaleFactor);
  orb_slam2_tracker_config_class.def_readwrite("ORBextractor_nLevels", &ORB_SLAM2::TrackerConfig::ORBextractor_nLevels);
  orb_slam2_tracker_config_class.def_readwrite("ORBextractor_iniThFAST", &ORB_SLAM2::TrackerConfig::ORBextractor_iniThFAST);
  orb_slam2_tracker_config_class.def_readwrite("ORBextractor_minThFAST", &ORB_SLAM2::TrackerConfig::ORBextractor_minThFAST);

  //'ORBVocabulary class'
  py::class_<ORB_SLAM2::ORBVocabulary> orb_slam2_orb_vocabulary_class(m, "ORBVocabulary");
  orb_slam2_orb_vocabulary_class.def("loadFromTextFile", &ORB_SLAM2::ORBVocabulary::loadFromTextFile);

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

  orb_slam2_system_class.def(py::init<const string, const string, ORB_SLAM2::TrackerConfig&, const ORB_SLAM2::System::eSensor, const bool>(),
                             py::arg("strVocFile") = "ORB_SLAM/Vocabulary/ORBvoc.txt",
                             py::arg("strSettingsFile") = "python_slam/default_config.yaml",
                             py::arg("config") = ORB_SLAM2::TrackerConfig::defaultSettings(),
                             py::arg("sensor") = ORB_SLAM2::System::eSensor::MONOCULAR,
                             py::arg("bUseViewer") = false);

  orb_slam2_system_class.def(py::init<ORB_SLAM2::ORBVocabulary&, const string, ORB_SLAM2::TrackerConfig&, const ORB_SLAM2::System::eSensor, const bool>(),
                             py::arg("strSettingsFile") = "python_slam/default_config.yaml",
                             py::arg("config") = ORB_SLAM2::TrackerConfig::defaultSettings(),
                             py::arg("sensor") = ORB_SLAM2::System::eSensor::MONOCULAR,
                             py::arg("bUseViewer") = false);

  orb_slam2_system_class.def("TrackMonocular", &ORB_SLAM2::System::TrackMonocular);
  orb_slam2_system_class.def("Reset", &ORB_SLAM2::System::Reset);
  orb_slam2_system_class.def("GetTrackingState", &ORB_SLAM2::System::GetTrackingState);
  orb_slam2_system_class.def("GetTrackedMapPoints", &ORB_SLAM2::System::GetTrackedMapPoints);
  orb_slam2_system_class.def("GetMap", &ORB_SLAM2::System::GetMap);
  orb_slam2_system_class.def("GetTrackedKeyPointsUn", &ORB_SLAM2::System::GetTrackedKeyPointsUn);
  orb_slam2_system_class.def("GetCurrentFrame", &ORB_SLAM2::System::GetCurrentFrame);
  orb_slam2_system_class.def("SetFrameForTrack", &ORB_SLAM2::System::SetFrameForTrack);
  orb_slam2_system_class.def("TrackCurrentFrame", &ORB_SLAM2::System::TrackCurrentFrame);
  orb_slam2_system_class.def("GetAllKeyFramesTimestamps", &ORB_SLAM2::System::GetAllKeyFramesTimestamps);
  orb_slam2_system_class.def("GetAllPoses", &ORB_SLAM2::System::GetAllPoses);
  orb_slam2_system_class.def("Shutdown", &ORB_SLAM2::System::Shutdown);
  orb_slam2_system_class.def("Reset", &ORB_SLAM2::System::Reset);


#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
