#include <pybind11/pybind11.h>
#include "mono_movie.cc"

namespace py = pybind11;
PYBIND11_MODULE(orb_slam2, m) {
    m.doc() = R"pbdoc(
        Pybind11 example plugin
        -----------------------

        .. currentmodule:: orb_slam2

        .. autosummary::
           :toctree: _generate

           slamOnImageFilenamesList
    )pbdoc";

    m.def("slamOnImageFilenamesList", &slamOnImageFilenamesList, R"pbdoc(
        slamOnImageFilenameList

        Do slam on the list of filenames
    )pbdoc");
  m.def("predefined_main", &predefined_main, R"pbdoc(
        predefined_main

        Do predefined_main
    )pbdoc");

  m.def("main_paths", &main_paths, R"pbdoc(
        main_paths

        Do main_paths
    )pbdoc");

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
