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

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
