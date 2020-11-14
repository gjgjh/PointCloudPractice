#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include "kmeans.h"
#include "gmm.h"
#include "spectralcluster.h"

namespace py = pybind11;

PYBIND11_MODULE(pycluster, m) {
    m.doc() = "Python binding of my clustering algorithm";

    // 注册类
    py::class_<Kmeans> kmeans(m, "Kmeans");
    kmeans.def(py::init<int, double, int>());
    kmeans.def("fit", &Kmeans::fit);
    kmeans.def("predict", &Kmeans::predict);

    py::class_<GMM> gmm(m, "GMM");
    gmm.def(py::init<int, double, int>());
    gmm.def("fit", &GMM::fit);
    gmm.def("predict", &GMM::predict);
    gmm.def("getMu", &GMM::getMu);
    gmm.def("getVar", &GMM::getVar);

    py::class_<SpectralCluster> spectralCluster(m, "SpectralCluster");
    spectralCluster.def(py::init<int, int, double, int>());
    spectralCluster.def("fit", &SpectralCluster::fit);
    spectralCluster.def("predict", &SpectralCluster::predict);
}