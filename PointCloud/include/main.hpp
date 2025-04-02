//#include <pybind11/pybind11.h>
//#include <pybind11/numpy.h>
//#include <sl/Camera.hpp>
//
//namespace py = pybind11;
//
//py::array_t<float> point_cloud_to_numpy(sl::Mat& point_cloud) {
//    std::cout << "entered function" << std::endl;
//
//    size_t width = point_cloud.getWidth();
//    size_t height = point_cloud.getHeight();
//    size_t channel = point_cloud.getChannels();
//    size_t totalSize = width * height * channel;
//
//    py::array_t<float> numpy_array({ height, width, channel });
//    float* np_data = (float*)numpy_array.mutable_data();
//    float* mat_data = point_cloud.getPtr<float>();
//
//    memcpy(np_data, mat_data, totalSize * sizeof(float));
//
//    return numpy_array;
//}