///////////////////////////////////////////////////////////////////////////
// ZED SDK 2D Bounding Box Point Cloud Filter Example (Simplified GUI - Optimized, No Skeleton)
// 
// This program demonstrates how to:
//   1. Use the ZED object detection module to generate a 2D bounding box
//      (or union of multiple boxes) around the detected object(s).
//   2. Filter the full 3D point cloud so that only the points whose image
//      coordinates lie inside that 2D box are kept (preserving the native color).
//   3. Display the filtered point cloud in the 3D viewer.
//   4. Print the FPS (frames per second) to the console.
//   5. Produce Mesh file in.ply format by pressing the 's' key.
//   6. The mesh file is saved in the same directory as the script
//      - The mesh file is named "generated_mesh{filenum}.ply"
// 	
// Performance improvements include clearing the output buffer once,
// restricting processing to the region of interest, and using OpenMP to
// parallelize the filtering loop.
// 
// Note: This code requires the ZED SDK and sample utilities (e.g., GLViewer.hpp,
//       render_2D()) as well as OpenCV. Compile with OpenMP support.
///////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <chrono>        // For FPS measurement
#include <cstring>       // For memcpy, memset
#include <algorithm>     // For std::min, std::max
#include <limits>        // For numeric_limits
#include <cstdlib>
#include <Python.h>
//#include <open3d/Open3D.h>

// OpenCV includes (video feed window code is commented out below).
#include <opencv2/opencv.hpp>

// Define ENABLE_GUI to 1 to enable the OpenGL viewer.
#define ENABLE_GUI 1

// ZED SDK includes.
#include <sl/Camera.hpp>

// Sample includes for 3D visualization.
#if ENABLE_GUI
#include "GLViewer.hpp"
// (TrackingViewer is not used in this simplified version.)
#endif
#include <TrackingViewer.hpp>  // (Included for render_2D() utility, if needed)

#ifdef _OPENMP
#include <omp.h>
#endif



using namespace std;
using namespace sl;

// Global variable to indicate if playback (SVO file input) is used.
bool is_playback = false;

bool called_python = false;
int fileCounter = 0;

// Function declarations.
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char** argv, InitParameters& param);

void initialize() {
    Py_Initialize();
    if (!Py_IsInitialized) {
        std::cerr << "Initialization failed" << endl;
        exit(1);
    }
    PyObject* sys_path = PySys_GetObject("path");
    PyList_Append(sys_path, PyUnicode_FromString("C:/Users/capstone/Desktop/ZED_Point_cloud_filtered/PointCloud/src"));
}

void finalize() {
    Py_Finalize();
}

int main(int argc, char** argv) {
#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif

    //---------------------------------------------------------------------------
    // 1. Initialize the ZED Camera
    //---------------------------------------------------------------------------
    Camera zed;  // Create a ZED camera object.

    // Set initialization parameters.
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::NEURAL; // Use NEURAL depth mode.
    init_parameters.depth_minimum_distance = 0.2f * 1000.0f;
    init_parameters.depth_maximum_distance = 0.85f * 1000.0f;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.coordinate_units = UNIT::MILLIMETER;
    init_parameters.sdk_verbose = 1;
    init_parameters.camera_resolution = RESOLUTION::HD720;
    parseArgs(argc, argv, init_parameters);

    // Open the camera.
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }

    // Enable positional tracking.
    auto camera_config = zed.getCameraInformation().camera_configuration;
    PositionalTrackingParameters positional_tracking_parameters;
    // Optionally set as static:
    // positional_tracking_parameters.set_as_static = true;
    zed.enablePositionalTracking(positional_tracking_parameters);

    //---------------------------------------------------------------------------
    // 2. Enable Detection Modules
    //---------------------------------------------------------------------------
    // 2.1 Enable Body Tracking module (for skeleton detection).
    print("Skeleton Detection: Loading Module...");
    BodyTrackingParameters body_tracking_parameters;
    body_tracking_parameters.enable_tracking = true;
    body_tracking_parameters.enable_segmentation = true; // (Skeleton data available but will not be used)
    body_tracking_parameters.detection_model = BODY_TRACKING_MODEL::HUMAN_BODY_MEDIUM;
    returned_state = zed.enableBodyTracking(body_tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enableBodyTracking", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    // 2.2 Enable Object Detection module (for 2D bounding boxes).
    ObjectDetectionParameters object_detection_parameters;
    object_detection_parameters.enable_tracking = true;
    object_detection_parameters.enable_segmentation = false;
    object_detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    object_detection_parameters.instance_module_id = 1;
    print("Object Detection: Loading Module...");
    returned_state = zed.enableObjectDetection(object_detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    // Runtime thresholds.
	int detection_confidence_od = 80; // For object detection. The higher the value, the more accurate the detection and the lower the probability of successful detection.
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence_od);
    int body_detection_confidence = 60; // For body tracking. The higher the value, the more accurate the detection and the lower the probability of successful detection.
    BodyTrackingRuntimeParameters body_tracking_parameters_rt(body_detection_confidence);

    //---------------------------------------------------------------------------
    // 3. Setup GUI Components and Allocate Memory (for 3D viewer only)
    //---------------------------------------------------------------------------
#if ENABLE_GUI
    // (Video feed window code is defined below but will be commented out.)
    // Setup a lower resolution for the point cloud (to improve performance).
    int requested_low_res_w = min(720, (int)camera_config.resolution.width);
    // Preserve the aspect ratio from the camera.
    Resolution pc_resolution(requested_low_res_w, requested_low_res_w * camera_config.resolution.height / camera_config.resolution.width);

    // Retrieve camera calibration for the left camera (for 3D viewer).
    auto camera_parameters = zed.getCameraInformation(pc_resolution).camera_configuration.calibration_parameters.left_cam;

    // Allocate sl::Mat objects for the full point cloud and the filtered point cloud.
    Mat point_cloud(pc_resolution, MAT_TYPE::F32_C4, MEM::CPU);
    Mat filtered_point_cloud(pc_resolution, MAT_TYPE::F32_C4, MEM::CPU);

    // Initialize the OpenGL viewer for 3D display.
    GLViewer viewer;
    viewer.init(argc, argv, camera_parameters, body_tracking_parameters.enable_tracking);

    // The following code creates the video feed window.
    // It is commented out to reduce overhead.
    
    // Compute display resolution (maintaining aspect ratio) for the video feed.
    float image_aspect_ratio = camera_config.resolution.width / (1.f * camera_config.resolution.height);
    int requested_video_w = min(1280, (int)camera_config.resolution.width);
    sl::Resolution display_resolution(requested_video_w, requested_video_w / image_aspect_ratio);

    // Create a simple video feed image (left image only).
    cv::Mat video_feed(display_resolution.height, display_resolution.width, CV_8UC4);
    // Create an sl::Mat wrapper for the left image.
    Mat image_left(display_resolution, MAT_TYPE::U8_C4, video_feed.data, video_feed.step);
    // Calculate scale factor for overlay purposes.
    sl::float2 img_scale(display_resolution.width / (float)camera_config.resolution.width,
                          display_resolution.height / (float)camera_config.resolution.height);
    // Create a window that shows the video feed.
    string window_name = "ZED | Video Feed";
    cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    
#endif

    //---------------------------------------------------------------------------
    // 4. Setup Runtime Parameters for Grab and Define Variables
    //---------------------------------------------------------------------------
    RuntimeParameters runtime_parameters;
    runtime_parameters.confidence_threshold = 80;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

    Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    // Containers for detection results.
    Objects objects;
    Bodies skeletons;  // This will remain empty so that no skeleton is shown.

    bool quit = false;
#if ENABLE_GUI
    bool gl_viewer_available = true;
#endif

    // For FPS calculation.
    auto t_prev = std::chrono::high_resolution_clock::now();
    float fps = 0.0f;

    //---------------------------------------------------------------------------
    // 5. Main Processing Loop
    //---------------------------------------------------------------------------
    while (
#if ENABLE_GUI
        gl_viewer_available &&
#endif
        !quit)
    {
        // Grab a new image, depth, and measurements.
        auto grab_state = zed.grab(runtime_parameters);
        if (grab_state == ERROR_CODE::SUCCESS) {

            // --- FPS Calculation ---
            auto t_now = std::chrono::high_resolution_clock::now();
            float deltaTime = std::chrono::duration<float, std::milli>(t_now - t_prev).count();
            fps = 1000.0f / deltaTime;
            t_prev = t_now;
            // Print the FPS to the console.
            cout << "FPS: " << static_cast<int>(fps) << "\n" << flush;

            // --- Retrieve Detection Results ---
            detection_parameters_rt.detection_confidence_threshold = detection_confidence_od;
            returned_state = zed.retrieveObjects(objects, detection_parameters_rt,
                object_detection_parameters.instance_module_id);

            // Disable skeleton retrieval by not calling retrieveBodies.
            // Commenting out the following lines means skeletons will remain empty.
            // body_tracking_parameters_rt.detection_confidence_threshold = body_detection_confidence;
            // returned_state = zed.retrieveBodies(skeletons, body_tracking_parameters_rt,
            //                                     body_tracking_parameters.instance_module_id);
            skeletons.body_list.clear(); // Ensure skeleton list is empty.

#if ENABLE_GUI
			//Start the latency timer
            auto t_filter_start = std::chrono::high_resolution_clock::now();
            // --- Compute the 2D Bounding Box from Object Detection ---
            float union_min_x = std::numeric_limits<float>::max();
            float union_min_y = std::numeric_limits<float>::max();
            float union_max_x = 0;
            float union_max_y = 0;
            bool valid_bbox = false;
            for (auto& obj : objects.object_list) {
                for (auto& pt : obj.bounding_box_2d) {
                    union_min_x = std::min<float>(union_min_x, pt.x);
                    union_min_y = std::min<float>(union_min_y, pt.y);
                    union_max_x = std::max<float>(union_max_x, pt.x);
                    union_max_y = std::max<float>(union_max_y, pt.y);
                    valid_bbox = true;
                }
            }
            if (!valid_bbox) {
                // If no objects detected, use full image bounds.
                union_min_x = 0;
                union_min_y = 0;
                union_max_x = camera_config.resolution.width;
                union_max_y = camera_config.resolution.height;
            }

            // Convert the bounding box from camera resolution to point cloud resolution.
            float scaleX = static_cast<float>(pc_resolution.width) / static_cast<float>(camera_config.resolution.width);
            float scaleY = static_cast<float>(pc_resolution.height) / static_cast<float>(camera_config.resolution.height);
            int pc_bbox_min_x = std::max<int>(0, static_cast<int>(union_min_x * scaleX));
            int pc_bbox_min_y = std::max<int>(0, static_cast<int>(union_min_y * scaleY));
            int pc_bbox_max_x = std::min<int>(pc_resolution.width - 1, static_cast<int>(union_max_x * scaleX));
            int pc_bbox_max_y = std::min<int>(pc_resolution.height - 1, static_cast<int>(union_max_y * scaleY));

            // --- Retrieve the Full Point Cloud ---
            zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, pc_resolution);

            // --- Filter the Point Cloud Using the 2D Bounding Box (Optimized) ---
            // Clear the entire filtered point cloud buffer.
            memset(filtered_point_cloud.getPtr<sl::float4>(MEM::CPU), NAN,
                pc_resolution.width * pc_resolution.height * sizeof(sl::float4));
            // Only process the region inside the computed bounding box.
#pragma omp parallel for collapse(2)
            for (int py = pc_bbox_min_y; py <= pc_bbox_max_y; ++py) {
                for (int px = pc_bbox_min_x; px <= pc_bbox_max_x; ++px) {
                    sl::float4 pcValue;
                    point_cloud.getValue<sl::float4>(px, py, &pcValue, MEM::CPU);
                    if (std::isnormal(pcValue.z)) {
                        filtered_point_cloud.setValue<sl::float4>(px, py, pcValue, MEM::CPU);
                    }
                }
            }
            filtered_point_cloud.updateGPUfromCPU();

            // --- (Optional) 2D Overlay Rendering for Video Feed ---
            // The following can be commented out to reduce overhead.
            
            zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);
            render_2D(video_feed, img_scale, objects, skeletons,
                      true, body_tracking_parameters.enable_tracking);
            std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
            cv::putText(video_feed, fpsText, cv::Point(20, 40),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::imshow(window_name, video_feed);  
            char key = cv::waitKey(20);
            if (key == 'q') {
                
                if (called_python == true) {
                    finalize();
                }

                quit = true;
            }
            else if (key == 's') {
                //sl::Mat filtered_point_cloud;
				//The retriveMeasure method retrives the filtered point cloud
                zed.retrieveMeasure(filtered_point_cloud, MEASURE::XYZRGBA);
                auto write_suceed = filtered_point_cloud.write("Filtered_Pointcloud.ply");
                if (write_suceed == sl::ERROR_CODE::SUCCESS)
                    std::cout << "Current filtered_point_cloud.ply file saving succeed" << std::endl;
                else
                    std::cout << "Current filtered_point_cloud.ply file saving failed" << std::endl;

                const int width = filtered_point_cloud.getWidth();

                const int height = filtered_point_cloud.getHeight();

                std::vector<std::vector<float>> points;
                //Start the f
                for (int y = 0; y < height; y++) {
                    for (int x = 0; x < width; x++) {
                        sl::float4 point;
                        filtered_point_cloud.getValue(x, y, &point);

                        uint32_t color = *reinterpret_cast<uint32_t*>(&point.w);

                        float b = (color & 0x000000FF) / 255.0;
                        float g = ((color & 0x0000FF00) >> 8 ) / 255.0;
                        float r = ((color & 0x00FF0000) >> 16) / 255.0;


                        if (point.z != 0 && !std::isinf(point.z) && !std::isnan(point.z)) {
                            points.push_back({point.x, point.y, point.z, b, g, r, (float)fileCounter});
                        }
                    }
                }

                fileCounter++;

                initialize();

                called_python = true;

                PyObject* pyList = PyList_New(points.size());
                for (size_t i = 0; i < points.size(); i++) {
                    PyObject* pyPoint = PyTuple_New(7);
                    PyTuple_SetItem(pyPoint, 0, PyFloat_FromDouble(points[i][0]));
                    PyTuple_SetItem(pyPoint, 1, PyFloat_FromDouble(points[i][1]));
                    PyTuple_SetItem(pyPoint, 2, PyFloat_FromDouble(points[i][2]));
                    PyTuple_SetItem(pyPoint, 3, PyFloat_FromDouble(points[i][3]));
                    PyTuple_SetItem(pyPoint, 4, PyFloat_FromDouble(points[i][4]));
                    PyTuple_SetItem(pyPoint, 5, PyFloat_FromDouble(points[i][5]));
                    PyTuple_SetItem(pyPoint, 6, PyFloat_FromDouble(points[i][6]));
                    PyList_SetItem(pyList, i, pyPoint); 
                }

                PyObject* pName = PyUnicode_DecodeFSDefault("wrapper");
                PyObject* pModule = PyImport_Import(pName);
                Py_XDECREF(pName);
                if (!pModule) {
                    PyErr_Print();
                    std::cerr << "Failed to load Cython wrapper" << std::endl;
                    finalize();
                }
                else {
                    PyObject* pFunc = PyObject_GetAttrString(pModule, "call_python_function");
                    if (PyCallable_Check(pFunc)) {
                        PyObject* pResult = PyObject_CallFunctionObjArgs(pFunc, pyList, NULL);
                        if (pResult) {
                            std::cout << "Python Function returned successfully" << std::endl;
                            Py_XDECREF(pResult);
                        }
                        else {
                            PyErr_Print();
                            std::cerr << "Failed to Call Function" << std::endl;
                        }
                    }
                    else {
                        PyErr_Print();
                        std::cerr << "Python function not found" << std::endl;
                    }

                    Py_XDECREF(pFunc);
                    Py_XDECREF(pModule);
                }

                }
   

            // --- 3D Viewer Update ---
            //zed.getPosition(cam_w_pose, REFERENCE_FRAME::WORLD);
            sl::Transform transform;
            transform.setIdentity();
            // clear the 3D bounding box list
            objects.object_list.clear();
            // Pass the (empty) skeletons so that no skeleton is shown.
			//Watch the live filtered point cloud and camera pose in open 3D viewer
            viewer.updateData(filtered_point_cloud, objects, skeletons, cam_w_pose.pose_data);
			//Stop the timer and calculate the latency time
            auto t_filter_end = std::chrono::high_resolution_clock::now();
            float filter_time = std::chrono::duration<float, std::milli>(t_filter_end - t_filter_start).count();
            std::cout << "Latency Time: " << filter_time << " ms\n" << flush;
#endif  // ENABLE_GUI

            if (is_playback && zed.getSVOPosition() == zed.getSVONumberOfFrames())
                quit = true;
        } // End if (grab_state == SUCCESS)

#if ENABLE_GUI
        gl_viewer_available = viewer.isAvailable();
#endif
    } // End main loop

    //---------------------------------------------------------------------------
    // 6. Cleanup and Shutdown
    //---------------------------------------------------------------------------
#if ENABLE_GUI
    viewer.exit();
    filtered_point_cloud.free();
    point_cloud.free();
    // image_left.free(); // (Uncomment if video feed is re-enabled)
#endif
    zed.disableObjectDetection();
    zed.close();
    return EXIT_SUCCESS;
}

//------------------------------------------------------------------------------
// print: Utility function to print messages along with error code info.
//------------------------------------------------------------------------------
void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample] ";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : " << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

//------------------------------------------------------------------------------
// parseArgs: Parses command-line arguments to set input mode and camera resolution.
//------------------------------------------------------------------------------
void parseArgs(int argc, char** argv, InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        param.input.setFromSVOFile(argv[1]);
        is_playback = true;
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    }
    else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            string ip_address = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_address.c_str()), port);
            cout << "[Sample] Using Stream input, IP: " << ip_address << ", port: " << port << endl;
        }
        else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP: " << argv[1] << endl;
        }
        else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        }
        else if (arg.find("HD1200") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1200;
            cout << "[Sample] Using Camera in resolution HD1200" << endl;
        }
        else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        }
        else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        }
        else if (arg.find("SVGA") != string::npos) {
            param.camera_resolution = RESOLUTION::SVGA;
            cout << "[Sample] Using Camera in resolution SVGA" << endl;
        }
        else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
}
