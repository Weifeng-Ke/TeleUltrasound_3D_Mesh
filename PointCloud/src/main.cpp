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
//   5. Save the filtered point cloud to a file.
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
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <Python.h>
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

// Function declarations.
void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char** argv, InitParameters& param);

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
	//The two lines below are used to set the depth range for the camera.(Anything outside this range is not captured)
    init_parameters.depth_minimum_distance = 0.2f * 1000.0f;
    init_parameters.depth_maximum_distance = 2.0f * 1000.0f;
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
    // 2.1 Enable Object Detection module (for 2D bounding boxes).
    ObjectDetectionParameters object_detection_parameters;
    object_detection_parameters.enable_tracking = true;
    object_detection_parameters.enable_segmentation = true;
    object_detection_parameters.detection_model = OBJECT_DETECTION_MODEL::MULTI_CLASS_BOX_MEDIUM;
    object_detection_parameters.instance_module_id = 0;
	
    print("Object Detection: Loading Module...");
    returned_state = zed.enableObjectDetection(object_detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }

    // Runtime thresholds.
    int detection_confidence_od = 65; // For object detection.
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence_od);
    detection_parameters_rt.object_class_filter = { OBJECT_CLASS::PERSON };

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
    viewer.init(argc, argv, camera_parameters, false);



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
    runtime_parameters.confidence_threshold = 65;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;

    Pose cam_w_pose;
    cam_w_pose.pose_data.setIdentity();

    // Containers for detection results.
	Objects objects;  // This will conatin the detected objects
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
            std::cout << "FPS: " << static_cast<int>(fps) << "\n" << flush;

            // --- Retrieve Detection Results ---
            detection_parameters_rt.detection_confidence_threshold = detection_confidence_od;
            returned_state = zed.retrieveObjects(objects, detection_parameters_rt,
                object_detection_parameters.instance_module_id);

            sl::float3 personPos = objects.object_list[0].position;
			std::vector<sl::float3>bbox3d = objects.object_list[0].bounding_box;
            float z_min = -std::numeric_limits<float>::min();
            float z_max = -std::numeric_limits<float>::max();
            for (size_t i = 0; i < bbox3d.size(); i++) {
                if (bbox3d[i].z < z_min) z_min = bbox3d[i].z;
                if (bbox3d[i].z > z_max) z_max = bbox3d[i].z;
            }
            // Define a tolerance (in millimeters, adjust as needed)
            float zTolerance = 500.0f;
            z_min -= zTolerance;
            z_max += zTolerance;
#if ENABLE_GUI
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
            // Convert the sl::Mat mask to a cv::Mat using the utility function.
            cv::Mat mask_cv = slMat2cvMat(objects.object_list[0].mask);
            // Resize the mask to match the point cloud resolution (using nearest-neighbor interpolation to preserve binary values). 
            cv::Mat resized_mask_cv;
            cv::resize(mask_cv, resized_mask_cv, cv::Size(point_cloud.getWidth(), point_cloud.getHeight()), 0, 0, cv::INTER_NEAREST);

            int width = point_cloud.getWidth();
            int height = point_cloud.getHeight();

            // Loop through each pixel in the full point cloud.
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    // Retrieve the current point from the full point cloud.
                    sl::float4 point;
                    point_cloud.getValue<sl::float4>(x, y, &point, sl::MEM::CPU);

                    // Retrieve the corresponding mask value.
                    // (the mask is stored as a single-channel 8-bit image.8UC1)
                    uchar maskValue = 0;
                    maskValue = resized_mask_cv.at<uchar>(y, x);

                    if (maskValue > 0 && (point.z >= z_min && point.z <= z_max)) {
                        // If the mask is active (nonzero), keep the original point.
                        filtered_point_cloud.setValue<sl::float4>(x, y, point, sl::MEM::CPU);
                    }
                    else {
                        // Otherwise, set the point to an invalid value (e.g., NAN) so it won't be rendered.
                        filtered_point_cloud.setValue<sl::float4>(x, y, sl::float4(NAN, NAN, NAN, NAN), sl::MEM::CPU);
                    }
                }
            }
            filtered_point_cloud.updateGPUfromCPU();

            // --- 2D Overlay Rendering for Video Feed ---
            // The following block is commented out to reduce overhead.

            zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);
            render_2D(video_feed, img_scale, objects, skeletons,
                true, false);
            std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
            cv::putText(video_feed, fpsText, cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::imshow(window_name, video_feed);
            char key = cv::waitKey(20);
            if (key == 'q') {
                quit = true;
			}
            else if (key == 's') {
                //sl::Mat filtered_point_cloud;
                zed.retrieveMeasure(filtered_point_cloud, MEASURE::XYZRGBA);
                auto write_suceed = filtered_point_cloud.write("Filtered_Pointcloud.ply");
                if (write_suceed == sl::ERROR_CODE::SUCCESS)
                    std::cout << "Current filtered_point_cloud.ply file saving succeed" << std::endl;
                else
                    std::cout << "Current filtered_point_cloud.ply file saving failed" << std::endl;

            }
            // --- 3D Viewer Update ---
            //zed.getPosition(cam_w_pose, REFERENCE_FRAME::WORLD);
            sl::Transform transform;
            transform.setIdentity();
            // clear the 3D bounding box list
            objects.object_list.clear();
            
            //viewer.updateData(point_cloud, objects, skeletons, transform);
            // Display the filtered point cloud in the viewer.
            viewer.updateData(filtered_point_cloud, objects, skeletons, transform);

            auto t_filter_end = std::chrono::high_resolution_clock::now();
            float filter_time = std::chrono::duration<float, std::milli>(t_filter_end - t_filter_start).count();
			std::cout << "Filtering Time: " << filter_time << " ms\n" << flush;
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
