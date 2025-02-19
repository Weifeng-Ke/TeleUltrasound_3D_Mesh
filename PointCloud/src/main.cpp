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
            cout << "FPS: " << static_cast<int>(fps) << "\n" << flush;

            // --- Retrieve Detection Results ---
            detection_parameters_rt.detection_confidence_threshold = detection_confidence_od;
            returned_state = zed.retrieveObjects(objects, detection_parameters_rt,
                object_detection_parameters.instance_module_id);

            

#if ENABLE_GUI
            auto t_filter_start = std::chrono::high_resolution_clock::now();
           
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
                    // (Assuming the mask is stored as a single-channel 8-bit image.)
                    uchar maskValue = 0;
                    maskValue = resized_mask_cv.at<uchar>(y, x);

                    if (maskValue > 50) {
                        // If the mask is active (nonzero), keep the original point.
                        filtered_point_cloud.setValue<sl::float4>(x, y, point, sl::MEM::CPU);
                    }
                    else {
                        // Otherwise, set the point to an invalid value (e.g., NAN) so it won't be rendered.
                        filtered_point_cloud.setValue<sl::float4>(x, y, sl::float4(NAN,NAN, NAN, NAN), sl::MEM::CPU);
                    }
                }
            }
            filtered_point_cloud.updateGPUfromCPU();

            // --- 2D Overlay Rendering for Video Feed ---
            // The following block is commented out to reduce overhead.

            zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);
            render_2D(video_feed, img_scale, objects, skeletons,
                true,false);
            std::string fpsText = "FPS: " + std::to_string(static_cast<int>(fps));
            cv::putText(video_feed, fpsText, cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::imshow(window_name, video_feed);
            char key = cv::waitKey(10);
            if (key == 'q') {
                quit = true;
            }


            // --- 3D Viewer Update ---
            //zed.getPosition(cam_w_pose, REFERENCE_FRAME::WORLD);
            sl::Transform transform;
            transform.setIdentity();
            // clear the 3D bounding box list
            objects.object_list.clear();
            // Pass the (empty) skeletons so that no skeleton is shown.
            //viewer.updateData(filtered_point_cloud, objects, skeletons, cam_w_pose.pose_data);
            //viewer.updateData(point_cloud, objects, skeletons, transform);
            // Display the filtered point cloud in the viewer.
            viewer.updateData(filtered_point_cloud, objects, skeletons, transform);

            auto t_filter_end = std::chrono::high_resolution_clock::now();
            float filter_time = std::chrono::duration<float, std::milli>(t_filter_end - t_filter_start).count();
			cout << "Filtering Time: " << filter_time << " ms\n" << flush;
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
    //filtered_point_cloud.free();
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
