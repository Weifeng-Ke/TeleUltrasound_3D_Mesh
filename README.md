# Tele-Ultrasound 3D Mesh

Welcome to the set-up documentation for the 3D Mesh project!

## Purpose: 
This project is intended to be used with the ZED Mini stereo camera in order to produce Point Clouds and 3D meshes of a patient. These meshes can be used to provide the Tele-Ultrasound experts with a 3D visual and haptic feedback for improved orientation in virtual space.


# Point Cloud Project Setup Instructions (C++)
Below is step by step guide on how to set up, build and run the Point Cloud Project on a Windows device

## Point Cloud Project Dependencies
1. ZED SDK
   * The ZED SDK is a development toolkit from Stereolabs that enables depth sensing, 3D mapping, and spatial tracking using ZED stereo cameras.
   * Download the latest [ZED SDK](https://www.stereolabs.com/developers/release/)
   * Refer to the ZED SDK [Documentation](https://www.stereolabs.com/docs/) for more information
2. CMAKE (3.5.0+)
   * CMake is a build system generator that manages project configuration and creates build files from a single source.
   * Download the latest version of [CMake](https://cmake.org/download/)
   * Refer to the CMAKE [Documentation](https://cmake.org/documentation/) for more information
3. Visual Studio (Version 2015+ x64)
   * Download the 64-bit version of [Visual Studio 2015 or newer](https://learn.microsoft.com/en-us/visualstudio/releases/2022/release-notes)
   * Select the VisualC++ option while installing Visual Studio
4. Python 3.11
   * Download [Python 3.11]([https://www.python.org/downloads/](https://www.python.org/downloads/release/python-3110/))
   * Select "Add to PATH" while installing python
   * *NOTE* Please ensure that you are using Python 3.11 for compatibility for Open3D

## Point Cloud Project Setup 
1. Clone this repository
2. Make sure that the project includes the following directories and files:
   * TeleUltrasound_3D_Mesh/PointCloud/CMakeLists.txt
   * TeleUltrasound_3D_Mesh/PointCloud/src/
   * TeleUltrasound_3D_Mesh/PointCloud/include/

### Building the PointCloud Project on Windows
Before building, please ensure that all project dependencies have been installed. Building this project requires [CMake](https://cmake.org/) (3.5.0+) and [Visual Studio 2015+ 64-bit](https://learn.microsoft.com/en-us/visualstudio/releases/2022/release-notes).
 1. Open the **CMake-gui** application
 2. In “Where is the source code“, provide the path of the project folder where the CMakeLists.txt is located. The path should be similar to: \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud
 3. In “Where to build the binaries“, provide the previous path followed by /build. The path should be similar to: \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud/build
 4. Click on "Configure"

![CMake_configure](https://github.com/user-attachments/assets/abc34ed1-9dec-4ba8-9e80-e54f42468e01)

 5. A dialog window asks you if CMake can create the “build” folder. Select "Yes"

![Create_New_Directory](https://github.com/user-attachments/assets/e1bcc2c2-799b-4de1-bf5e-22d0b7ab1aa1)

 6. Another dialog window will ask you to specify a generator for your project. Choose Visual Studio in Win64 and click on [Finish]

![CMake_Generator_Selection](https://github.com/user-attachments/assets/41b7f79d-6789-4163-a902-b3b9758a39b3)

 7. (CMake may take a few seconds) Click on [Generate] to build the Visual Studio project files

![CMake_Generate_Button](https://github.com/user-attachments/assets/8257af76-2d5c-48b5-ac31-15ee1950e3ce)

 8. CMake has now generated the project in the build directory. The solution can be accessed by clicking on [Open Project] or by opening ZED_Patient_3D.sln

![CMake_Open_Project](https://github.com/user-attachments/assets/58faffee-86e2-4de5-a5ae-d88c20a92899)

![CMake_Build_Directory](https://github.com/user-attachments/assets/5f5c11f7-1766-4b45-9eb4-b29e82458c42)

 9. In the Visual Studio solution, set "ZED_Patient_3D" as the "Startup Project"

![VS_Startup_Project](https://github.com/user-attachments/assets/6829fe2d-3347-44c1-9c95-2fd7fab8f576)

10. In the Visual Studio solution, "Solution Configuration" to "Release"

![VS_Solution_Config_Release](https://github.com/user-attachments/assets/06bbd9e3-da4e-4b8c-9fd0-8286eb3c631c)

11. Build the Visual Studio Project from the Build menu or from keyboard shortcut "CTRL+SHIFT+B"

![VS_Build](https://github.com/user-attachments/assets/c1840f6f-8397-4a67-af6e-49e494367c15)

12. You can now edit and compile your program in the Visual Studio IDE. Hit the Ctrl+F5 key to launch the program.

### For more information about building projects involving the ZED IDE see the following documentation:
 - Build for [Windows](https://www.stereolabs.com/docs/app-development/cpp/windows/)
 - Build for [Linux/Jetson](https://www.stereolabs.com/docs/app-development/cpp/linux/)


## Run the Point Cloud Program
- Navigate to the build directory and launch the executable
- Or open a terminal in the build directory and run the executable:
````
      ./ZED_Patient_3D
````
# Point Cloud Code Parameters
The Point Cloud code resides within the main.cpp file.
The code can be altered to change the depth cutoff for point cloud generation. 
The minimum depth cutoff can be specified by setting init_parameters.depth_minimum_distance to the desired minimum depth (in mm) on line 97:
````cpp
    init_parameters.depth_minimum_distance = 0.2f * 1000.0f;
````
The maximum depth cutoff can be specified by setting init_parameters.depth_maximum_distance to the desired maximum depth (in mm) on line 98:
````cpp
    init_parameters.depth_maximum_distance = 0.85f * 1000.0f;
````
The confidence cutoffs for object detection and body tracking can be altered as well.
The object detection confidence cutoff can be specified by setting detection_confidence_od to the desired cutoff (in %) on line 150:
````cpp
    int detection_confidence_od = 80;
````
The body tracking confidence cutoff can be specified by setting body_detection_confidence to the desired cutoff (in %) on line 152:
````cpp
    int body_detection_confidence = 60;
````
The generated full point clouds and filtered point clouds can be accessed through the point_cloud and filtered_point_cloud objects respectively.
The full point cloud is retrieved on line 288:
````cpp
    zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA, MEM::CPU, pc_resolution);
````
The filtered point cloud is generated from the full point cloud on lines 296-305.

# Mesh Project Setup Instructions (Python)
Below is step by step guide on how to setup, build and run the Mesh Project on a Windows device

## Mesh Project Dependencies
1. Open3D
   * An open-source library for working with 3D data like point clouds and meshes.
   * To download this library, use the command `pip install open3d`
   * NOTE: Open3D runs on Python 3.11 or older as of the making of this file
   * Refer to the Open3D [Documentation](https://www.open3d.org/docs/release/) for more information
2. Cython
   * A tool that lets you compile Python code into C extensions.
   * To download this library, use the command `pip install Cython`
   * Refer to the Cython [Documentation](https://cython.readthedocs.io/en/latest/) for more information
3. NumPy
   * A library for numerical computing in Python.
   * To download this library, use the command `pip install numpy`
   * Refer to the NumPy [Documentation](https://numpy.org/doc/stable/) for more information
4. setuptools (usually installed with pip but should be verified)
   * A package development and distribution library used to build, package, and install Python projects.
   * To download this library, use the command `pip install setuptools`
   * Refer to the Cython [Documentation](https://setuptools.pypa.io/en/latest/) for more information

## Mesh Project Setup
1. Make sure that the project includes the following files:
   * TeleUltrasound_3D_Mesh/PointCloud/src/Mesh.py
   * TeleUltrasound_3D_Mesh/PointCloud/src/setup.py
   * TeleUltrasound_3D_Mesh/PointCloud/src/wrapper.pyx
2. In a command line, navigate to the /src directory containing setup.py and mesh.py. The path should be similar to \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud/src
3. Setup the project by running the command 'setup.py build_ext --inplace'
4. Verify that wrapper.c and a .pyd file have been generated
5. Ensure that main.cpp properly inlcudes Python.h
6. Ensure that the path variable is reflective of the actual path of the src directory on main.cpp line 75:
````cpp
    PyList_Append(sys_path, PyUnicode_FromString("C:/Users/capstone/Desktop/ZED_Point_cloud_filtered/PointCloud/src"));
````
7. Rebuild the Point Cloud (C++) project using Cmake
8. The Mesh.py program is now callable by main.cpp

# Mesh Code Parameters
The Mesh code resides within the Mesh.py file.
The code can be altered to change the generated mesh resolution and generation speed. 
The voxel downsizing can be altered by changing voxel_size in line 31:
````
    cloud = cloud.voxel_down_sample(voxel_size = 3)
````
Larger values will lead to lower resolutions and faster generation speeds. 

Resolution and generation speed can also be effectively altered by changing the radii used for the Ball Pivoting Algorithm.
This can be found on line 68:
````
    radii = o3d.utility.DoubleVector([avg_spacing * 3, avg_spacing * 3.5, avg_spacing * 4, avg_spacing*4.5, avg_spacing*5]) 
````
\***Note** While the radii values can be any arbitrary number, it is recommended to keep them multiples of the average spacing of points \*

Radii values can be individually changed and can be added or removed. More radii values leads to less holes but longer generation speeds. Larger values lead to lower resolution and faster generation speeds. 
## Additional Features
Commented out on line 23 is code that allows for the program to instead read point cloud data from a file. To run and test this feature, the code must be removed from the processMesh() function. 

Commented out on lines 45, 89-120 is the rough code for parallelization of the mesh function. The current implementation of this feature does not function with the point cloud generation integretion method (ie. the use of a Cython wrapper). 

Commented out on lines 50-51 is code for alternative methods of aligning normal orientations. These orient normals either towards a set direction or towards a "camera" perspective. 

# Hardware Setup
Below is step by step guide on how to setup the hardware for this project

## Hardware Requirements
1. Lab Laptop
2. ZED Mini Stereo Camera
3. USB-C to USB-C cable (USB 3.0 or faster)
4. Tripod
5. Tripod Clamp (to hold the ZED Mini)

## Setting Up the Hardware
1. Set up the Tripod
2. Attach the Tripod Clamp to the Tripod
3. Secure the ZED Mini in the Tripod Clamp
4. Connect the ZED Mini to the Lab Laptop using the USB-C to USB-C cable
   * A "Change Camera" icon should appear in the top right of the screen.
   * If the indicator is not present, try unplugging and flipping the USB-C connection on the ZED Mini side (unplug and turn the USB-C port upside down).
   * The ZED Mini is sensitive to the orientation of the USB-C cable

![Camera_Flip_Camera_Indicator](https://github.com/user-attachments/assets/317d9c56-e294-40f0-a2a5-162a4a8c8c1e)

5. Use the tripod to position the ZED Mini such that the patient is in view of the camera
