# Tele-Ultrasound

## Purpose: Help experts teleoperate remote ultrasound scanning by enhancing their depth perception.
This program allows the experts to see the patient's live point cloud and save the patient mesh by pressing the 's' key and moving, pan rotating the patient mesh to see a vitual 3D model of the patient. 
### Features
 - The camera point cloud is displayed in a 3D OpenGL view
 - 2D bounding boxes are used to filter out objects that's not the patient
 - Identifying Patient in real time
 - Produce Mesh file in .ply format by pressing the 's' key.
 - The mesh file is saved in the same directory as the script
	- The mesh file is named "generated_mesh{filenum}.ply"

![reversal](https://capsule-render.vercel.app/api?type=waving&text=How%20To%20Build%20Project&fontAlign=50&fontSize=60&desc=How%20To%20Run&descAlign=900&descAlignY=50&theme=gruvbox)

# Project Setup Instructions
Below is step by step guide on how to setup, build and run the project on a Windows device

## Dependencies
1. ZED SDK
   * The ZED SDK is a development toolkit from Stereolabs that enables depth sensing, 3D mapping, and spatial tracking using ZED stereo cameras.
   * Download the latest [ZED SDK](https://www.stereolabs.com/developers/release/)
   * Refer to the ZED SDK [Documentation](https://www.stereolabs.com/docs/) for more information
5. CMAKE (3.5.0+)
   * CMake is a build system generator that manages project configuration and creates build files from a single source.
   * Download the latest version of [CMAKE](https://cmake.org/download/)
   * Refer to the CMAKE [Documentation](https://cmake.org/documentation/) for more information
7. Visual Studio (Version 2015+ x64)
   * Download the 64-bit version of [Visual Studio 2015 or newer](https://learn.microsoft.com/en-us/visualstudio/releases/2022/release-notes)
   * Select the VisualC++ option while installing Visual Studio

## Setting up 
1. Clone this repository
2. Make sure that the project includes the following directories and files:
   * TeleUltrasound_3D_Mesh/PointCloud/CMakeLists.txt
   * TeleUltrasound_3D_Mesh/PointCloud/src/
   * TeleUltrasound_3D_Mesh/PointCloud/README.md

### Building on Windows
Before building, please ensure that all project dependencies have been installed. Building this project requires [CMAKE](https://cmake.org/) (3.5.0+) and [Visual Studio 2015+ 64-bit](https://learn.microsoft.com/en-us/visualstudio/releases/2022/release-notes).
 1. Open the **CMake-gui** application
 2. In “Where is the source code“, provide the path of the project folder where the CMakeLists.txt is located. The path should be similar to: \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud
 3. In “Where to build the binaries“, provide the previous path followed by /build. The path should be similar to: \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud/build
 4. Click on "Configure"
Example:
![cmake_configure](https://github.com/user-attachments/assets/d9f0e4a2-bf28-4a82-b592-7e9dce1f6009)

 5. A dialog window asks you if CMake can create the “build” folder. Say yes.
 6. Another dialog window will ask you to specify a generator for your project. Choose Visual Studio in Win64 and click on [Finish].
    
![cmake_prompt](https://github.com/user-attachments/assets/0a797395-c5bd-4527-bfa0-2db172d69005)
*NOTE: CMake needs to define the target platform as x64, on new version it is set by default, if not you can set it.*

![cmake_prompt_new](https://github.com/user-attachments/assets/71225ac6-8df3-44fb-81d4-d671c1e7dc35)

 7. (CMake may take a few seconds) Click on [Generate] to build the Visual Studio project files.
![cmake_generate](https://github.com/user-attachments/assets/98c5fae4-bc31-4169-84b1-115c1a966bc4)

 8. CMake has now generated your project in the build directory. You can directly open the solution by clicking on [Open Project] or by closing the cmake-gui window and opening the build folder.
![pic1](https://github.com/user-attachments/assets/acb00bd5-6817-4bdc-8d0c-70b587268c42)
9. A Visual Studio solution has been generated. Open Project.sln and set it in Release mode.
![pic2](https://github.com/user-attachments/assets/afb36a3c-2d16-45db-8618-9f42f7019940)
10. To run the builds from the Build menu or from keyboard shortcuts, set the ZED_concurrent_detections target as the startup project
(right click and choose)

![pic3](https://github.com/user-attachments/assets/6779bd9d-a51c-4db7-8e1b-6385892b87e0)
11. You can now edit and compile your program in the Visual Studio IDE. Hit the Ctrl+F5 key to launch the program.

## Build the program
 - Build for [Windows](https://www.stereolabs.com/docs/app-development/cpp/windows/)
 - Build for [Linux/Jetson](https://www.stereolabs.com/docs/app-development/cpp/linux/)


## Run the program
*NOTE: The ZED v1 is not compatible with this module*
- Navigate to the build directory and launch the executable
- Or open a terminal in the build directory and run the sample :

      ./ZED_Object_detection_multi_instance

### Features
 - The camera point cloud is displayed in a 3D OpenGL view
 - 3D bounding boxes and human skeletons around detected objects are drawn
 - Objects classes and confidences can be changed

## Support
If you need assistance go to our Community site at https://community.stereolabs.com/
