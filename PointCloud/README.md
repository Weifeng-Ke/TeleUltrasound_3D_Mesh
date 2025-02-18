![reversal](https://capsule-render.vercel.app/api?type=waving&text=PointCloud%20Build&fontAlign=50&fontSize=60&desc=How%20To%20Run&descAlign=900&descAlignY=50&theme=gruvbox)
# Project Build Tutorial
Below is step by step how to build the project to work on your Window device

## Getting Started
 - Get the latest [ZED SDK](https://www.stereolabs.com/developers/release/)
 - Check the [Documentation](https://www.stereolabs.com/docs/) (Documentation for this project is mainly from here)

### Setting up 
1. Download and install the latest [ZED SDK.](https://www.stereolabs.com/en-ca/developers/release)
2. Make sure there are:
   * PointCloud/CMakeLists.txt
   * PointCloud/src/
   * PointCloud/README.md
### Building on Windows
We need [CMAKE](https://cmake.org/) (3.5.0+) and [Visual Studio](https://visualstudio.microsoft.com/vs/older-downloads/) (version 2015+). The application needs to be compiled at 64-bit.

*NOTE: choose the VisualC++ option while installing Visual Studio.*
 1. Open CMake-gui application
 2. In “Where is the source code“, enter the path of the project folder where the CMakeLists.txt is located.
 3. In “Where to build the binaries“, enter the previous path and add: /build.
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
