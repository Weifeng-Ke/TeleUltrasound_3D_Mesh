# Tele-Ultrasound 3D Mesh

## Purpose: 
This project is intended to be used with the ZED Mini stereo camera in order to produce Point Clouds and 3D meshes of a patient. These meshes can be used to provide the Tele-Ultrasound experts with a 3D visual and haptic feedback for improved orientation in virtual space.

**TODO: PLEASE REVAMP THIS SECTION,** I am not sure what is supposed to be here but it doesn't seem right
### Features
 - The camera point cloud is displayed in a 3D OpenGL view
 - 2D bounding boxes are used to filter out objects that's not the patient
 - Identifying Patient in real time
 - Produce Mesh file in .ply format by pressing the 's' key.
 - The mesh file is saved in the same directory as the script
	- The mesh file is named "generated_mesh{filenum}.ply"

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
4. Python
   * Download the latest version of [Python](https://www.python.org/)
   * Select "Add to PATH" while installing python

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

      ./ZED_Patient_3D

# Mesh Project Setup Instructions (Python)
Below is step by step guide on how to setup, build and run the Mesh Project on a Windows device

## Mesh Project Dependencies
1. Open3D
   * An open-source library for working with 3D data like point clouds and meshes.
   * To download this library, use the command `pip install open3d`
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

**TODO: PLEASE REVIEW AND TEST THIS SECTION**
## Mesh Project Setup
1. Make sure that the project includes the following files:
   * TeleUltrasound_3D_Mesh/PointCloud/src/Mesh.py
   * TeleUltrasound_3D_Mesh/PointCloud/src/setup.py
   * TeleUltrasound_3D_Mesh/PointCloud/src/wrapper.pyx
2. In a command line, navigate to the /src directory containing setup.py and mesh.py. The path should be similar to \<project-root\>/TeleUltrasound_3D_Mesh/PointCloud/src
3. Setup the project by running the command 'setup.py build_ext --inplace'
4. Verify that wrapper.c and a .pyd file have been generated
5. You can now run the Mesh.py program by navigating to the /src directory and using the command 'python Mesh.py'

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

## Support
For assistance, please email wke01@student.ubc.ca
