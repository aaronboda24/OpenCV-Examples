# StereoCalibration

## Synopsis

Stereo Camera Calibration : Estimate transformation between two cameras making a stereo pair.

## Motivation

Testing the new OpenCV v4.0 in VisualStudio 2017. This can help users who are new to OpenCV.

## Installation

This project is particularly suited for VisualStudio users. 
Of course, you also need OpenCV (v4.0 used here).

#### Setting up OpenCV with VisualStudio
This tutorial is comprehensive: https://www.learnopencv.com/install-opencv3-on-windows/

#### Project Property Settings
Right-click on Visual Studio project and go to properties:
1) C++ -> General -> Additional Include Directories -> Add Folder -> $OpenCV Path$/build/include
2) Linker -> General -> Additional Library Directories -> Add Folder -> $OpenCV Path$/build/x64/vc15/lib
3) Linker -> Input -> Additional Dependencies -> opencv_world400d.lib

## Testing

To TEST the utility, you can check out "StereoCalibration.cpp" which provides step-by-step instructions. 
I have also included sample chessboard calibration images in the "Images" Folder.

## License

Free-to-use (MIT), but at your own risk.
