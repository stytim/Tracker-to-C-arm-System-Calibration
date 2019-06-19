# Tracker to C-arm System Calibration

## Overview
Calibration between C-arm's X-ray source and the visual tracker that is attached to the C-arm. The calibration process has 2 steps in total.

## Step 1 
Calibration between X-ray source and the marker that is attached to the C-arm. After this step, we can get the X-ray source's pose in NDI tracker's coordinate frame.

## Step 2
Hand-Eye calibration between X-ray-source and HoloLens. 

![Transformation Chain](https://raw.githubusercontent.com/stytim/Tracker-to-C-arm-System-Calibration/master/Readme_Images/calbration.jpg)

## Requirements
* Python 3
* Matlab
* Jupyter Notebook
* OpenCV
* Numpy
* Scipy
* Matplotlib
This code was tested on Windows 10. 

## Usage
1. Run the jupyter notebook file `System_Calibration.ipynb` and follow the instructions.
2. Run the Matlab script `HandEyeCalibration.m`.

## Citation
To be updated.
