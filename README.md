# Tracker to C-arm System Calibration

## Overview
Calibration between C-arm's X-ray source and the visual tracker that is attached to the C-arm. The calibration process has 2 steps in total.

## Step 1 
Calibration between X-ray source and the marker that is attached to the C-arm. After this step, we can get the X-ray source's pose in NDI tracker's coordinate frame.

## Step 2
Hand-Eye calibration between X-ray-source and HoloLens. 

![Transformation Chain](https://raw.githubusercontent.com/stytim/Tracker-to-C-arm-System-Calibration/master/Readme_Images/calbration.jpg)

## Prerequisites
* Python 3
* Matlab
* Jupyter Notebook
* OpenCV
* Numpy
* Scipy
* Matplotlib

This code was tested on Windows 10. 

## Getting Started
1. Run the jupyter notebook file `System_Calibration.ipynb` and follow the instructions.
2. Run the Matlab script `HandEyeCalibration.m`.

## Citation
If you use this code for your research, please cite our paper <a href="https://doi.org/10.1109/TMI.2020.3037013">Development and Pre-Clinical Analysis of Spatiotemporal-Aware Augmented Reality in Orthopedic Interventions</a>:
```
@ARTICLE{9252943,
  author={Fotouhi, Javad and Mehrfard, Arian and Song, Tianyu and Johnson, Alex and Osgood, Greg and Unberath, Mathias and Armand, Mehran and Navab, Nassir},
  journal={IEEE Transactions on Medical Imaging}, 
  title={Development and Pre-Clinical Analysis of Spatiotemporal-Aware Augmented Reality in Orthopedic Interventions}, 
  year={2021},
  volume={40},
  number={2},
  pages={765-778},
  doi={10.1109/TMI.2020.3037013}}
```

<a href="https://link.springer.com/article/10.1007/s11548-019-01943-z">Interactive Flying Frustums (IFFs): spatially aware surgical data visualization</a>:

```
@article{fotouhi2019interactive,
  title={Interactive Flying Frustums (IFFs): spatially aware surgical data visualization},
  author={Fotouhi, Javad and Unberath, Mathias and Song, Tianyu and Gu, Wenhao and Johnson, Alex and Osgood, Greg and Armand, Mehran and Navab, Nassir},
  journal={International journal of computer assisted radiology and surgery},
  volume={14},
  number={6},
  pages={913--922},
  year={2019},
  publisher={Springer}
}
```
and <a href="https://link.springer.com/article/10.1007/s11548-019-02035-8">Co-localized augmented human and X-ray observers in collaborative surgical ecosystem</a>:

```
@article{fotouhi2019co,
  title={Co-localized augmented human and X-ray observers in collaborative surgical ecosystem},
  author={Fotouhi, Javad and Unberath, Mathias and Song, Tianyu and Hajek, Jonas and Lee, Sing Chun and Bier, Bastian and Maier, Andreas and Osgood, Greg and Armand, Mehran and Navab, Nassir},
  journal={International journal of computer assisted radiology and surgery},
  volume={14},
  number={9},
  pages={1553--1563},
  year={2019},
  publisher={Springer}
}
```
