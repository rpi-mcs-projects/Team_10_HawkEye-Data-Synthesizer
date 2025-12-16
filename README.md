# mmWave Radar Training Data Synthesizer - Final Project for ECSE-6560: Modern Communication Systems
Author: Aidan Rosenblatt, Gabriela Crother-Collado



## Overview
This project synthesizes deep learning training data for a millimeter wave radar imaging in a similar manner to the "HawkEye" project from the research paper *Through Fog High Resolution Imaging Using Millimeter Wave Radar*. This synthesizer is implemented entirely in MATLAB and uses a dataset of car CAD models from the HawkEye GitHub repository.

Unlike computer-vision-based solutions, mm wave radars are able to penetrate dense fog, making them an attractive alternative solution for use in automatic driving systems.


## Software Dependencies
This project relies on only MATLAB scripts/functions. **All requirements listed below.**

### 1. MATLAB
* **Required Add-Ons:**
* DSP System Toolbox
* Computer Vision Toolbox
* Image Processing Toolbox


## Installation & Setup Guide
1. Download the previously listed add-ons from the matlab add on explorer.
2. Download the "Synthesizer" folder from this repository.
3. Add the "scripts" subfolder to your MATLAB path.

## Usage
### 1. Set up the main script
* Open "main.m".
* Enter the path to the CAD file from the "CAD" folder to be processed on line 4.

### 2. Run the synthesizer
* Run the "main.m" script. 



## Files in Repository (by folder)
### CAD Folder
* `CAD_model_x.mat`: Dataset of 36 CAD files depicting different types of cars

### Scripts Folder
* `main.m`: Main training data synthesization script which calls helper functions and outputs figures containing training data
* `occlude_points.m`: Simulated visually occluded points from radar POV
* `shininess.m`: Approximates specularity of surface points on car and outputs point cloud of reflective surface "blobs" to bounce radar beams off of
* `variable_library_radar`: Contains radar characteristics for simulating FMCW mm wave radar
* functions folder: contains radar signal and transmission simulation functions from the HawkEye GitHub repository (not implemented by our group)




## Troubleshooting



## References
* [Through Fog High Resolution Imaging Using Millimeter Wave Radar]
(https://jguan.page/HawkEye/)
