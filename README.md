# Eel Tracking (OpenCV, C++)

## Description
This project implements automatic eel tracking in sonar underwater videos using OpenCV. Each region potentially corresponding to an eel is assigned a unique ID and highlighted with a blue contour. When the region is confirmed as an eel, a green bounding box appears around it, and its trajectory is recorded in a CSV file. The algorithm combines background subtraction (MOG2), contour filtering, and centroid-based tracking to detect and track eels over time.

## Quick Start
Clone the repository:
```bash
git clone https://github.com/nellbru/eel-tracking-opencv.git
cd eel-tracking-opencv
```
Run the executable and provide the video path:
```bash
./eel_tracking <video-file>
```

## Results
The visualization shows the input video where each potential eel is outlined in blue and assigned an ID. When a region is confirmed as an eel, a green bounding box appears around it with the label “Anguille” and its trajectory is recorded in a CSV file. Console updates also display the frame processing progress. The algorithm is robust to slow movements and partial occlusions, and each eel is confirmed only after several consecutive frames to reduce false detections.

<p align="center">
  <img src="docs/Result_Vid1.gif" alt="Result" width="100%">
</p>

<p align="center">
  <img src="docs/Result_Vid2.gif" alt="Result" width="100%">
</p>

The algorithm is robust to noise and slow movements. Each eel is confirmed after several consecutive frames to reduce false detections.

## Project Structure
    eel-tracking-opencv/
    ├─ Videos/                 # Input videos
    ├─ Results/                # Results examples
    ├─ docs/                   # Documentation assets
    ├─ eel_tracking.cpp        # Tracking algorithm
    ├─ eel_tracking.exe        # Precompiled executable (Windows)
    ├─ CMakeLists.txt          # Visual Studio project configuration
    ├─ opencv_world4120.dll    # OpenCV library dll
    ├─ .gitignore              # Git ignore rules
    ├─ LICENSE                 # Project license
    └─ README.md               # Project documentation

## Requirements
- Windows 10/11 (64-bit)

## License
This project is licensed under the terms of the [LICENSE](LICENSE) file.