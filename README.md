## Git RoboCIn - Very Small Size League Vision Software
---

#### Content of this repository:

> Implementation of Very Small Size League Vision Software

## Installing the Dependencies

1. Install the needed dependencies beyond Qt Creator.
```bash
./InstallDependencies
```

2. 1. Install Qt Creator (5.12.0) using the maintanance tool of Qt from linorg.usp.br repository. Use this script to make this simpler.
```bash
./InstallQt
```

2. 2.  Select (only) the packages below:
```bash
. Qt
│
.
.
.
└── Qt 5.12.0
    ├── Desktop gcc 64-bit
    ├── Qt Charts
    └── Qt Data Visualization
```

## Repository Tree

```bash
.
├── include
│   └── spdlog
└── src
    ├── cameraconfigurationdialog.cpp
    ├── CameraManager
    │   ├── CameraManager.cpp
    │   └── CameraThread.cpp
    ├── Config
    │   └── Segmentation
    ├── Entity
    │   └── Entity.cpp
    ├── fieldpointscalibratedialog.cpp
    ├── iconTools
    ├── Images
    ├── Log
    │   ├── general
    │   ├── others
    │   └── vision
    ├── Logging
    │   └── logging.cpp
    ├── maggicsegmentationdialog.cpp
    ├── main.cpp
    ├── mainwindow.cpp
    ├── qrc_icontools.cpp
    ├── qrc_image.cpp
    ├── segmentationconfigdialog.cpp
    ├── TBBThreadManager.cpp
    ├── Timer
    │   └── Timer.cpp
    ├── trackconfigdialog.cpp
    ├── Utils
    │   ├── Constants.cpp
    │   └── Utils.cpp
    ├── Vision
    │   ├── ColorSpace.cpp
    │   ├── ImageProcessing
    │   │   ├── Cuda
    │   │   │   └── LUT
    │   │   ├── ImageProcessing.cpp
    │   │   ├── LUTSegmentation.cpp
    │   │   ├── MaggicSegmentation.cpp
    │   │   ├── OpenCV
    │   │   │   └── connectedcomponents.cpp
    │   │   └── WarpCorrection.cpp
    │   ├── PositionProcessing
    │   │   ├── BlobDetection.cpp
    │   │   ├── PositionProcessing.cpp
    │   │   ├── runlengthencoding.cpp
    │   │   └── WhereAreThose.cpp
    │   └── Vision.cpp
    ├── visionconfigdialog.cpp
    └── visionthread.cpp
```
- - - 

* Vision
	* Does all image processing
		* Segmentation
		* Tracking
		* Blob detection

---
* Vision (Opencv stuff, segmentation,tracking,blob detection algorithms)
	* Src->Vision

