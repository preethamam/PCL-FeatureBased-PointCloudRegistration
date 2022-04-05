# PCL feature-based point cloud registration
A computer program on PCL framework to register two point clouds using the feature-based keypoints (SIFT, SHOT, FPFH).

## Two point clouds registration with SIFT keypoints

## Requirements
[PCL library](https://github.com/PointCloudLibrary/pcl) <br/>
C/C++ compiler (Visual Studio) <br/>
Tested on Visual Studio 2017 and 2019

## Usage
Run the C/C++ program: `two_pointcloud_registration.cpp`. Change the below variables:
|                                                      From                                                      |                                     To                                    |
|:--------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------:|
| ```string src_file = "Plate_no_change_500000_scaled.pcd"; string tgt_file = "Plate_change_500000.pcd"; ``` | ```string src_file = "bunny.pcd"; string tgt_file = "bunny2.pcd";``` |
| `string src_tgt_filepath = "D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\";`                          | `string src_tgt_filepath = your\directory`                                |

Also depending on your point cloud complexity the hyperparameters (keypoints and correspondences) should be changed,

```cpp
//// Hyper parameters (plate)
// RANSAC hyper parameters
#define LEAF_SIZE .1
#define normal_radius 0.1
#define feature_radius 0.1
#define RANSAC_Inlier_Threshold 0.2  //0.2
#define RANSAC_Iterations 5000
#define CorrRejDist_Maximum_Distance 0.7 //0.7

// ICP hyper parameters
#define ICP_Iterations 10000
#define ICP_TransformationEpsilon 1e-6
#define ICP_EuclideanFitnessEpsilon  1
#define ICP_RANSAC_Inlier_Threshold 0.001
#define ICP_Max_Correspondence_Distance 0.4

// Parameters for sift computation
#define min_scale .4
#define nr_octaves 4
#define nr_scales_per_octave 5
#define min_contrast 0.25
```

Lastly, comment/uncomment relevant lines in the `void compute_Initial_Transformation` function when various Estimations and their corresponding Find correspondences between keypoint are used.

# Two point clouds registration with all possible working keypoints, local and global descriptors, correspondences estimation and rejections
## Introduction
- The original program has been prompted to a multiprocess program to run all the possible combinations of the functions provided for feature based point cloud registration<br/>
- User can easily achieve the result by following the hyperparameters provided by us, or user is free to change and test by modifying them<br/>
- The evaluation script and visulization script are also been included to find the good combinations and display the result for the registration
- The flowchart for the prpgram


![flowchart](https://user-images.githubusercontent.com/90239950/161680597-b770b5eb-6597-48dc-99d2-99592cb9ad18.png)

## Requirements
Ubuntu (20.04)<br/>
[PCL library 1.12.1](https://github.com/PointCloudLibrary/pcl)<br/>
Cmake (3.16.3 or higher)<br/>
open3d 
```
pip install open3d
```

## Usage
- Prepare the directory for the program
```
|--your work directory
|  |--your dataset
|  |  |--source_pointcloud0.pcd
|  |  |--target_pointcloud0.pcd
|  |  |--source_pointcloud1.pcd
|  |  |--target_pointcloud1.pcd
|  |  |--.......
|  |--two_pointcloud_registration.cpp (main program)
|  |--ret (path to store the log and reuslts)
|  |--include.h (all the header files)
|  |--functions (implementation of all the functions)
|  |--run.sh (bash script to run the program)
|  |--evalution.py
|  |--registration_visualization.py
```
- Compile the source code
```
cd your work directory
mkdir build&&cd build
cmake ..
make -j8
```

- Run the code
User can run all the combinations by running the following command
```
cd your work directory
bash run.sh
```
User can also run single combination by running the following command
```
 ./build/project_two_pointcloud_registration --src "root path for dataset" --src_file "source pointcloud" --tgt_file "target point cloud" --normal "selection of normal computing function" --keypoint "selection of keypoints extrator" --feature "selection of feature descriptors" --correspondences "selection of finding correspondences" --reject "selection of rejecting bad correspondence" --ret "path to save the result"
```
- Changing the hyper parameters
All the parameters in run.sh are free to change according to different purpose
```
normal=("omp" "normal")
keypoint=("sift" "harries3d" "harries6d" "iss3d" "susan" "Trajkovic")
feature=("fpfh" "pfh" "pfhrgb" "shot" "3dsc" "usc" "FPFHOMP" "principal" "cvfh" "ourcvfh" "gasd" "gasdcolor" "esf" "vfh")
correspondences=("back" "normal" "default")
reject=("distance" "median" "poly" "default")

# root path for the dataset
src=""
# file path for source and target path
src_file=""
tgt_file=""
# result path to store the log and results
ret=""

# Parameters for filtering
LEAF_SIZE="0.1"
# Parameters for sift
min_scale="0.4"
MinNeighbors="5"
NumberOfThreads="4"

# susan
radius='0.1f'
radiusSearch='0.1f'

# Trajkovic
FirstThreshold='0.00046f'
SecondThreshold='0.03589'
WindowSize='3'

# harris_6d
Threshold='0.01f'

# normal 
normal_radius="0.1"

# RANSAC
RANSAC_Inlier_Threshold=0.2
RANSAC_Iterations=5000

# ICP hyper parameters
ICP_Iterations=10000
ICP_TransformationEpsilon=1e-6
ICP_EuclideanFitnessEpsilon=1
ICP_RANSAC_Inlier_Threshold=0.001
ICP_Max_Correspondence_Distance=0.4
model_resolution="0.2"
Threshold21="0.975"
Threshold32="0.975"
MinNeighbors="5"
NumberOfThreads="4"

# susan
radius='0.1f'
radiusSearch='0.1f'

# Trajkovic
FirstThreshold='0.00046f'
SecondThreshold='0.03589'
WindowSize='3'

# harris_6d
Threshold='0.01f'

# normal 
normal_radius="0.1"

# RANSAC
RANSAC_Inlier_Threshold=0.2
RANSAC_Iterations=5000

# ICP hyper parameters
ICP_Iterations=10000
ICP_TransformationEpsilon=1e-6
ICP_EuclideanFitnessEpsilon=1
ICP_RANSAC_Inlier_Threshold=0.001
ICP_Max_Correspondence_Distance=0.4

```
## Evaluation
```
python evaluation.py
```
## Visualization
Considering the unfixed bugs for PCL library, we use Open3d to do the visualization and acheive good results
```
python registration_visualization.py
```

## Experimental results
- Evalutation
```
Num of successful combinations:  560 / 2016
Top 5 combinations are 
Combinations:  normal.sift.ourcvfh.back.poly  Scores:  31.8838
Combinations:  omp.sift.gasd.default.median  Scores:  31.8838
Combinations:  normal.sift.gasd.back.median  Scores:  31.8838
Combinations:  omp.sift.cvfh.default.median  Scores:  31.8838
Combinations:  omp.sift.gasd.default.poly  Scores:  31.8838
```
- Visualization
![keypoints-Viewer](https://user-images.githubusercontent.com/90239950/161681138-2d2db002-d4ca-46af-930a-4098fd321742.png)
![Correspondence-viewer](https://user-images.githubusercontent.com/90239950/161681161-2352e709-c236-4db9-afc8-2a2074770120.png)
![image](https://github.com/preethamam/PCL-FeatureBased-PointCloudRegistration/blob/main/visualization_results/initial-registration.png)
![image](https://github.com/preethamam/PCL-FeatureBased-PointCloudRegistration/blob/main/visualization_results/ICP-registration.png)

----
## Computational time and space complexity
To be added in recent future

----
# Known issues
1. PCL visualization do not align the registered point clouds well ([Bad graphics](https://github.com/PointCloudLibrary/pcl/issues/3261#issuecomment-518360537)). See below figures:

| View 1                                                                                                       | View 2                                                                                                       | View 3                                                                                                       |
|--------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| ![01](https://user-images.githubusercontent.com/28588878/151403140-1683e335-2872-44d7-9d7c-10954e88bc95.png) | ![02](https://user-images.githubusercontent.com/28588878/151403161-9a6afc89-002c-449b-af6f-b016498152fc.png) | ![03](https://user-images.githubusercontent.com/28588878/151403185-f67fa9a2-000c-4193-804f-756df9841d92.png) |

----
# Authors
1. Dr. Preetham Manjunatha, Ph.D in Civil Engineering, M.S in Computer Science, M.S in Electrical Engineering and M.S in Civil Engineering, University of Southern California.

2. Chaoyi Zhou ([ChaoyiZh](https://github.com/ChaoyiZh)), M.S in Computer Science, University of Southern California.
