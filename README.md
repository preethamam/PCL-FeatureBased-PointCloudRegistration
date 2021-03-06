# PCL feature-based point cloud registration
A computer program on PCL framework to register two point clouds using the feature-based keypoints (SIFT, SHOT, FPFH, etc.), local/global feature descriptors, followed by various correspondence estimation and rejection methods. Below summarizes the available keypoints, descriptors, correspondence estimation and rejection methods that works in different combinations.

```
May require Point Cloud Library (PCL) > 1.11.0 
Refer to: https://github.com/PointCloudLibrary/pcl/releases
Refer to page 26, table, paper: A comprehensive review of 3D point cloud descriptors -- https://arxiv.org/abs/1802.02297

----------------------------------------------------------------------------------------------------------------------------------------------------------
Estimating Keypoints
----------------------------------------------------------------------------------------------------------------------------------------------------------
pcl::ISSKeypoint3D< PointInT, PointOutT, NormalT > **
pcl::HarrisKeypoint3D< PointInT, PointOutT, NormalT > **
pcl::HarrisKeypoint6D< PointInT, PointOutT, NormalT > **
pcl::SIFTKeypoint< PointInT, PointOutT > ***
pcl::SUSANKeypoint< PointInT, PointOutT, NormalT, IntensityT > **  Note: Carefully check if it needs RGB-D (check if depthmap or 3D point cloud) or point cloud XYZRGB
pcl::TrajkovicKeypoint3D< PointInT, PointOutT, NormalT > ***


----------------------------------------------------------------------------------------------------------------------------------------------------------
Describing keypoints - Feature descriptors
----------------------------------------------------------------------------------------------------------------------------------------------------------
Local
----------------
pcl::ShapeContext3DEstimation< PointInT, PointNT, PointOutT > -- Outperform the Spin Image estimation (SI)
pcl::PFHEstimation< PointInT, PointNT, PointOutT > -- Be invariant to position, orientation and point cloud density
pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>
pcl::FPFHEstimation< PointInT, PointNT, PointOutT > -- Reduce time consuming of PFH
pcl::FPFHEstimationOMP< PointInT, PointNT, PointOutT > -- ** Reduce time consuming of PFH
pcl::NormalEstimation< PointInT, PointOutT >
pcl::NormalEstimationOMP< PointInT, PointOutT > **
pcl::IntensityGradientEstimation< PointInT, PointNT, PointOutT, IntensitySelectorT >
pcl::PrincipalCurvaturesEstimation< PointInT, PointNT, PointOutT > **
pcl::SHOTEstimationOMP< PointInT, PointNT, PointOutT, PointRFT > -- ** Outperform Spin Image estimation
pcl::UniqueShapeContext< PointInT, PointOutT, PointRFT > -- Improve the accuracy and decrease memory cost of 3DSC

----------------------------------------------------------------
Global (may not work well for point/feature-based method)
----------------------------------------------------------------
pcl::OURCVFHEstimation< PointInT, PointNT, PointOutT > -- Outperform CVFH and SHOT
pcl::CVFHEstimation< PointInT, PointNT, PointOutT > -- Outperform SI.
pcl::GASDEstimation< PointInT, PointOutT > -- Outperform ESF, VFH and CVFH.
pcl::GASDColorEstimation< PointInT, PointOutT > -- Outperform ESF, VFH and CVFH with color info.
pcl::ESFEstimation< PointInT, PointOutT > -- Ourperform SDVS,VFH, CVFH and GSHOT
pcl::VFHEstimation< PointInT, PointNT, PointOutT > -- Outperform SI and be fast and robust to large surface noise


----------------------------------------------------------------------------------------------------------------------------------------------------------
Correspondence Estimation
----------------------------------------------------------------------------------------------------------------------------------------------------------
pcl::registration::CorrespondenceEstimation< PointSource, PointTarget, Scalar >
pcl::registration::CorrespondenceEstimationBackProjection< PointSource, PointTarget, NormalT, Scalar >
pcl::registration::CorrespondenceEstimationNormalShooting< PointSource, PointTarget, NormalT, Scalar >


----------------------------------------------------------------------------------------------------------------------------------------------------------
Correspondence rejection
----------------------------------------------------------------------------------------------------------------------------------------------------------
pcl::registration::CorrespondenceRejectorSampleConsensus< PointT > ***
pcl::registration::CorrespondenceRejectorDistance ***
pcl::registration::CorrespondenceRejectorPoly< SourceT, TargetT >
pcl::registration::CorrespondenceRejectorMedianDistance **
```
-----
## Two point clouds registration with SIFT keypoints

## Requirements
[PCL library](https://github.com/PointCloudLibrary/pcl) <br/>
C/C++ compiler (Visual Studio) <br/>
Tested on Visual Studio 2017 and 2019

## Usage
Run the C/C++ program: `two_pointcloud_registration.cpp`. Change the below variables:
|                                                      From                                                      |                                     To                                    |
|:--------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------:|
| ```string src_file = "Plate_no_change_500000_scaled.pcd"; string tgt_file = "Plate_change_500000.pcd"; ``` | ```string src_file = "1189_kinect_v2_scene_1.pcd"; string tgt_file = "1189_kinect_v2_scene_2_rottranslated.pcd";``` |
| `string src_tgt_filepath = "D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\";`                          | `string src_tgt_filepath = your\directory`                                |

Also depending on your point cloud complexity the hyperparameters (keypoints and correspondences) should be changed,

```cpp
// Hyper parameters (room)

#define LEAF_SIZE .01
#define normal_radius 0.05
#define feature_radius 0.05
#define RANSAC_Inlier_Threshold .3
#define RANSAC_Iterations 5000
#define CorrRejDist_Maximum_Distance 2

// ICP hyper parameters
#define ICP_Iterations 5000
#define ICP_TransformationEpsilon 1e-6
#define ICP_EuclideanFitnessEpsilon  1
#define ICP_RANSAC_Inlier_Threshold 0.5
#define ICP_Max_Correspondence_Distance 2

// Parameters for sift computation
#define min_scale .05
#define nr_octaves 8
#define nr_scales_per_octave 3
#define min_contrast 0.05
```

Lastly, comment/uncomment relevant lines in the `void compute_Initial_Transformation` function when various Estimations and their corresponding Find correspondences between keypoint are used.

-----
# Two point clouds registration with all possible working keypoints, local and global descriptors, correspondences estimation and rejections
## Introduction
- The original program has been prompted to a multiprocess program to run all the possible combinations of the functions provided for feature based point cloud registration. The code is in `two_pointcloud_registrationBasedOnCombinations.cpp`<br/>
- User can easily achieve the result by following the hyperparameters provided by us, or user is free to change and test by modifying them<br/>
- The evaluation script and visulization script are also been included to find the good combinations and display the result for the registration
- The flowchart for the program

![??????????????? (2)](https://user-images.githubusercontent.com/90239950/162109262-78572536-af15-48b6-8e63-63575072e5f8.png)



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
|  |--run_customized.sh (bash script which can be modified by the user to run the program)
|  |--run_console.sh (bash script to run the program with console output)
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
User can run all the combinations and save the log for the further evlaution by running the following command
```
cd your work directory
bash run.sh
```
- Run code with console output
```
bash run_console.sh
```
- Or the user can run code with their own hyper parameters
```
bash run_customized.sh
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
RANSAC_Inlier_Threshold="0.2"
RANSAC_Iterations="5000"

# ICP hyper parameters
ICP_Iterations="10000"
ICP_TransformationEpsilon="1e-6"
ICP_EuclideanFitnessEpsilon="1"
ICP_RANSAC_Inlier_Threshold="0.001"
ICP_Max_Correspondence_Distance="0.4"

```
## Evaluation
- change root_path to your result
```
python evaluation.py
```
## Visualization
- Considering the unfixed bugs for PCL library, we use Open3d to do the visualization and acheive good results
```
python registration_visualization.py
```
- Change the path according to your own directory
```
root_path = ""

src_pcloud_filename = 'normal.sift.pfh.default.default.src_pcd.pcd'
tgt_pcloud_filename = 'normal.sift.pfh.default.default.tgt_pcd.pcd'

# keypoints

src_keypoints_filename = 'normal.sift.pfh.default.default.src_keypoints_file.txt'
tgt_keypoints_filename = 'normal.sift.pfh.default.default.tgt_keypoints_file.txt'
src_good_keypoints_filename = 'normal.sift.pfh.default.default.src_good_keypoints_file.txt'
tgt_good_keypoints_filename = 'normal.sift.pfh.default.default.tgt_good_keypoints_file.txt'
initial_trans = "normal.sift.pfh.default.default.initial_transformation_matrix.txt"
final_trans ="normal.sift.pfh.default.default.final_transformation_matrix.txt"

```

## Experimental results
- Evaluation
```
Num of successful combinations:  560 / 2016
Top 5 combinations are 
Combinations:  normal.iss3d.pfhrgb.back.default  Scores:  0.002926
Combinations:  omp.iss3d.pfhrgb.back.default  Scores:  0.002926
Combinations:  omp.susan.pfh.back.default  Scores:  0.00354727
Combinations:  normal.susan.pfh.back.default  Scores:  0.00354727
Combinations:  omp.susan.pfhrgb.back.default  Scores:  0.00432473
```
- Visualization
![image](https://user-images.githubusercontent.com/90239950/162356390-09ffe5c8-efef-4100-a416-9ce1707a27a5.png)
![image](https://user-images.githubusercontent.com/90239950/162356317-9d572b85-962a-4653-9282-b54e05eca4e6.png)
![image](https://user-images.githubusercontent.com/90239950/162348719-7c574b4b-6d4e-4780-83d9-6c5ee10aa7e0.png)
![image](https://user-images.githubusercontent.com/90239950/162348530-4a3f2b2f-6dd6-4a67-9a6b-b7e01a221e38.png)



----
## Computational time and space complexity
- A desktop computer using a 64-bit Ubuntu 20.04 operating system, 128 GB memory, and an AMD Ryzen ThreadRipper 2950x processor of 3.5 GHz 16 core processor
- The total running times for the test based on room model is 2 hours 11 minutes.

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
