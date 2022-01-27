# PCL-FeatureBased-PointCloudRegistration
A computer program on PCL framework to register two point clouds using the feature-based keypoints (SIFT, SHOT, FPFH).

# Requirements
[PCL library](https://github.com/PointCloudLibrary/pcl) <br/>
C/C++ compiler (Visual Studio) <br/>
Tested on Visual Studio 2017 and 2019

# Usage
Run the C/C++ program: `two_pointcloud_registration.cpp`. Change the below variables:
|                                                      From                                                      |                                     To                                    |
|:--------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------:|
| ```cpp string src_file = "Plate_no_change_500000_scaled.pcd"; string tgt_file = "Plate_change_500000.pcd"; ``` | ```cpp string src_file = "bunny.pcd"; string tgt_file = "bunny2.pcd"; ``` |
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

# Note
This program was developed in 2019. Further improvements will be made in the near future.

# Known issues
1. PCL visualization do not align the registered point clouds well ([Bad graphics](https://github.com/PointCloudLibrary/pcl/issues/3261#issuecomment-518360537)). See below figures:
| View 1 | View 2 | Imagesiew 3 |
| --- | --- | --- |
| ![01](https://user-images.githubusercontent.com/28588878/151403140-1683e335-2872-44d7-9d7c-10954e88bc95.png) | ![02](https://user-images.githubusercontent.com/28588878/151403161-9a6afc89-002c-449b-af6f-b016498152fc.png) | ![03](https://user-images.githubusercontent.com/28588878/151403185-f67fa9a2-000c-4193-804f-756df9841d92.png) |



