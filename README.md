# PCL-FeatureBased-PointCloudRegistration
A computer program on PCL framework to register two point clouds using the feature-based keypoints (SIFT, SHOT, FPFH).

# Usage
Run the C/C++ program: `two_pointcloud_registration.cpp`



# Change
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
