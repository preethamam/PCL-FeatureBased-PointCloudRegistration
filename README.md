# PCL-FeatureBased-PointCloudRegistration
A computer program on PCL framework to register two point clouds using the feature-based keypoints (SIFT, SHOT, FPFH).

# Usage
Run the C/C++ program: `two_pointcloud_registration.cpp`



# Change


| From     |  To |
| -------- | ----------- |
|```cpp
string src_file = "Plate_no_change_500000_scaled.pcd";
string tgt_file = "Plate_change_500000.pcd";
```
|
```cpp
string src_file = "bunny.pcd";
string tgt_file = "bunny2.pcd";
```
|

bunny.pcd
string src_tgt_filepath = "D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\";

|                                                            From                                                           |                                                             To                                                            |
|:-------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------:|
| ```cpp int main() {   int y = SOME_MACRO_REFERENCE;   int x = 5 + 6;   cout << "Hello World! " << x << std::endl(); } ``` | ```cpp int main() {   int y = SOME_MACRO_REFERENCE;   int x = 5 + 6;   cout << "Hello World! " << x << std::endl(); } ``` |
| `string src_tgt_filepath = "D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\";`                                     | `string src_tgt_filepath = your\directory`                                                                                |

