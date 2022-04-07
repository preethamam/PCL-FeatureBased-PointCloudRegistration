#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <ctime>
#include <vector>
#include <sstream>
#include <algorithm>

#include <pcl/common/angles.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/harris_6d.h>
#include <pcl/keypoints/trajkovic_3d.h>
#include <pcl/keypoints/susan.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/moment_invariants.h>
#include <pcl/features/rsd.h>
#include <pcl/features/usc.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/features/gasd.h>
#include <pcl/features/esf.h>
#include <pcl/features/vfh.h>
#include <pcl/features/rift.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/features/crh.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>


void detect_keypoints_sift(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out);

#include "functions/compute_normal.cpp"
#include "functions/comnpute_gradient.cpp"
#include "functions/extract_keypoints.cpp"
#include "functions/describe_feature.cpp"
#include "functions/compute_correspondence_default.cpp"
#include "functions/compute_correspondence_backprojection.cpp"
#include "functions/compute_correspondence_normalshooting.cpp"
#include "functions/reject_badcorrespondence.cpp"
#include "functions/icp.cpp"
#include "functions/hash.cpp"
#include "functions/filtering.cpp"

