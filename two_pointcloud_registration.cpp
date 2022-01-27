#include <fstream>
#include <iostream>
#include <cstring>
#include <string>
#include <ctime>
#include <vector>
#include <sstream>

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

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/pfh.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>

#include <pcl/correspondence.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/shot_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/default_convergence_criteria.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_estimation_svd.h>


using namespace std;
// using namespace pcl;
// using namespace pcl::io;
// using namespace pcl::console;
// using namespace pcl::registration;
// using namespace pcl::visualization;

// Node:VThis program is written for RGB point clouds for source and target.
// Change for any other point types (like PointXYZ, PointXYZI, PointXYZRGBA, etx.)


///////////////////////////////////////////////////////////////////////////////////////////////////////
// Inputs
///////////////////////////////////////////////////////////////////////////////////////////////////////
string src_tgt_filepath = "D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\";



///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (room)
/*
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

string src_file = "1189_kinect_v2_scene_1.pcd";
string tgt_file = "1189_kinect_v2_scene_2_rottranslated.pcd"; //1189_kinect_v2_scene_2_rottranslated

//*/
///////////////////////////////////////////////////////////////////////////////////////////////////////
//// Hyper parameters (plate)
//*
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

string src_file = "Plate_no_change_500000_scaled.pcd";
string tgt_file = "Plate_change_500000.pcd";

//*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (civic engine)
//#define LEAF_SIZE .01
//#define normal_radius 0.05
//#define feature_radius 0.05
//#define RANSAC_Inlier_Threshold 0.2
//#define RANSAC_Iterations 5000
//#define CorrRejDist_Maximum_Distance 5
//
//// ICP hyper parameters
//#define ICP_Iterations 1000
//#define ICP_TransformationEpsilon 1e-6
//#define ICP_EuclideanFitnessEpsilon  1
//#define ICP_RANSAC_Inlier_Threshold 0.7
//#define ICP_Max_Correspondence_Distance 1.25
//
//// Parameters for sift computation
//#define min_scale .05
//#define nr_octaves 4
//#define nr_scales_per_octave 5
//#define min_contrast 0.25

//string src_file = "Civic_nochange_5Million_trimmed.pcd"
//string tgt_file = "Civic_change_1Million_scaled_trimmed.pcd"

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Hyper parameters (RAV4 engine)
//#define LEAF_SIZE .01
//#define normal_radius 0.05
//#define feature_radius 0.05
//#define RANSAC_Inlier_Threshold 0.2
//#define RANSAC_Iterations 5000
//#define CorrRejDist_Maximum_Distance 5
//
//// ICP hyper parameters
//#define ICP_Iterations 1000
//#define ICP_TransformationEpsilon 1e-6
//#define ICP_EuclideanFitnessEpsilon  1
//#define ICP_RANSAC_Inlier_Threshold 0.7
//#define ICP_Max_Correspondence_Distance 1.25
//
//// Parameters for sift computation
//#define min_scale .05
//#define nr_octaves 4
//#define nr_scales_per_octave 5
//#define min_contrast 0.25

//string src_file = "RAV4_Engine_No_Change_trimmed.pcd"
//string tgt_file = "RAV4_Engine_Change_scaled_trimmed.pcd"

void detect_keypoints(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints_out) {

	pcl::SIFTKeypoint <pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

	// Use a FLANN-based KdTree to perform neighbourhood searches
	pcl::search::KdTree <pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree <pcl::PointXYZRGB>);
	sift_detect.setSearchMethod(tree);

	// Set the detection parameters
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);

	// Set the input
	sift_detect.setInputCloud(points);

	// Detect the keypoints and store them in "keypoints.out"
	sift_detect.compute(*keypoints_out);
}

void compute_normals(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
	 pcl::PointCloud <pcl::Normal>::Ptr &normals_out) {

	pcl::NormalEstimation <pcl::PointXYZRGB, pcl::Normal> norm_est;
	// Use a FLANN-based KdTree to perform neighbourhood searches
	norm_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree <pcl::PointXYZRGB>));

	norm_est.setRadiusSearch(normal_radius);
	norm_est.setInputCloud(points);
	norm_est.compute(*normals_out);
}

void compute_FPFH_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &points,
	pcl::PointCloud <pcl::Normal>::Ptr &normals,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
	
	pcl::PointCloud <pcl::FPFHSignature33>::Ptr &descriptors_out) {

	pcl::FPFHEstimation <pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree <pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree <pcl::PointXYZRGB>));
	pfh_est.setRadiusSearch(feature_radius);

	// copy only XYZ data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure 
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints_xyzrgb);
	pfh_est.compute(*descriptors_out);

}

void compute_PFH_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
	pcl::PointCloud <pcl::Normal>::Ptr &normals,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
	pcl::PointCloud <pcl::PFHSignature125>::Ptr &descriptors_out) {

	
	// copy only XYZ data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(keypoints_xyzrgb);
	pfh.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
	pfh.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pfh.setSearchMethod(tree);

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch(feature_radius);

	// Compute the features
	pfh.compute(*descriptors_out);

}

void compute_PFHRGB_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
	pcl::PointCloud <pcl::Normal>::Ptr &normals,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
	pcl::PointCloud <pcl::PFHRGBSignature250>::Ptr &descriptors_out) {


	// copy only XYZ data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgbEstimation;


	pfhrgbEstimation.setInputCloud(keypoints_xyzrgb);
	pfhrgbEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
	pfhrgbEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pfhrgbEstimation.setSearchMethod(tree);
	//pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfhrgbEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	pfhrgbEstimation.compute(*descriptors_out);

}



void compute_SHOTRGB_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
	pcl::PointCloud <pcl::Normal>::Ptr &normals,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
	pcl::PointCloud <pcl::SHOT1344>::Ptr &descriptors_out) {


	// copy only XYZ data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::SHOTColorEstimationOMP <pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shotrgbEstimation;


	shotrgbEstimation.setInputCloud(keypoints_xyzrgb);
	shotrgbEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
	shotrgbEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	shotrgbEstimation.setSearchMethod(tree);
	//pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	shotrgbEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	shotrgbEstimation.compute(*descriptors_out);

}


void compute_SHOT_features(pcl::PointCloud <pcl::PointXYZRGB>::Ptr &cloud,
	pcl::PointCloud <pcl::Normal>::Ptr &normals,
	pcl::PointCloud <pcl::PointWithScale>::Ptr &keypoints,
	pcl::PointCloud <pcl::SHOT352>::Ptr &descriptors_out) {


	// copy only XYZ data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::SHOTEstimationOMP <pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shotEstimation;


	shotEstimation.setInputCloud(keypoints_xyzrgb);
	shotEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure 
	shotEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	shotEstimation.setSearchMethod(tree);
	//pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	shotEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	shotEstimation.compute(*descriptors_out);

}


void findCorrespondences_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
						 const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
							   pcl::Correspondences &all_correspondences) {

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}


void findCorrespondences_PFH(const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences) {

	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}


void findCorrespondences_PFHRGB(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences) {

	pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}


void findCorrespondences_SHOTRGB(const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences) {

	pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_SHOT(const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences) {

	pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}




void rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
							  const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_src,
							  const pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_tgt,
									pcl::Correspondences &remaining_correspondences)
{
	// Thresholding the distances bad correspondence rejector
	//pcl::registration::CorrespondenceRejectorDistance rej;
	//rej.setInputSource<pcl::PointXYZ>(keypoints_src);
	//rej.setInputTarget<pcl::PointXYZ>(keypoints_tgt);
	//rej.setMaximumDistance(CorrRejDist_Maximum_Distance);    // hyperparameter
	//rej.setInputCorrespondences(all_correspondences);
	//rej.getCorrespondences(remaining_correspondences);

	// copy only XYZRGB data of keypoints for use in estimating features
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_src_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_tgt_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	pcl::copyPointCloud(*keypoints_src, *keypoints_src_xyzrgb);
	pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_xyzrgb);


	// RandomSampleConsensus bad correspondence rejector
	pcl::registration::CorrespondenceRejectorSampleConsensus <pcl::PointXYZRGB> correspondence_rejector;
	correspondence_rejector.setInputSource (keypoints_src_xyzrgb);
	correspondence_rejector.setInputTarget (keypoints_tgt_xyzrgb);
	correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
	correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
	correspondence_rejector.setRefineModel(true);//false
	correspondence_rejector.setInputCorrespondences(all_correspondences);
	correspondence_rejector.getCorrespondences(remaining_correspondences);
}



void compute_Initial_Transformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src,
									pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tgt,
									Eigen::Matrix4f &transform, 
									pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_src_visualize_temp, 
									pcl::PointCloud<pcl::PointXYZ>::Ptr & keypoints_tgt_visualize_temp, 
									pcl::Correspondences & good_correspondences) {


	// ESTIMATING KEY POINTS
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointWithScale>);
	detect_keypoints(src,  keypoints_src);
	cout << "No of SIFT points in the src are " << keypoints_src->points.size() << endl;

	detect_keypoints(tgt, keypoints_tgt);
	cout << "No of SIFT points in the tgt are " << keypoints_tgt->points.size() << endl;

	// ESTIMATING PFH FEATURE DESCRIPTORS AT KEYPOINTS AFTER COMPUTING NORMALS
	pcl::PointCloud <pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud <pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);

	compute_normals(src,src_normals);
	compute_normals(tgt, tgt_normals);

	// FPFH Estimation
	pcl::PointCloud <pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);;
	pcl::PointCloud <pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);;
	//const float feature_radius = 0.2;		// adjust this hyperparameter
	compute_FPFH_features(src, src_normals, keypoints_src,  fpfhs_src);
	compute_FPFH_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

	// PFH Estimation
	//pcl::PointCloud <pcl::PFHSignature125>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PFHSignature125>);;
	//pcl::PointCloud <pcl::PFHSignature125>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>);;
	//const float feature_radius = 0.2;		// adjust this hyperparameter
	//compute_PFH_features(src, src_normals, keypoints_src,  fpfhs_src);
	//compute_PFH_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

	// PFHRGB Estimation
	//pcl::PointCloud <pcl::PFHRGBSignature250>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PFHRGBSignature250>);;
	//pcl::PointCloud <pcl::PFHRGBSignature250>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PFHRGBSignature250>);;
	//compute_PFHRGB_features(src, src_normals, keypoints_src,  fpfhs_src);
	//compute_PFHRGB_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

	// SHOTRGB Estimation
	//pcl::PointCloud <pcl::SHOT1344>::Ptr fpfhs_src(new pcl::PointCloud<pcl::SHOT1344>);
	//pcl::PointCloud <pcl::SHOT1344>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::SHOT1344>);
	//const float feature_radius = 0.2;		// adjust this hyperparameter
	//compute_SHOTRGB_features(src, src_normals, keypoints_src,  fpfhs_src);
	//compute_SHOTRGB_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

	// SHOT Estimation
	//pcl::PointCloud <pcl::SHOT352>::Ptr fpfhs_src(new pcl::PointCloud<pcl::SHOT352>);
	//pcl::PointCloud <pcl::SHOT352>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::SHOT352>);
	//const float feature_radius = 0.2;		// adjust this hyperparameter
	//compute_SHOT_features(src, src_normals, keypoints_src,  fpfhs_src);
	//compute_SHOT_features(tgt, tgt_normals, keypoints_tgt,  fpfhs_tgt);

	// For debugging purposes only: uncomment the lines below and use pcl_viewer to view the results, i.e.:
	/*pcl_viewer fpfhs_src.pcd
	pcl::PCLPointCloud2 s, t, out;
	pcl::toPCLPointCloud2(*keypoints_src, s);
	pcl::toPCLPointCloud2(*fpfhs_src, t);
	pcl::concatenateFields(s, t, out);
	pcl::io::savePCDFile("fpfhs_src.pcd", out);

	pcl::toPCLPointCloud2(*keypoints_tgt, s);
	pcl::toPCLPointCloud2(*fpfhs_tgt, t);
	pcl::concatenateFields(s, t, out);
	pcl::io::savePCDFile("fpfhs_tgt.pcd", out);*/

	cout << "End of compute_FPFH_features! " << endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::copyPointCloud(*keypoints_src, *keypoints_src_visualize_temp);
	pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_visualize_temp);

	cout << "SIFT points in the keypoints_src_visualize_temp are " << keypoints_src_visualize_temp->points.size() << endl;
	cout << "SIFT points in the keypoints_tgt_visualize_temp are " << keypoints_tgt_visualize_temp->points.size() << endl;


	// Find correspondences between keypoints in FPFH space
	pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
	findCorrespondences_FPFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
	//findCorrespondences_PFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
	//findCorrespondences_PFHRGB(fpfhs_src, fpfhs_tgt, *all_correspondences);
	//findCorrespondences_SHOTRGB(fpfhs_src, fpfhs_tgt, *all_correspondences);
	//findCorrespondences_SHOT(fpfhs_src, fpfhs_tgt, *all_correspondences);

	cout << "End of findCorrespondences! " << endl;
	cout << "All correspondences size: " << all_correspondences->size() << endl;

	// Reject correspondences based on their XYZ distance
	/*rejectBadCorrespondences(all_correspondences, keypoints_src_visualize_temp, keypoints_tgt_visualize_temp, good_correspondences);*/
	rejectBadCorrespondences(all_correspondences, keypoints_src, keypoints_tgt, good_correspondences);
	cout << "End of rejectBadCorrespondences! " << endl;
	cout << "Good correspondences size: " << good_correspondences.size() << endl;

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
	trans_est.estimateRigidTransformation(*keypoints_src_visualize_temp, *keypoints_tgt_visualize_temp, good_correspondences, transform);
}


void ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tgt,
	     pcl::PointCloud<pcl::PointXYZRGB> &Final, Eigen::Matrix4f &transformation) {

	//pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp; 
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(src);
	icp.setInputTarget(tgt);

	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance(ICP_Max_Correspondence_Distance);

	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations(ICP_Iterations);

	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon(ICP_TransformationEpsilon);

	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon(ICP_EuclideanFitnessEpsilon);

	icp.setRANSACOutlierRejectionThreshold(ICP_RANSAC_Inlier_Threshold);

	// Perform the alignment
	icp.align(Final);

	// Print scores
	cout << "has converged: " << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << endl;

	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	transformation = icp.getFinalTransformation();
	cout << "FInal ICP Transformation Matrix" << endl;
	std::cout << transformation << std::endl;
}


int main(int argc, char** argv)  {		

	// Time start (main function)
	time_t start_computation, end_computation, start_total, end_total;
	time(&start_total);
	time(&start_computation);

	// READ SOURCE AND TARGET FILES
	string src_fullpath = src_tgt_filepath + src_file;
	string tgt_fullpath = src_tgt_filepath + tgt_file;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_original(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_original(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(src_fullpath, *src_original) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_fullpath, *tgt_original) == -1)
	{
	  PCL_ERROR("Couldn't read src or tgt file");
	  return -1; 
	}

	cout << "Src points: " << src_original->points.size() << endl;
	cout << "Tgt points: " << tgt_original->points.size() << endl;

	pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\src_pcd.pcd", *src_original);
	pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\tgt_pcd.pcd", *tgt_original);

	///*
	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(src_original);
	sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter(*src_decimated);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
	sor.setInputCloud(tgt_original);
	sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter(*tgt_decimated);

	cerr << "Src PointCloud after decimation: " << src_decimated->width * src_decimated->height
		<< " data points (" << pcl::getFieldsList(*src_decimated) << ")." << endl;

	cerr << "Tgt PointCloud after decimation: " << tgt_decimated->width * tgt_decimated->height
		<< " data points (" << pcl::getFieldsList(*tgt_decimated) << ")." << endl;

	// Filtered point cloud copy
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	src = src_decimated;
	tgt = tgt_decimated;

	// pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\src_pcd.pcd", *src);
	// pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\tgt_pcd.pcd", *tgt);

	//pcl::visualization::CloudViewer keypoints_viewer("Simple Cloud Viewer");
	//keypoints_viewer.showCloud(src, "Source");
	//keypoints_viewer.showCloud(tgt, "Target");
	//while (!keypoints_viewer.wasStopped())
	//{
	//}

	// Compute the best transformtion
	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
	
	// Find correspondences between keypoints in FPFH space
	pcl::CorrespondencesPtr good_correspondences(new pcl::Correspondences);

	// Obtain the initial transformation matirx by using the key-points
	Eigen::Matrix4f transform;
	compute_Initial_Transformation(src, tgt, transform, keypoints_src_visualize_temp, keypoints_tgt_visualize_temp, *good_correspondences);
	cout << "Initial Transformation Matrix" << endl;
	std::cout << transform << std::endl;

	// Write the source and target keypoints 
	ofstream src_keypoints_file;
	ofstream tgt_keypoints_file;
	src_keypoints_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\src_keypoints_file.txt");
	tgt_keypoints_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\tgt_keypoints_file.txt");

	for (int i = 0; i < keypoints_src_visualize_temp->points.size(); ++i)
		src_keypoints_file << keypoints_src_visualize_temp->points[i] << endl;

	for (int i = 0; i < keypoints_tgt_visualize_temp->points.size(); ++i)
		tgt_keypoints_file << keypoints_tgt_visualize_temp->points[i] << endl;

	src_keypoints_file.close();
	tgt_keypoints_file.close();

	// Write good correspondences of source and target point clouds
	ofstream src_good_keypoints_file;
	ofstream tgt_good_keypoints_file;
	src_good_keypoints_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\src_good_keypoints_file.txt");
	tgt_good_keypoints_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\tgt_good_keypoints_file.txt");

	for (int i = 0; i < good_correspondences->size(); ++i)
	{
		pcl::PointXYZ & src_idx = keypoints_src_visualize_temp->points[(*good_correspondences)[i].index_query];
		pcl::PointXYZ & tgt_idx = keypoints_tgt_visualize_temp->points[(*good_correspondences)[i].index_match];
		src_good_keypoints_file << src_idx << endl;
		tgt_good_keypoints_file << tgt_idx << endl;
	}

	src_good_keypoints_file.close();
	tgt_good_keypoints_file.close();

	

	// Time end (main computation)
	time(&end_computation);
	double time_elapsed_computation = difftime(end_computation, start_computation);
	cout << "Elasped computation time in seconds: " << time_elapsed_computation << endl;

	// ************************************* Visualization ************************************* 
	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer keypoints_viewer("Key-points Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_src_color_handler(keypoints_src_visualize_temp, 255, 0, 0); // Red - Source key points
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_tgt_color_handler(keypoints_tgt_visualize_temp, 0, 255, 0); // Green - Target key points

	keypoints_viewer.setBackgroundColor(0.0, 0.0, 0.0);
	keypoints_viewer.addPointCloud(src, "Src cloud");
	keypoints_viewer.addPointCloud(tgt, "Tgt cloud");
	keypoints_viewer.addPointCloud(keypoints_src_visualize_temp, keypoints_src_color_handler, "keypoints src");
	keypoints_viewer.addPointCloud(keypoints_tgt_visualize_temp, keypoints_tgt_color_handler, "keypoints tgt");
	keypoints_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints src");
	keypoints_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints tgt");

	// Visualization of keypoints along with the original cloud and correspondences
	pcl::visualization::PCLVisualizer corresp_viewer("Correspondences Viewer");
	corresp_viewer.setBackgroundColor(0, 0, 0);
	corresp_viewer.addPointCloud(src, "Src cloud");
	corresp_viewer.addPointCloud(tgt, "Tgt cloud");
	corresp_viewer.addPointCloud<pcl::PointXYZ>(keypoints_src_visualize_temp, keypoints_src_color_handler, "keypoints_src_corresp_viewer");
	corresp_viewer.addPointCloud<pcl::PointXYZ>(keypoints_tgt_visualize_temp, keypoints_tgt_color_handler, "keypoints_tgt_corresp_viewer");
	corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints_src_corresp_viewer");
	corresp_viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints_tgt_corresp_viewer");


	for (int i = 0; i < good_correspondences->size(); ++i)
	{
		pcl::PointXYZ & src_idx = keypoints_src_visualize_temp->points[(*good_correspondences)[i].index_query];
		pcl::PointXYZ & tgt_idx = keypoints_tgt_visualize_temp->points[(*good_correspondences)[i].index_match];
		string lineID = to_string(i);
		string lineID2 = to_string(i+200);

			// Generate a random (bright) color
			double r = (rand() % 100);
			double g = (rand() % 100);
			double b = (rand() % 100);
			double max_channel = max(r, max(g, b));
			r /= max_channel;
			g /= max_channel;
			b /= max_channel;

		corresp_viewer.addLine<pcl::PointXYZ, pcl::PointXYZ>(src_idx, tgt_idx, r, g, b, lineID);
	}
	

	// Display the initial alignment
	// Transform the data and write it to disk
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_initial_output (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*src, *src_initial_output, transform);

	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer initial_alignment_viewer("Initial Alignment");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_initial_color_handler(src_initial_output, 0, 0, 255); // Blue -> Initial alignment
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_color_handler(tgt, 255, 0, 0); // Red -> Target
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_color_handler(src, 0, 255, 0); // Green -> Source

	initial_alignment_viewer.setBackgroundColor(0.0, 0.0, 0.0);
	initial_alignment_viewer.addPointCloud<pcl::PointXYZRGB>(src_initial_output, src_initial_color_handler, "Intial Aligned Src cloud");
	initial_alignment_viewer.addPointCloud<pcl::PointXYZRGB>(src, src_color_handler, "Intial Src cloud");
	initial_alignment_viewer.addPointCloud(tgt, tgt_color_handler, "Initial Tgt cloud");
	
	// Display final ICP registration
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f ICP_transformation;

	ICP(src_initial_output, tgt, *Final, ICP_transformation);
	pcl::transformPointCloud(*src_initial_output, *final_output, ICP_transformation);

	pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\initial.pcd", *src_initial_output);
	pcl::io::savePCDFileASCII("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\final.pcd", *final_output);

	// Write trasformation matrices to a text file
	ofstream initial_transformation_matrix_file;
	initial_transformation_matrix_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\initial_transformation_matrix.txt");
	initial_transformation_matrix_file << transform << '\n';
	initial_transformation_matrix_file.close();

	ofstream final_transformation_matrix_file;
	final_transformation_matrix_file.open("D:\\OneDrive\\Team Work\\Team PCloud\\3D models\\PCL\\final_transformation_matrix.txt");
	final_transformation_matrix_file << ICP_transformation << '\n';
	final_transformation_matrix_file.close();

	// Time end (main function)
	time(&end_total);
	double time_elapsed_total = difftime(end_total, start_total);
	cout << "Elasped total main function time in seconds: " << time_elapsed_total << endl;

	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer icp_alignment_viewer("ICP Alignment");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_final_color_handler(Final, 0, 0, 255); // Blue -> Initial alignment

	icp_alignment_viewer.setBackgroundColor(0.0, 0.0, 0.0);
	icp_alignment_viewer.addPointCloud<pcl::PointXYZRGB>(final_output, src_initial_color_handler, "ICP Aligned Src cloud");
	icp_alignment_viewer.addPointCloud<pcl::PointXYZRGB>(src, src_color_handler, "ICP Src cloud");
	icp_alignment_viewer.addPointCloud(tgt, tgt_color_handler, "ICP Tgt cloud");

	while (!keypoints_viewer.wasStopped() && !corresp_viewer.wasStopped() && !initial_alignment_viewer.wasStopped() && !icp_alignment_viewer.wasStopped()) {
		keypoints_viewer.spinOnce();
		corresp_viewer.spinOnce();
		initial_alignment_viewer.spinOnce();
		icp_alignment_viewer.spinOnce();
	}

	keypoints_viewer.close();
	corresp_viewer.close();
	initial_alignment_viewer.close();
	icp_alignment_viewer.close();
}

