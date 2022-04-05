// ICP hyper parameters
double ICP_Iterations = 10000;
double ICP_TransformationEpsilon = 1e-6;
double ICP_EuclideanFitnessEpsilon = 1;
double ICP_RANSAC_Inlier_Threshold = 0.001;
double ICP_Max_Correspondence_Distance = 0.4;

void ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &tgt,
		 pcl::PointCloud<pcl::PointXYZRGB> &Final, Eigen::Matrix4f &transformation)
{

	// pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	// cout << "Value of ICP_Iterations: " << ICP_Iterations << endl;
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
	cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;

	// Obtain the transformation that aligned cloud_source to cloud_source_registered
	transformation = icp.getFinalTransformation();
	cout << "FInal ICP Transformation Matrix" << endl;
	std::cout << transformation << std::endl;
}