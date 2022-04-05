double RANSAC_Inlier_Threshold =  0.2; // 0.2
double RANSAC_Iterations =5000;
void rejectBadCorrespondences(const pcl::CorrespondencesPtr &all_correspondences,
							  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
							  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
							  pcl::Correspondences &remaining_correspondences)
{

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZRGB> correspondence_rejector;
	cout << "Value of RANSAC_Inlier_Threshold: " << RANSAC_Inlier_Threshold << endl;
	correspondence_rejector.setInputSource(keypoints_src);
	correspondence_rejector.setInputTarget(keypoints_tgt);
	correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
	correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
	correspondence_rejector.setRefineModel(true); // false
	correspondence_rejector.setInputCorrespondences(all_correspondences);
	correspondence_rejector.getCorrespondences(remaining_correspondences);
}

void rejectBadCorrespondences_distance(const pcl::CorrespondencesPtr &all_correspondences,
									   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
									   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
									   pcl::Correspondences &remaining_correspondences)
{

	// pcl::registration::CorrespondenceRejectorDistance <pcl::PointXYZRGB> correspondence_rejector;
	pcl::registration::CorrespondenceRejectorDistance rej;
	rej.setInputSource<pcl::PointXYZRGB>(keypoints_src);
	rej.setInputTarget<pcl::PointXYZRGB>(keypoints_tgt);
	rej.setMaximumDistance(1);
	// correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
	// correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
	// correspondence_rejector.setRefineModel(true);//false
	rej.setInputCorrespondences(all_correspondences);
	rej.getCorrespondences(remaining_correspondences);
}


void rejectBadCorrespondences_MedianDistance(const pcl::CorrespondencesPtr &all_correspondences,
											 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
											 const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
											 pcl::Correspondences &remaining_correspondences)
{

	// pcl::registration::CorrespondenceRejectorDistance <pcl::PointXYZRGB> correspondence_rejector;
	pcl::registration::CorrespondenceRejectorMedianDistance rej;
	rej.setMedianFactor(8.79241104);
	// rej.setInputSource<pcl::PointXYZRGB> (keypoints_src);
	// rej.setInputTarget<pcl::PointXYZRGB> (keypoints_tgt);
	// rej.setMaximumDistance (1);
	// correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
	// correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
	// correspondence_rejector.setRefineModel(true);//false
	rej.setInputCorrespondences(all_correspondences);
	rej.getCorrespondences(remaining_correspondences);
}

void rejectBadCorrespondences_poly(const pcl::CorrespondencesPtr &all_correspondences,
								   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_src,
								   const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_tgt,
								   pcl::Correspondences &remaining_correspondences)
{

	pcl::registration::CorrespondenceRejectorPoly<pcl::PointXYZRGB, pcl::PointXYZRGB> correspondence_rejector;
	correspondence_rejector.setInputSource(keypoints_src);
	correspondence_rejector.setInputTarget(keypoints_tgt);
	// correspondence_rejector.setInlierThreshold(RANSAC_Inlier_Threshold);
	// correspondence_rejector.setMaximumIterations(RANSAC_Iterations);
	// correspondence_rejector.setRefineModel(true);//false
	correspondence_rejector.setInputCorrespondences(all_correspondences);
	correspondence_rejector.getCorrespondences(remaining_correspondences);
}