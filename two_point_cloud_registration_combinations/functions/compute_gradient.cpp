void compute_IntensityGradient(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
							   pcl::PointCloud<pcl::Normal>::Ptr &normals,
							   pcl::PointCloud<pcl::IntensityGradient>::Ptr &gradient)
{

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*points, *cloud_in);

	pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient> gradient_est;
	// Use a FLANN-based KdTree to perform neighbourhood searches
	gradient_est.setInputCloud(cloud_in);
	gradient_est.setInputNormals(normals);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr treept2(new pcl::search::KdTree<pcl::PointXYZI>(false));
	gradient_est.setSearchMethod(treept2);
	gradient_est.setRadiusSearch(0.25);
	gradient_est.compute(*gradient);
}