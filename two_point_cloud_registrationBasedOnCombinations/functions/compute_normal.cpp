





double normal_radius = 0.1;
void compute_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
					 pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{

	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
	// Use a FLANN-based KdTree to perform neighbourhood searches
	norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));

	norm_est.setRadiusSearch(normal_radius);
	norm_est.setInputCloud(points);
	norm_est.compute(*normals_out);
}

void compute_normalsOMP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
	//  cout << "Computing normal by OMP " << endl;

	pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
	// Use a FLANN-based KdTree to perform neighbourhood searches
	norm_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	norm_est.setNumberOfThreads(6);

	norm_est.setRadiusSearch(normal_radius);
	norm_est.setInputCloud(points);
	norm_est.compute(*normals_out);
}