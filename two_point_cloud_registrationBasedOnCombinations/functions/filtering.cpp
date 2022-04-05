double LEAF_SIZE =0.1;
void filtering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
					 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points_filtered)
{

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(points);
	sor.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
	sor.filter(*points_filtered);
	
}