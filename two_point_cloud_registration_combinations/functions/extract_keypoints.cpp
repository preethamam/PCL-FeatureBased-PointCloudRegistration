// Parameters for keypoints computation
// #define min_scale .4
// #define nr_octaves 4
// #define nr_scales_per_octave 5
// #define min_contrast 0.25


// sift
double min_scale = 0.4;
double nr_octaves = 4;
double nr_scales_per_octave = 5;
double min_contrast = 0.25;
// iss3d
double model_resolution = 0.2;
double Threshold21 = 0.975;
double Threshold32 = 0.975;
double MinNeighbors = 5;
double NumberOfThreads = 4;
// susan
double radius = 0.1f;
double radiusSearch = 0.1f;
// trajkovic
double FirstThreshold=0.00046;
double SecondThreshold=0.03589;
double WindowSize=3;

// harris

double Threshold = 0.01f;


// keypoints
void detect_keypoints_sift(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{

	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

	// Use a FLANN-based KdTree to perform neighbourhood searches
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	sift_detect.setSearchMethod(tree);

	// Set the detection parameters
	sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast(min_contrast);
	// cout << "Value of min_scale: " << min_scale << endl;
	// cout << "Value of nr_octaves: " << nr_octaves << endl;
	// cout << "Value of nr_scales_per_octave: " << nr_scales_per_octave << endl;
	// cout << "Value of min_contrast: " << min_contrast << endl;

	// Set the input
	sift_detect.setInputCloud(points);

	// Detect the keypoints and store them in "keypoints.out"
	pcl::PointCloud<pcl::PointWithScale> result;
	sift_detect.compute(result);
	pcl::copyPointCloud(result, *keypoints_out);
}

void detect_keypoints_iss3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	// cout << "Value of model_resolution: " << model_resolution << endl;


	pcl::ISSKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZRGB> iss_detector;
	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(6 * model_resolution);
	iss_detector.setNonMaxRadius(4 * model_resolution);
	iss_detector.setThreshold21(Threshold21);
	iss_detector.setThreshold32(Threshold32);
	iss_detector.setMinNeighbors(MinNeighbors);
	iss_detector.setNumberOfThreads(NumberOfThreads);
	iss_detector.setInputCloud(points);
	iss_detector.compute(*keypoints_out);
}

void detect_keypoints_susan(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
							pcl::PointCloud<pcl::Normal>::Ptr &normals,
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	// cout << "No of points after convertion in the src are " << cloud_in->points.size() << endl;
	pcl::SUSANKeypoint<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::Normal> susan3D;
	// cout << "Value of radius: " << radius << endl;
	susan3D.setSearchMethod(tree);
	susan3D.setInputCloud(points);
	susan3D.setNonMaxSupression(true);
	susan3D.setRadius(radius);
	susan3D.setRadiusSearch(radiusSearch);
	susan3D.setNormals(normals);
	susan3D.compute(*keypoints_out);
}

void detect_keypoints_Trajkovic(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
								pcl::PointCloud<pcl::Normal>::Ptr &normals,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::copyPointCloud(*points, *cloud_in);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
	// cout << "No of points after convertion in the src are " << cloud_in->points.size() << endl;
	pcl::TrajkovicKeypoint3D<pcl::PointXYZI, pcl::PointXYZI, pcl::Normal> trajkovic;
	trajkovic.setFirstThreshold(FirstThreshold);
	trajkovic.setSecondThreshold(SecondThreshold);
	trajkovic.setWindowSize(WindowSize);
	// cout << "Value of WindowSize: " << WindowSize << endl;
	trajkovic.setSearchMethod(tree);
	trajkovic.setInputCloud(cloud_in);
	trajkovic.setNormals(normals);
	pcl::PointCloud<pcl::PointXYZI> result;
	trajkovic.compute(result);
	pcl::copyPointCloud(result, *keypoints_out);
}

void detect_keypoints_harris_3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
								pcl::PointCloud<pcl::Normal>::Ptr &normals,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*points, *cloud_in);
	// cout << "No of points after convertion in the src are " << cloud_in->points.size() << endl;
	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	harris.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
	harris.setInputCloud(cloud_in);
	harris.setNonMaxSupression(true);
	// harris.setRadius(0.01f);
	harris.setNormals(normals);
	// harris.setThreshold(0.01f);
	pcl::PointCloud<pcl::PointXYZI> result;
	harris.compute(result);
	pcl::copyPointCloud(result, *keypoints_out);
}

void detect_keypoints_harris_6d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
								pcl::PointCloud<pcl::Normal>::Ptr &normals,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints_out)
{

	// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZI>);
	// pcl::copyPointCloud(*points, *cloud_in);
	// cout << "No of points after convertion in the src are " << cloud_in->points.size() << endl;
	pcl::HarrisKeypoint6D<pcl::PointXYZRGB, pcl::PointXYZI, pcl::Normal> harris;
	cout << "Value of Threshold: " << Threshold << endl;
	harris.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	harris.setInputCloud(points);
	harris.setNonMaxSupression(true);
	harris.setRadius(radius);
	// harris.setNormals(normals);
	harris.setThreshold(Threshold);
	pcl::PointCloud<pcl::PointXYZI> result;
	harris.compute(result);
	pcl::copyPointCloud(result, *keypoints_out);
}