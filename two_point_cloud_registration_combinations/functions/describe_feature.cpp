#define feature_radius 0.1

void compute_FPFH_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						   pcl::PointCloud<pcl::Normal>::Ptr &normals,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

						   pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_out)
{

	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	pfh_est.setRadiusSearch(feature_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}
void compute_PFH_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
						  pcl::PointCloud<pcl::Normal>::Ptr &normals,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
						  pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud(keypoints);
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
void compute_PFHRGB_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
							 pcl::PointCloud<pcl::Normal>::Ptr &normals,
							 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
							 pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors_out)
{

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250> pfhrgbEstimation;

	pfhrgbEstimation.setInputCloud(keypoints);
	pfhrgbEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
	pfhrgbEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pfhrgbEstimation.setSearchMethod(tree);
	// pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfhrgbEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	pfhrgbEstimation.compute(*descriptors_out);
}

void compute_SHOTRGB_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
							  pcl::PointCloud<pcl::Normal>::Ptr &normals,
							  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
							  pcl::PointCloud<pcl::SHOT1344>::Ptr &descriptors_out)
{

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::SHOTColorEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shotrgbEstimation;

	shotrgbEstimation.setInputCloud(keypoints);
	shotrgbEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
	shotrgbEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	shotrgbEstimation.setSearchMethod(tree);
	// pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	shotrgbEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	shotrgbEstimation.compute(*descriptors_out);
}

void compute_SHOT_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
						   pcl::PointCloud<pcl::Normal>::Ptr &normals,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
						   pcl::PointCloud<pcl::SHOT352>::Ptr &descriptors_out)
{

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> shotEstimation;

	shotEstimation.setInputCloud(keypoints);
	shotEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
	shotEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	shotEstimation.setSearchMethod(tree);
	// pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	shotEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	shotEstimation.compute(*descriptors_out);
}
void compute_3dsc_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
						   pcl::PointCloud<pcl::Normal>::Ptr &normals,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
						   pcl::PointCloud<pcl::ShapeContext1980>::Ptr &descriptors_out)
{

	// copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::ShapeContext3DEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ShapeContext1980> SC_Estimation;

	SC_Estimation.setInputCloud(keypoints);
	SC_Estimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
	SC_Estimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	SC_Estimation.setSearchMethod(tree);
	// pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	SC_Estimation.setRadiusSearch(feature_radius);

	// Compute the features
	SC_Estimation.compute(*descriptors_out);
}

void compute_USC_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
						  pcl::PointCloud<pcl::Normal>::Ptr &normals,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
						  pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &descriptors_out)
{

	// Create the PFH estimation class, and pass the input dataset+normals to it
	pcl::UniqueShapeContext<pcl::PointXYZRGB, pcl::UniqueShapeContext1960> uscEstimation;

	uscEstimation.setInputCloud(keypoints);
	uscEstimation.setSearchSurface(cloud); // use all points for analyzing local cloud structure
	// uscEstimation.setInputNormals(normals);
	// alternatively, if cloud is of tpe PointNormal, do pfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	uscEstimation.setSearchMethod(tree);
	// pfhrgbEstimation.setKSearch(100);

	// Use all neighbors in a sphere of radius radius
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	uscEstimation.setRadiusSearch(feature_radius);

	// Compute the features
	uscEstimation.compute(*descriptors_out);
}

void compute_FPFHOMP_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
							  pcl::PointCloud<pcl::Normal>::Ptr &normals,
							  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

							  pcl::PointCloud<pcl::FPFHSignature33>::Ptr &descriptors_out)
{

	pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	pfh_est.setRadiusSearch(feature_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);
	pfh_est.setNumberOfThreads(6);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_PrincipalCurvatures_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
										  pcl::PointCloud<pcl::Normal>::Ptr &normals,
										  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

										  pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &descriptors_out)
{

	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PrincipalCurvatures> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	pfh_est.setRadiusSearch(feature_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_CVFH_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						   pcl::PointCloud<pcl::Normal>::Ptr &normals,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

						   pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_out)
{

	pcl::CVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_ourcvfh_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
							  pcl::PointCloud<pcl::Normal>::Ptr &normals,
							  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

							  pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_out)
{

	pcl::OURCVFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_gasd_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						   pcl::PointCloud<pcl::Normal>::Ptr &normals,
						   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

						   pcl::PointCloud<pcl::GASDSignature512>::Ptr &descriptors_out)
{

	pcl::GASDEstimation<pcl::PointXYZRGB, pcl::GASDSignature512> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	// pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_gasdcolor_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
								pcl::PointCloud<pcl::Normal>::Ptr &normals,
								pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

								pcl::PointCloud<pcl::GASDSignature984>::Ptr &descriptors_out)
{

	pcl::GASDEstimation<pcl::PointXYZRGB, pcl::GASDSignature984> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	// pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_esf_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						  pcl::PointCloud<pcl::Normal>::Ptr &normals,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

						  pcl::PointCloud<pcl::ESFSignature640>::Ptr &descriptors_out)
{

	pcl::ESFEstimation<pcl::PointXYZRGB, pcl::ESFSignature640> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	// pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(keypoints);
	pfh_est.compute(*descriptors_out);
}

void compute_vfh_features(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
						  pcl::PointCloud<pcl::Normal>::Ptr &normals,
						  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,

						  pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors_out)
{

	pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> pfh_est;
	pfh_est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	// pfh_est.setRadiusSearch(0.0);
	// pfh_est.setRadiusSearch(normal_radius);

	// // copy only XYZ data of keypoints for use in estimating features
	// pcl::PointCloud <pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud <pcl::PointXYZRGB>);
	// pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);
	pfh_est.setSearchSurface(points); // use all points for analyzing local cloud structure
	pfh_est.setInputNormals(normals);

	// But only compute features at keypoints
	pfh_est.setInputCloud(points);
	pfh_est.compute(*descriptors_out);
}