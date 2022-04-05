void findCorrespondences_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
							  const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
							  pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_PFH(const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
							 const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
							 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_PFHRGB(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
								const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
								pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_SHOTRGB(const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_src,
								 const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_tgt,
								 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::SHOT1344, pcl::SHOT1344> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_SHOT(const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
							  const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
							  pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::SHOT352, pcl::SHOT352> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_3dsc(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_src,
							  const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_tgt,
							  pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::ShapeContext1980, pcl::ShapeContext1980> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_USC(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_src,
							 const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_tgt,
							 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::UniqueShapeContext1960, pcl::UniqueShapeContext1960> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_FPFHOMP(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
								 const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
								 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_PrincipalCurvatures(const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_src,
											 const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_tgt,
											 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::PrincipalCurvatures, pcl::PrincipalCurvatures> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_CVFH(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
							  const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
							  pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_ourcvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
								 const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
								 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_gasd(const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_src,
							  const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_tgt,
							  pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::GASDSignature512, pcl::GASDSignature512> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_gasdcolor(const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_src,
								   const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_tgt,
								   pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::GASDSignature984, pcl::GASDSignature984> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_esf(const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_src,
							 const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_tgt,
							 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::ESFSignature640, pcl::ESFSignature640> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondences_vfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
							 const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
							 pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.determineReciprocalCorrespondences(all_correspondences);
}