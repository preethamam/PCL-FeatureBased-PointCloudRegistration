void findCorrespondencesNormalShooting_FPFH(
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_pfh(
	const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_pfhrgb(
	const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_shotrgb(
	const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_shot(
	const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_usc(
	const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_FPFHOMP(
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_principal(
	const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_cvfh(
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_ourcvfh(
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_gasd(
	const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_gasdcolor(
	const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_esf(
	const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_vfh(
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}

void findCorrespondencesNormalShooting_3dsc(
	const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_src,
	const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_tgt,
	pcl::Correspondences &all_correspondences)
{
	pcl::PointCloud<pcl::PointNormal>::Ptr source;
	pcl::PointCloud<pcl::PointNormal>::Ptr target;
	pcl::copyPointCloud(*fpfhs_src, *source);
	pcl::copyPointCloud(*fpfhs_tgt, *target);

	pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal> est;
	est.setInputSource(source);
	est.setSourceNormals(source);
	est.setInputTarget(target);
	est.setKSearch(10);
	// est.setTargetNormals(tgt_normals);
	est.determineReciprocalCorrespondences(all_correspondences);
}