void findCorrespondencesBackProjection_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::FPFHSignature33, pcl::FPFHSignature33, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_pfh(const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::PFHSignature125>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::PFHSignature125, pcl::PFHSignature125, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_pfhrgb(const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::PFHRGBSignature250, pcl::PFHRGBSignature250, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_shotrgb(const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::SHOT1344>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::SHOT1344, pcl::SHOT1344, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_shot(const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::SHOT352>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::SHOT352, pcl::SHOT352, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_usc(const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::UniqueShapeContext1960, pcl::UniqueShapeContext1960, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_FPFHOMP(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::FPFHSignature33, pcl::FPFHSignature33, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_principal(const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::PrincipalCurvatures, pcl::PrincipalCurvatures, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_cvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::VFHSignature308, pcl::VFHSignature308, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_ourcvfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::VFHSignature308, pcl::VFHSignature308, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_gasd(const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::GASDSignature512>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::GASDSignature512, pcl::GASDSignature512, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_gasdcolor(const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::GASDSignature984>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::GASDSignature984, pcl::GASDSignature984, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_esf(const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::ESFSignature640>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::ESFSignature640, pcl::ESFSignature640, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_vfh(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::VFHSignature308>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::VFHSignature308, pcl::VFHSignature308, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}

void findCorrespondencesBackProjection_3dsc(const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_src,
											const pcl::PointCloud<pcl::ShapeContext1980>::Ptr &fpfhs_tgt,
											pcl::PointCloud<pcl::Normal>::Ptr &src_normals,
											pcl::PointCloud<pcl::Normal>::Ptr &tgt_normals,
											pcl::Correspondences &all_correspondences)
{

	pcl::registration::CorrespondenceEstimationBackProjection<pcl::ShapeContext1980, pcl::ShapeContext1980, pcl::Normal> est;
	est.setInputSource(fpfhs_src);
	est.setInputTarget(fpfhs_tgt);
	est.setSourceNormals(src_normals);
	est.setTargetNormals(tgt_normals);
	est.setKSearch(10);
	est.determineCorrespondences(all_correspondences);
}
