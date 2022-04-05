#include "include.h"

using namespace std;
string src_tgt_filepath = "";


string src_file = "";
string tgt_file = "";

// parse command line args
class InputParser
{
public:
	InputParser(int &argc, char **argv)
	{
		for (int i = 1; i < argc; ++i)
			this->tokens.push_back(string(argv[i]));
	}
	const string &getCmdOption(const string &option) const
	{   
		vector<string>::const_iterator itr;
		itr = find(this->tokens.begin(), this->tokens.end(), option);
		if (itr != this->tokens.end() && ++itr != this->tokens.end())
		{
			return *itr;
		}
		static const string empty_string("");
		return empty_string;
	}
// for parameters

	bool cmdOptionExists(const string &option) const
	{
		return find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
	}

private:
	vector<string> tokens;
};


// template to convert the input from cmd to the target datatype
template <class T> 
void convertFromString(T &value, const std::string &s) {
  std::stringstream ss(s);
  ss >> value;
}

int main(int argc, char **argv)
{
	InputParser input(argc, argv);
	// reading the datapath
	string src_tgt_filepath = input.getCmdOption("--src");
	string src_file = input.getCmdOption("--src_file");
	string tgt_file = input.getCmdOption("--tgt_file");
	// reading the combinations
	string normal = input.getCmdOption("--normal");
	string keypoint = input.getCmdOption("--keypoint");
	string feature = input.getCmdOption("--feature");
	string correspondences = input.getCmdOption("--correspondences");
	string reject = input.getCmdOption("--reject");
	string ret = input.getCmdOption("--ret");

	// reading the parameters to do the filtering
	string LEAF_SIZE_cmd = input.getCmdOption("--LEAF_SIZE");
	double LEAF_SIZE_dafault = 0.4;
	convertFromString(LEAF_SIZE_dafault, LEAF_SIZE_cmd);
	// cout << "Value of LEAF_SIZE in cmd: " << LEAF_SIZE_dafault << endl;
	LEAF_SIZE =LEAF_SIZE_dafault;
// reading the parameters to the keypoints extractor
// sift
	string min_scale_cmd = input.getCmdOption("--min_scale");
	double min_scale_dafault = 0.4;
	convertFromString(min_scale_dafault, min_scale_cmd);
	// cout << "Value of min_scale in cmd: " << min_scale_dafault << endl;
	min_scale =min_scale_dafault;

	string nr_octaves_cmd = input.getCmdOption("--nr_octaves");
	double nr_octaves_default = 4;
	convertFromString(nr_octaves_default, nr_octaves_cmd);
	// cout << "Value of nr_octaves in cmd: " << nr_octaves_default << endl;
	nr_octaves =nr_octaves_default;

	string nr_scales_per_octave_cmd = input.getCmdOption("--nr_scales_per_octave");
	double nr_scales_per_octave_default = 5;
	convertFromString(nr_scales_per_octave_default, nr_scales_per_octave_cmd);
	// cout << "Value of nr_scales_per_octave in cmd: " << nr_scales_per_octave_default << endl;
	nr_scales_per_octave =nr_scales_per_octave_default;

	string min_contrast_cmd = input.getCmdOption("--min_contrast");
	double min_contras_octave_default = 0.25;
	convertFromString(min_contras_octave_default, min_contrast_cmd);
	// cout << "Value of min_contrast in cmd: " << min_contras_octave_default << endl;
	min_contrast =min_contras_octave_default;
// iss3d
	string model_resolution_cmd = input.getCmdOption("--model_resolution");
	double model_resolution_default = 0.2;
	convertFromString(model_resolution_default, model_resolution_cmd);
	// cout << "Value of model_resolution in cmd: " << model_resolution_default << endl;
	model_resolution =model_resolution_default;

	string Threshold21_cmd = input.getCmdOption("--Threshold21");
	double Threshold21_default = 0.975;
	convertFromString(Threshold21_default, Threshold21_cmd);
	// cout << "Value of Threshold21 in cmd: " << Threshold21_default << endl;
	Threshold21 =Threshold21_default;

	string Threshold32_cmd = input.getCmdOption("--Threshold32");
	double Threshold32_default = 0.975;
	convertFromString(Threshold32_default, Threshold32_cmd);
	// cout << "Value of Threshold32 in cmd: " << Threshold32_default << endl;
	Threshold32 =Threshold32_default;

	string MinNeighbors_cmd = input.getCmdOption("--MinNeighbors");
	double MinNeighbors_default = 5;
	convertFromString(MinNeighbors_default, MinNeighbors_cmd);
	// cout << "Value of MinNeighbors in cmd: " << MinNeighbors_default << endl;
	MinNeighbors =MinNeighbors_default;

	string NumberOfThreads_cmd = input.getCmdOption("--NumberOfThreads");
	double NumberOfThreads_default = 4;
	convertFromString(NumberOfThreads_default, NumberOfThreads_cmd);
	// cout << "Value of NumberOfThreads in cmd: " << NumberOfThreads_default << endl;
	NumberOfThreads =NumberOfThreads_default;

// susan
	string radius_cmd = input.getCmdOption("--radius");
	double radius_default = 0.1f;
	convertFromString(radius_default, radius_cmd);
	// cout << "Value of radius in cmd: " << radius_default << endl;
	radius =radius_default;

	string radiusSearch_cmd = input.getCmdOption("--radiusSearch");
	double radiusSearch_default = 0.1f;
	convertFromString(radiusSearch_default, radius_cmd);
	// cout << "Value of radiusSearch in cmd: " << radiusSearch_default << endl;
	radiusSearch =radiusSearch_default;

// Trajkovic
	string FirstThreshold_cmd = input.getCmdOption("--FirstThreshold");
	double FirstThreshold_default = 0.00046;
	convertFromString(FirstThreshold_default, FirstThreshold_cmd);
	// cout << "Value of FirstThreshold in cmd: " << FirstThreshold_default << endl;
	FirstThreshold =FirstThreshold_default;
	
	string SecondThreshold_cmd = input.getCmdOption("--SecondThreshold");
	double SecondThreshold_default = 0.03589;
	convertFromString(SecondThreshold_default, SecondThreshold_cmd);
	// cout << "Value of SecondThreshold in cmd: " << SecondThreshold_default << endl;
	SecondThreshold =SecondThreshold_default;
	
	string WindowSize_cmd = input.getCmdOption("--WindowSize");
	double WindowSize_default = 3;
	convertFromString(WindowSize_default, WindowSize_cmd);
	// cout << "Value of WindowSize in cmd: " << WindowSize_default << endl;
	WindowSize =WindowSize_default;

// harris_6d
	string Threshold_cmd = input.getCmdOption("--Threshold");
	double Threshold_default = 0.01f;
	convertFromString(Threshold_default, Threshold_cmd);
	// cout << "Value of Threshold in cmd: " << Threshold_default << endl;
	FirstThreshold =FirstThreshold_default;

// reading the parameters to caculate the normal
	string normal_radius_cmd = input.getCmdOption("--normal_radius");
	double normal_radius_default = 0.1;
	convertFromString(normal_radius_default, normal_radius_cmd);
	// cout << "Value of normal_radius in cmd: " << normal_radius_default << endl;
	normal_radius =normal_radius_default;

//reading the parameters for RANSAC to do the correspondence rejection
	string RANSAC_Inlier_Threshold_cmd = input.getCmdOption("--RANSAC_Inlier_Threshold");
	double RANSAC_Inlier_Threshold_default = 0.2;
	convertFromString(RANSAC_Inlier_Threshold_default, RANSAC_Inlier_Threshold_cmd);
	// cout << "Value of RANSAC_Inlier_Threshold in cmd: " << RANSAC_Inlier_Threshold << endl;
	RANSAC_Inlier_Threshold =RANSAC_Inlier_Threshold_default;

	string RANSAC_Iterations_cmd = input.getCmdOption("--RANSAC_Iterations");
	double RANSAC_Iterations_default = 5000;
	convertFromString(RANSAC_Iterations_default, RANSAC_Iterations_cmd);
	// cout << "Value of RANSAC_Iterations in cmd: " << RANSAC_Iterations_default << endl;
	RANSAC_Iterations =RANSAC_Iterations_default;

//reading the parameters for ICP registration
	string ICP_Iterations_cmd = input.getCmdOption("--ICP_Iterations");
	double ICP_Iterations_default = 10000;
	convertFromString(ICP_Iterations_default, ICP_Iterations_cmd);
	// cout << "Value of ICP_Iterations in cmd: " << ICP_Iterations_default << endl;
	ICP_Iterations =ICP_Iterations_default;
	
	string ICP_TransformationEpsilon_cmd = input.getCmdOption("--ICP_TransformationEpsilon");
	double ICP_TransformationEpsilon_default = 1e-6;
	convertFromString(ICP_TransformationEpsilon_default, ICP_TransformationEpsilon_cmd);
	// cout << "Value of ICP_TransformationEpsilon in cmd: " << ICP_TransformationEpsilon_default << endl;
	ICP_TransformationEpsilon =ICP_TransformationEpsilon_default;

	string ICP_EuclideanFitnessEpsilon_cmd = input.getCmdOption("--ICP_EuclideanFitnessEpsilon");
	double ICP_EuclideanFitnessEpsilon_default = 1;
	convertFromString(ICP_EuclideanFitnessEpsilon_default, ICP_EuclideanFitnessEpsilon_cmd);
	// cout << "Value of ICP_EuclideanFitnessEpsilon in cmd: " << ICP_EuclideanFitnessEpsilon_default << endl;
	ICP_EuclideanFitnessEpsilon =ICP_EuclideanFitnessEpsilon_default;

	string ICP_RANSAC_Inlier_Threshold_cmd = input.getCmdOption("--ICP_RANSAC_Inlier_Threshold");
	double ICP_RANSAC_Inlier_Threshold_default = 0.001;
	convertFromString(ICP_RANSAC_Inlier_Threshold_default, ICP_RANSAC_Inlier_Threshold_cmd);
	// cout << "Value of ICP_RANSAC_Inlier_Threshold in cmd: " << ICP_RANSAC_Inlier_Threshold_default << endl;
	ICP_RANSAC_Inlier_Threshold =ICP_RANSAC_Inlier_Threshold_default;

	string ICP_Max_Correspondence_Distance_cmd = input.getCmdOption("--ICP_Max_Correspondence_Distance");
	double ICP_Max_Correspondence_Distance_default = 0.4;
	convertFromString(ICP_Max_Correspondence_Distance_default, ICP_Max_Correspondence_Distance_cmd);
	// cout << "Value of ICP_Max_Correspondence_Distance in cmd: " << ICP_Max_Correspondence_Distance_default << endl;
	ICP_Max_Correspondence_Distance =ICP_Max_Correspondence_Distance_default;


	// Time start (main function)
	time_t start_computation, end_computation, start_total, end_total;
	time(&start_total);
	time(&start_computation);

	// READ SOURCE AND TARGET FILES
	string src_fullpath = src_tgt_filepath + src_file;
	string tgt_fullpath = src_tgt_filepath + tgt_file;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_original(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_original(new pcl::PointCloud<pcl::PointXYZRGB>);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(src_fullpath, *src_original) == -1 || pcl::io::loadPCDFile<pcl::PointXYZRGB>(tgt_fullpath, *tgt_original) == -1)
	{
		PCL_ERROR("Couldn't read src or tgt file");
		return -1;
	}

	cout << "Src points: " << src_original->points.size() << endl;
	cout << "Tgt points: " << tgt_original->points.size() << endl;
	// cout << "Value of ret: " << ret << endl;
	pcl::io::savePCDFileASCII(ret + "src_pcd.pcd", *src_original);
	pcl::io::savePCDFileASCII(ret + "tgt_pcd.pcd", *tgt_original);

	// Create the filtering object
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt_decimated(new pcl::PointCloud<pcl::PointXYZRGB>);
	filtering(src_original, src_decimated);
	filtering(tgt_original, tgt_decimated);

	cerr << "Src PointCloud after decimation: " << src_decimated->width * src_decimated->height
		 << " data points (" << pcl::getFieldsList(*src_decimated) << ")." << endl;

	cerr << "Tgt PointCloud after decimation: " << tgt_decimated->width * tgt_decimated->height
		 << " data points (" << pcl::getFieldsList(*tgt_decimated) << ")." << endl;

	// Filtered point cloud copy
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	src = src_decimated;
	tgt = tgt_decimated;

	pcl::PointCloud<pcl::Normal>::Ptr src_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr tgt_normals(new pcl::PointCloud<pcl::Normal>);

	// select the normal caculating function
	switch (hash_(normal.data()))
	{
	case "omp"_hash:
		compute_normalsOMP(src, src_normals);
		compute_normalsOMP(tgt, tgt_normals);
		break;
	default:
		compute_normals(src, src_normals);
		compute_normals(tgt, tgt_normals);
		break;
	}

	// caculate the IntensityGradient for the point cloud
	// IntensityGradient is needed as input when caculating some feature descriptors
	pcl::PointCloud<pcl::IntensityGradient>::Ptr src_ig(new pcl::PointCloud<pcl::IntensityGradient>);
	pcl::PointCloud<pcl::IntensityGradient>::Ptr tgt_ig(new pcl::PointCloud<pcl::IntensityGradient>);

	compute_IntensityGradient(src, src_normals, src_ig);
	compute_IntensityGradient(tgt, tgt_normals, tgt_ig);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
	// select the keypoints extractor functions
	switch (hash_(keypoint.data()))
	{
	case "sift"_hash:
		detect_keypoints_sift(src, keypoints_src);
		cout << "No of sift points in the src are " << keypoints_src->points.size() << endl;
		detect_keypoints_sift(tgt, keypoints_tgt);
		cout << "No of sift points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;

	case "harries3d"_hash:
		detect_keypoints_harris_3d(src, src_normals, keypoints_src);
		cout << "No of harries3d points in the src are " << keypoints_src->points.size() << endl;

		detect_keypoints_harris_3d(tgt, tgt_normals, keypoints_tgt);
		cout << "No of harries3d points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;

	case "harries6d"_hash:
		detect_keypoints_harris_6d(src, src_normals, keypoints_src);
		cout << "No of harries6d points in the src are " << keypoints_src->points.size() << endl;

		detect_keypoints_harris_6d(tgt, tgt_normals, keypoints_tgt);
		cout << "No of harries6d points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;

	case "iss3d"_hash:
		detect_keypoints_iss3d(src, keypoints_src);
		cout << "No of iss3d points in the src are " << keypoints_src->points.size() << endl;

		detect_keypoints_iss3d(tgt, keypoints_tgt);
		cout << "No of iss3d points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;

	case "susan"_hash:
		detect_keypoints_susan(src, src_normals, keypoints_src);
		cout << "No of susan points in the src are " << keypoints_src->points.size() << endl;

		detect_keypoints_susan(tgt, tgt_normals, keypoints_tgt);
		cout << "No of susan points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;

	case "Trajkovic"_hash:
	default:
		detect_keypoints_Trajkovic(src, src_normals, keypoints_src);
		cout << "No of Trajkovic points in the src are " << keypoints_src->points.size() << endl;

		detect_keypoints_Trajkovic(tgt, tgt_normals, keypoints_tgt);
		cout << "No of Trajkovic points in the tgt are " << keypoints_tgt->points.size() << endl;
		break;
	}

	// selecting the feature descriptor functions and the correspondences finder
	// Since the input for correspondences finder is determined by the output of feature descriptor, so we need nested switch here
	pcl::CorrespondencesPtr all_correspondences(new pcl::Correspondences);
	if (feature == "fpfh")
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_FPFH_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_FPFH_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of FPFH feature! " << endl;

		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_FPFH(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_FPFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_FPFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}
	if (feature == "pfh")
	{
		pcl::PointCloud<pcl::PFHSignature125>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PFHSignature125>);
		;
		pcl::PointCloud<pcl::PFHSignature125>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PFHSignature125>);
		;
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_PFH_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_PFH_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of PFH feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_pfh(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_pfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_PFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "pfhrgb")
	{
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PFHRGBSignature250>);
		pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PFHRGBSignature250>);
		compute_PFHRGB_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_PFHRGB_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of PFHRGB feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_pfhrgb(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_pfhrgb(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_PFHRGB(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "shotrgb")
	{
		pcl::PointCloud<pcl::SHOT1344>::Ptr fpfhs_src(new pcl::PointCloud<pcl::SHOT1344>);
		pcl::PointCloud<pcl::SHOT1344>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::SHOT1344>);
		compute_SHOTRGB_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_SHOTRGB_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of SHOTRGB feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_shotrgb(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_shotrgb(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_SHOTRGB(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}

	}

	if (feature == "shot")
	{
		pcl::PointCloud<pcl::SHOT352>::Ptr fpfhs_src(new pcl::PointCloud<pcl::SHOT352>);
		pcl::PointCloud<pcl::SHOT352>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::SHOT352>);
		compute_SHOT_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_SHOT_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of SHOT feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_shot(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_shot(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_SHOT(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "3dsc")
	{
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr fpfhs_src(new pcl::PointCloud<pcl::ShapeContext1980>);
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::ShapeContext1980>);
		compute_3dsc_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_3dsc_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of 3dsc feature! " << endl;
		
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_3dsc(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_3dsc(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_3dsc(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "usc")
	{
		pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr fpfhs_src(new pcl::PointCloud<pcl::UniqueShapeContext1960>);
		pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::UniqueShapeContext1960>);
		compute_USC_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_USC_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of usc feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_usc(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_usc(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_USC(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "FPFHOMP")
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_FPFHOMP_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_FPFHOMP_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of FPFHOMP feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_FPFHOMP(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_FPFHOMP(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_FPFHOMP(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}
	if (feature == "principal")
	{
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr fpfhs_src(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_PrincipalCurvatures_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_PrincipalCurvatures_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of PrincipalCurvatures(feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_principal(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_principal(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_PrincipalCurvatures(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}
	if (feature == "cvfh")
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_src(new pcl::PointCloud<pcl::VFHSignature308>);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::VFHSignature308>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_CVFH_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_CVFH_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of CVFH feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_cvfh(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_cvfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_CVFH(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "ourcvfh")
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_src(new pcl::PointCloud<pcl::VFHSignature308>);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::VFHSignature308>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_ourcvfh_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_ourcvfh_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of ourcvfh feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_ourcvfh(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_ourcvfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_ourcvfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "gasd")
	{
		pcl::PointCloud<pcl::GASDSignature512>::Ptr fpfhs_src(new pcl::PointCloud<pcl::GASDSignature512>);
		pcl::PointCloud<pcl::GASDSignature512>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::GASDSignature512>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_gasd_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_gasd_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of gasd feature! " << endl;
		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_gasd(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_gasd(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_gasd(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "gasdcolor")
	{
		pcl::PointCloud<pcl::GASDSignature984>::Ptr fpfhs_src(new pcl::PointCloud<pcl::GASDSignature984>);
		pcl::PointCloud<pcl::GASDSignature984>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::GASDSignature984>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_gasdcolor_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_gasdcolor_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of gasdcolor feature! " << endl;

		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_gasdcolor(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_gasdcolor(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_gasdcolor(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "esf")
	{
		pcl::PointCloud<pcl::ESFSignature640>::Ptr fpfhs_src(new pcl::PointCloud<pcl::ESFSignature640>);
		pcl::PointCloud<pcl::ESFSignature640>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::ESFSignature640>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_esf_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_esf_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of esf feature! " << endl;

		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_esf(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_esf(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_esf(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	if (feature == "vfh")
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_src(new pcl::PointCloud<pcl::VFHSignature308>);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::VFHSignature308>);
		// const float feature_radius = 0.2;		// adjust this hyperparameter
		compute_vfh_features(src, src_normals, keypoints_src, fpfhs_src);
		compute_vfh_features(tgt, tgt_normals, keypoints_tgt, fpfhs_tgt);
		cout << "End of vfh feature! " << endl;

		switch (hash_(correspondences.data()))
		{
		case "back"_hash:
			findCorrespondencesBackProjection_vfh(fpfhs_src, fpfhs_tgt, src_normals, tgt_normals, *all_correspondences);
			break;
		case "normal"_hash:
			findCorrespondencesNormalShooting_vfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		default:
			findCorrespondences_vfh(fpfhs_src, fpfhs_tgt, *all_correspondences);
			break;
		}
	}

	cout << "End of findCorrespondences! " << endl;
	cout << "All correspondences size: " << all_correspondences->size() << endl;

	// convert the XYZ to XYZRGB
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_tgt_visualize_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints_src, *keypoints_src_visualize_temp);
	pcl::copyPointCloud(*keypoints_tgt, *keypoints_tgt_visualize_temp);

	cout << "Number of points in the keypoints_src_visualize_temp are " << keypoints_src_visualize_temp->points.size() << endl;
	cout << "Number of points in the keypoints_tgt_visualize_temp are " << keypoints_tgt_visualize_temp->points.size() << endl;

	pcl::CorrespondencesPtr good_correspondences(new pcl::Correspondences);
	switch (hash_(reject.data()))
	{
	case "distance"_hash:
		rejectBadCorrespondences_distance(all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
		break;
	case "median"_hash:
		rejectBadCorrespondences_MedianDistance(all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
		break;
	case "poly"_hash:
		rejectBadCorrespondences_poly(all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
		break;
	default:
		rejectBadCorrespondences(all_correspondences, keypoints_src, keypoints_tgt, *good_correspondences);
		break;
	}

	cout << "End of rejectBadCorrespondences! " << endl;
	cout << "Good correspondences size: " << good_correspondences->size() << endl;

	// Estimate the transformation matrix based on the initial registration by using SVD 
	Eigen::Matrix4f transform;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
	trans_est.estimateRigidTransformation(*keypoints_src_visualize_temp, *keypoints_tgt_visualize_temp, *good_correspondences, transform);
	cout << "Initial Transformation Matrix" << endl;
	std::cout << transform << std::endl;

	// writing the intial keypoints into the disk
	ofstream src_keypoints_file;
	ofstream tgt_keypoints_file;
	src_keypoints_file.open(ret + "src_keypoints_file.txt");
	tgt_keypoints_file.open(ret + "tgt_keypoints_file.txt");

	for (int i = 0; i < keypoints_src_visualize_temp->points.size(); ++i)
		src_keypoints_file << keypoints_src_visualize_temp->points[i] << endl;

	for (int i = 0; i < keypoints_tgt_visualize_temp->points.size(); ++i)
		tgt_keypoints_file << keypoints_tgt_visualize_temp->points[i] << endl;

	src_keypoints_file.close();
	tgt_keypoints_file.close();

	// Writing the good keypoints into the disk based on the good correspondences 
	ofstream src_good_keypoints_file;
	ofstream tgt_good_keypoints_file;
	src_good_keypoints_file.open(ret + "src_good_keypoints_file.txt");
	tgt_good_keypoints_file.open(ret + "tgt_good_keypoints_file.txt");

	for (int i = 0; i < good_correspondences->size(); ++i)
	{
		pcl::PointXYZ &src_idx = keypoints_src_visualize_temp->points[(*good_correspondences)[i].index_query];
		pcl::PointXYZ &tgt_idx = keypoints_tgt_visualize_temp->points[(*good_correspondences)[i].index_match];
		src_good_keypoints_file << src_idx << endl;
		tgt_good_keypoints_file << tgt_idx << endl;
	}

	src_good_keypoints_file.close();
	tgt_good_keypoints_file.close();


	// // Transform the result of intial registration  and write it to disk
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_initial_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*src, *src_initial_output, transform);



	// ICP registration and write the final result into the disk
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_output(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f ICP_transformation;

	ICP(src_initial_output, tgt, *Final, ICP_transformation);
	pcl::transformPointCloud(*src_initial_output, *final_output, ICP_transformation);

	pcl::io::savePCDFileASCII(ret + "initial.pcd", *src_initial_output);
	pcl::io::savePCDFileASCII(ret + "final.pcd", *final_output);

	// Write initial transformation matrices to a text file
	ofstream initial_transformation_matrix_file;
	initial_transformation_matrix_file.open(ret + "initial_transformation_matrix.txt");
	initial_transformation_matrix_file << transform << '\n';
	initial_transformation_matrix_file.close();

	// Write final transformation matrices to a text file
	ofstream final_transformation_matrix_file;
	final_transformation_matrix_file.open(ret + "final_transformation_matrix.txt");
	final_transformation_matrix_file << ICP_transformation << '\n';
	final_transformation_matrix_file.close();

	// Time end (main function)
	time(&end_total);
	double time_elapsed_total = difftime(end_total, start_total);
	cout << "Elasped total main function time in seconds: " << time_elapsed_total << endl;

}
