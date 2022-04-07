normal=("omp" "normal")
keypoint=("sift" "harries3d" "harries6d" "iss3d" "susan" "Trajkovic")
feature=("fpfh" "pfh" "pfhrgb" "shot" "3dsc" "usc" "FPFHOMP" "principal" "cvfh" "ourcvfh" "gasd" "gasdcolor" "esf" "vfh")
correspondences=("back" "normal" "default")
reject=("distance" "median" "poly" "default")

# root path for the dataset
src=""
# file path for source and target path
src_file=""
tgt_file=""
# result path to store the log and results
ret=""

# Parameters for filtering
LEAF_SIZE="0.1"
# Parameters for sift
min_scale="0.4"
nr_octaves="4";
nr_scales_per_octave="5";
min_contrast="0.25";
# Parameters iss3d
model_resolution="0.2";
Threshold21="0.975";
Threshold32="0.975";
MinNeighbors="5"
NumberOfThreads="4"

# susan
radius='f'
radiusSearch='f'

# Trajkovic
FirstThreshold='f'
SecondThreshold=''
WindowSize=''

# harris_6d
Threshold=''

# normal 
normal_radius=""

# RANSAC
RANSAC_Inlier_Threshold=0.2
RANSAC_Iterations=5000

# ICP hyper parameters
ICP_Iterations=10000
ICP_TransformationEpsilon=1e-6
ICP_EuclideanFitnessEpsilon=1
ICP_RANSAC_Inlier_Threshold=0.001
ICP_Max_Correspondence_Distance=0.4
model_resolution=""
Threshold21=""
Threshold32=""
MinNeighbors=""
NumberOfThreads=""

# susan
radius=''
radiusSearch=''

# Trajkovic
FirstThreshold=''
SecondThreshold=''
WindowSize=''

# harris_6d
Threshold=''

# normal 
normal_radius=""

# RANSAC
RANSAC_Inlier_Threshold=""
RANSAC_Iterations=""

# ICP hyper parameters
ICP_Iterations=""
ICP_TransformationEpsilon=""
ICP_EuclideanFitnessEpsilon=""
ICP_RANSAC_Inlier_Threshold=""
ICP_Max_Correspondence_Distance=""

for n in ${normal[@]}
do
    for k in ${keypoint[@]}
    do
        for f in ${feature[@]}
        do
            for c in ${correspondences[@]}
            do
                for r in ${reject[@]}
                do
                    ./build/project_two_pointcloud_registrationBasedOnRegistration --src $src --src_file $src_file --tgt_file $tgt_file --LEAF_SIZE $LEAF_SIZE --normal $n --keypoint $k --ICP_Iterations $ICP_Iterations --ICP_TransformationEpsilon $ICP_TransformationEpsilon --ICP_EuclideanFitnessEpsilon $ICP_EuclideanFitnessEpsilon --ICP_RANSAC_Inlier_Threshold $ICP_RANSAC_Inlier_Threshold --ICP_Max_Correspondence_Distance $ICP_Max_Correspondence_Distance--RANSAC_Inlier_Threshold $RANSAC_Inlier_Threshold --RANSAC_Iterations $RANSAC_Iterations --normal_radius $normal_radius --min_scale $min_scale --nr_octaves $nr_octaves --nr_scales_per_octave $nr_scales_per_octave --min_contrast $min_contrast --model_resolution $model_resolution --Threshold21 $Threshold21 --Threshold32 $Threshold32 --MinNeighbors $MinNeighbors --NumberOfThreads $NumberOfThreads --radius $radius --radiusSearch $radiusSearch --feature $f --correspondences $c --reject $r --ret "$ret/$n.$k.$f.$c.$r." >> "$ret/$n.$k.$f.$c.$r.log"&
                done
            done
        done
    done 
done
