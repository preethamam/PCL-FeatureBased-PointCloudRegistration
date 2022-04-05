import open3d as o3d

def read_keypoints(path):
    f = open(path)
    lines = f.readlines()
    if "src_keypoints_file" in path:
        colors = [[1, 0, 0] for i in range(len(lines))]
    else:
        colors = [[0, 1, 0] for i in range(len(lines))] 
    points = []
    keypoints = o3d.geometry.PointCloud()
    for line in lines:
        point =line.replace("(", "").replace(")","").replace('\n',"").split(",")
        points.append(point)
    keypoints.points = o3d.utility.Vector3dVector(points)
    keypoints.colors = o3d.utility.Vector3dVector(colors) 
    return keypoints

def line_pcd(src_good_path, tgt_good_path):
    src_file = open(src_good_path)
    tgt_file = open(tgt_good_path)
    src_lines = src_file.readlines()
    tgt_lines = tgt_file.readlines()
    points = []
    lines = []
    for line in src_lines:
        a = line.replace("(", "").replace(")","").replace('\n',"").split(",")
        points.append(a)
    for line in tgt_lines:
        point = line.replace("(", "").replace(")","").replace('\n',"").split(",")
        points.append(point)
    for i in range(len(src_lines)):
        lines.append([i,i+len(src_lines)])
    colors = [[0, 0, 1] for i in range(len(lines))]
    line_pcd = o3d.LineSet()
    line_pcd.lines = o3d.Vector2iVector(lines)
    line_pcd.colors = o3d.Vector3dVector(colors)
    line_pcd.points = o3d.Vector3dVector(points)

    return line_pcd

def read_trans(path):
    f = open(path)
    info = f.readlines()
    trans = []
    for line in info:
        a = line.strip("\n").split(" ")
        row = []
        for elem in a:
            if elem:
                row.append(float(elem))
        trans.append(row)
    return trans

root_path = ""


src_pcloud_filename = 'normal.sift.pfh.default.default.src_pcd.pcd'
tgt_pcloud_filename = 'normal.sift.pfh.default.default.tgt_pcd.pcd'

# keypoints

src_keypoints_filename = 'normal.sift.pfh.default.default.src_keypoints_file.txt'
tgt_keypoints_filename = 'normal.sift.pfh.default.default.tgt_keypoints_file.txt'
src_good_keypoints_filename = 'normal.sift.pfh.default.default.src_good_keypoints_file.txt'
tgt_good_keypoints_filename = 'normal.sift.pfh.default.default.tgt_good_keypoints_file.txt'
initial_trans = "normal.sift.pfh.default.default.initial_transformation_matrix.txt"
final_trans ="normal.sift.pfh.default.default.final_transformation_matrix.txt"


src_pcloud = o3d.io.read_point_cloud(root_path+src_pcloud_filename)
tgt_pcloud = o3d.io.read_point_cloud(root_path+tgt_pcloud_filename)
src_keypoints = read_keypoints(root_path+src_keypoints_filename)
tgt_keypoints = read_keypoints(root_path+tgt_keypoints_filename)
o3d.visualization.draw_geometries([src_pcloud,tgt_pcloud,src_keypoints, tgt_keypoints], window_name="Key-points Viewer")
lines = line_pcd(root_path+src_good_keypoints_filename, root_path+tgt_good_keypoints_filename)
o3d.visualization.draw_geometries([src_pcloud,tgt_pcloud,src_keypoints, tgt_keypoints,lines], window_name="Correspondence Viewer")
intial_transformation_matrix = read_trans(root_path+initial_trans)
final_transformation_matrix =  read_trans(root_path+final_trans)
src_pcloud.transform(intial_transformation_matrix)
o3d.visualization.draw_geometries([src_pcloud,tgt_pcloud], window_name="Initial Alignment")
src_pcloud.transform(final_transformation_matrix)
o3d.visualization.draw_geometries([src_pcloud,tgt_pcloud], window_name="ICP Alignment")


