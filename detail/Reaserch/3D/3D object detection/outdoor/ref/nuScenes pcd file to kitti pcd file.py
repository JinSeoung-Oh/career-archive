import numpy as np
from pyquaternion import Quaternion

def space_rotate(points, rot_matrix):
    points[:3, :] = np.dot(rot_matrix, points[:3, :])
    
    return points
    
def write_ply(verts, colors, indices, out_path):
    if colors is None:
        colors = np.zeros_like(verts)
    if indices is None:
        indices = []

    file = open(out_path, 'w')
    file.write('ply \n')
    file.write('format ascii 1.0\n')
    file.write('element vertex {:d}\n'.format(len(verts)))
    file.write('property float x\n')
    file.write('property float y\n')
    file.write('property float z\n')
    file.write('property uchar red\n')
    file.write('property uchar green\n')
    file.write('property uchar blue\n')
    file.write('element face {:d}\n'.format(len(indices)))
    file.write('property list uchar uint vertex_indices\n')
    file.write('end_header\n')
    
    for vert, color in zip(verts, colors):
        file.write('{:f} {:f} {:f} {:d} {:d} {:d}\n'.format(vert[0], vert[1], vert[2], int(color[0]), int(color[1]), int(color[2])))
                                                           
    for ind in indices:
        file.write('3 {:d} {:d} {:d}\n'.format(ind[0], ind[1], ind[2]))

    file.close()
                    
    print('write_ply is done')   
    
    
def ply_to_pcd(input_path, output_path,img_id):
    #exten = output_path.split('/')[-1].split('.')[-1]
    #if exten == 'pcd':
    #    pcd = o3d.io.read_point_cloud(input_path)
    #    o3d.io.write_point_cloud(output_path, pcd)
    #else:
    output_path = output_path + '/' + str(img_id) + '.pcd'
    pcd = o3d.io.read_point_cloud(input_path)
    o3d.io.write_point_cloud(output_path, pcd)
    #os.remove(input_path)
    print('ply_to_pcd is done') 


#Loads LIDAR data from binary numpy format. Data is stored as (x, y, z, intensity, ring index)
scan = np.fromfile('./3D_apis/smart/nus_bevf/bevfusion/data/nuscenes/samples/LIDAR_TOP/n015-2018-11-21-19-58-31+0800__LIDAR_TOP__1542801733448313.pcd.bin', dtype=np.float32)

points = scan.reshape((-1,5))[:,:3]
kitti_to_nu_lidar = Quaternion(axis=(0,0,1), angle = np.pi/2)
kitti_to_nu_lidar_inv = kitti_to_nu_lidar.inverse
points = space_rotate(points, kitti_to_nu_lidar_inv.rotation_matrix)

colors = []
for i in range(len(points)):
    color = [0,255,255]
    colors.append(color)
    
write_ply(points, colors, None, './test_convert.ply')
ply_to_pcd('./test_convert.ply', './', test_convert)
