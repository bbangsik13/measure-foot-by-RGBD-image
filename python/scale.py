import numpy as np
import copy
import open3d as o3d
import math
from watershed import get_hull
from transform_measure import distance

def scale(projection,debug=False,draw=False):

    pcd=copy.deepcopy(projection)

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(0.1, center=(0, 0, 0))
    if draw==True:o3d.visualization.draw_geometries([pcd, mesh])

    xyz=np.array(pcd.points)
    xy=(xyz[:,0:2]*500).astype(np.int)
    xy[:,0]-=np.min(xy[:,0])
    xy[:, 1] -= np.min(xy[:, 1])
    mask=np.zeros((np.max(xy[:,1])+11,np.max(xy[:,0])+11),dtype='uint8')
    for i in range(xy.shape[0]):
        mask[xy[i][1]+5,xy[i][0]+5]=255


    _,vertex=get_hull(mask,debug)
    vertex.sort(key=lambda x:x[0])#좌하단 좌상단 우상단 우하단
    if vertex[0][1]<vertex[1][1]:
        tmp=vertex[0]
        vertex[0]=vertex[1]
        vertex[1]=tmp
    if vertex[3][1]<vertex[2][1]:
        tmp=vertex[2]
        vertex[2]=vertex[3]
        vertex[3]=tmp

    vertex=np.array(vertex)

    p1=(vertex[1]+vertex[2])//2
    p2=(vertex[0]+vertex[3])//2
    a=abs(p1[0]-p2[0])
    b=abs(p1[1]-p2[1])
    theta=math.atan(a/b)

    if vertex[1][0]>vertex[0][0]:
        theta=-theta

    R=np.eye(4)
    R[0,0]=math.cos(theta)
    R[1,0]=math.sin(theta)
    R[0,1]=-math.sin(theta)
    R[1,1]=math.cos(theta)
    pcd=pcd.transform(R)
    xyz=np.array(pcd.points)
    T=np.eye(4)
    T[0,3]=-np.min(xyz[:,0])
    T[1,3]=-np.min(xyz[:,1])

    pcd.transform(T)

    box = pcd.get_axis_aligned_bounding_box()
    bpp = np.array(box.get_box_points())
    width=distance(bpp[0],bpp[1])*1000
    length=distance(bpp[0],bpp[2])*1000
    width_scale=210/width
    length_scale=297/length
    if draw==True:o3d.visualization.draw_geometries([pcd, mesh,box])
    return width_scale,length_scale

'''if __name__ =="__main__":
    pcd=o3d.io.read_point_cloud("../pcd/transform_a4_foot.ply")
    print(scale(pcd,True,True))'''