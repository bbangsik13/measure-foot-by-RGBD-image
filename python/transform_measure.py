import numpy as np
from matplotlib import pyplot as plt
import copy
import open3d as o3d
import math
import time

def get_plane(pcd,debug=False,draw=False):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    if False:print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = pcd.select_by_index(inliers)
    if draw==True:o3d.visualization.draw_geometries([inlier_cloud],window_name="get plane",width=720,height=480)
    return inlier_cloud

def rigid_transform_3D(A, B):
    assert A.shape == B.shape
    num_rows, num_cols = A.shape
    if num_rows != 3:
        raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")
    num_rows, num_cols = B.shape
    if num_rows != 3:
        raise Exception(f"matrix B is not 3xN, it is {num_rows}x{num_cols}")
    centroid_A = np.mean(A, axis=1)
    centroid_B = np.mean(B, axis=1)
    centroid_A = centroid_A.reshape(-1, 1)
    centroid_B = centroid_B.reshape(-1, 1)
    Am = A - centroid_A
    Bm = B - centroid_B
    H = Am @ np.transpose(Bm)
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T @ U.T
    t = -R @ centroid_A + centroid_B
    return R, t

def distance(pts1,pts2):
    dis=pts1-pts2
    dis=dis*dis
    dis=np.sum(dis)
    dis=np.sqrt(dis)
    return dis

def get_projection_matrix(pcd,debug=False,draw=False):
    plane_model, _ = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    if debug==True:print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    pts1=np.array([[-d/a,0,0],[0,-d/b,0],[0,0,-d/c]])
    pts2=np.zeros_like(pts1)
    d1=distance(pts1[1],pts1[2])
    d2=distance(pts1[0],pts1[1])
    d3=distance(pts1[0],pts1[2])
    h2=d2**2-((d3**2+d2**2-d1**2)**2)/(4*(d3**2))
    h=math.sqrt(h2)
    a=math.sqrt(d1**2-h2)

    pts2[0]=[d3,0,0]
    pts2[1] = [a, h, 0]
    pts2[2]=[0,0,0]

    pts1=pts1.T
    pts2=pts2.T
    rotate,translate=rigid_transform_3D(pts1,pts2)
    T=np.eye(4)
    T[:3,:3]=rotate
    T[0,3]=translate[0]
    T[1,3]=translate[1]
    T[2,3]=translate[2]


    return T

def match_xy_plane(pcd, debug=False):
    # ================평면 구하기==============#
    xyz = np.asarray(pcd.points)
    point_num = xyz.shape[0]
    X = np.ones((point_num, 1))
    X = np.append(X, xyz[:, :2], axis=1)
    beta = np.zeros((3, 1))
    Y = xyz[:, 2].reshape(-1, 1)
    pseudo_inv = np.dot(np.linalg.inv(np.dot(X.T, X)), X.T)
    beta = np.dot(pseudo_inv, Y)

    # ================평면 위 임의 점==============#
    x = np.linspace(-1., 1., 2)
    y = np.linspace(-1., 1., 2)
    mx, my = np.meshgrid(x, y)
    mz = (beta[1][0] * mx + beta[2][0] * my + beta[0][0])
    mxyz = np.append(mx, np.append(my, mz)).reshape((3, -1))

    # ================각 절편 구하기=============#
    zz = beta[0][0]
    yy = (-1) * beta[0][0] / beta[2][0]
    xx = (-1) * beta[0][0] / beta[1][0]

    # ================Y Rotate=============#
    xztheta = np.arctan(zz / xx)
    Ry = np.array([[np.cos(-xztheta), 0, np.sin(-xztheta)],
                   [0, 1, 0],
                   [(-1) * np.sin(-xztheta), 0, np.cos(-xztheta)]])
    temp = np.dot(Ry, mxyz).T

    # ================X Rotate=============#
    yztheta = np.arctan2((temp[0][2] - temp[3][2]), (temp[0][1] - temp[3][1]))

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(yztheta), np.sin(yztheta)],
                   [0, (-1) * np.sin(yztheta), np.cos(yztheta)]])

    R = np.dot(Rx, Ry)

    # ================X-Y 평면 평행=============#
    pcd_r = copy.deepcopy(pcd)
    pcd_r.rotate(R, center=(0, 0, 0))

    # ================Z trnslate=============#

    pcd_r.translate((0, 0, -np.min(np.asarray(pcd_r.points)[:, 2])))

    if debug:
        # ================Coordinate=============#
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        mesh.scale(0.5, center=(0, 0, 0))

        # ==============visualization==============#
        o3d.visualization.draw_geometries([mesh, pcd_r], window_name="match xy plane", width=720, height=480)
    return pcd_r, R


def get_xy_mean_and_eigenvectors (pcd):
    hull, _ = pcd.compute_convex_hull()
    vert = np.asarray(hull.vertices)
    cov = np.cov(vert[:,0], vert[:,1])
    _, _, v = np.linalg.svd(cov)
    mean = np.mean(vert[:2], axis=0)
    return mean[0], mean[1], v

def measure(o3d_ws_foot,o3d_a4_foot,draw=False):
    x_mean, y_mean, evec = get_xy_mean_and_eigenvectors(o3d_ws_foot)

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(0.1, center=(0, 0, 0))

    # z-회전 행렬 구하기
    theta = np.arctan2(evec[0][0], evec[1][0])
    Rz = np.array([[np.cos(theta), (-1) * np.sin(theta), 0],
                   [np.sin(theta), np.cos(theta), 0],
                   [0, 0, 1]])

    # x, y의 기대값을 중앙으로 이동 후 Z-회전, 제 1사분면으로 이동
    result = copy.deepcopy(o3d_ws_foot)
    a4=copy.deepcopy(o3d_a4_foot)
    result.translate((-x_mean, -y_mean, 0))
    result.rotate(Rz, center=(0, 0, 0))
    result.translate((-np.min(np.asarray(result.points)[:, 0]), -np.min(np.asarray(result.points)[:, 1]), 0))


    aabb = result.get_axis_aligned_bounding_box()
    bp=np.array(aabb.get_box_points())
    aabb.color = (0, 0, 0)

    if draw==True:o3d.visualization.draw_geometries([mesh, result, aabb], window_name="bounding box", width=720, height=480)
    width=bp[1][0]*1000
    length=bp[2][1]*1000
    #print(width,length)
    if width>length:
        tmp=width
        width=length
        length=tmp

    cut=copy.deepcopy(result)
    rgb = np.asarray(cut.colors)
    xyz = np.asarray(cut.points)
    xyzrgb = np.zeros((xyz.shape[0], 6))
    xyzrgb[:, :3] = xyz
    xyzrgb[:, 3:6] = rgb
    xyzrgb = xyzrgb[xyzrgb[:, 1] <length/2000]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyzrgb[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(xyzrgb[:, 3:6])
    box = pcd.get_axis_aligned_bounding_box()
    if draw == True: o3d.visualization.draw_geometries([pcd, box], window_name="width1", width=720, height=480)
    bpp = np.array(box.get_box_points())
    width1=bpp[1][0]*1000

    xyzrgb = np.zeros((xyz.shape[0], 6))
    xyzrgb[:, :3] = xyz
    xyzrgb[:, 3:6] = rgb
    xyzrgb = xyzrgb[xyzrgb[:, 1] >= length / 2000]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyzrgb[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(xyzrgb[:, 3:6])
    box = pcd.get_axis_aligned_bounding_box()
    if draw == True: o3d.visualization.draw_geometries([pcd, box], window_name="width2", width=720, height=480)
    bpp = np.array(box.get_box_points())
    width2 = bpp[1][0] * 1000

    if width1>width2:width=width2
    else:width=width1




    return result,a4 ,width,length

def get_height(pcd,debug=False):#yz평면으로 projection
    xyz = np.array(pcd.points)
    xyz[:,2]-=np.min(xyz[:,2])
    xyz[:, 1] -= np.min(xyz[:, 1])
    xyz=(1000*xyz).astype(np.uint16)

    mask = np.zeros((np.max(xyz[:, 2]) + 1, np.max(xyz[:, 1]) + 1), dtype='uint16')
    for i in range(xyz.shape[0]):
        mask[xyz[i][2], xyz[i][1]] = 255

    if debug==True:plt.imshow(mask),plt.show()

    l=np.zeros((np.max(xyz[:,2])//5)+1)
    count=l.copy()
    for i in range(0,np.max(xyz[:,2])//5+1):
        canvas=mask.copy()
        canvas[5*i+5:,:]=0
        canvas[:5*i,:]=0

        if False:plt.imshow(canvas),plt.title("%d~%d"%(10*i,10*i+10)),plt.show()
        a=np.where(canvas==255)
        if len(a[0])==0:continue
        count[i]=len(canvas[canvas==255])
        l[i]=np.max(a[1])-np.min(a[1])

    if debug==True:print(l)
    grad=abs(np.gradient(l))
    grad[count < np.max(count) // 40] = 0
    grad[0:12]=0
    grad[20:]=0
    #print(grad)

    height=np.argmax(grad)*5+5
    if debug==True:
        plt.subplot(1,3,2),plt.plot(grad,color='g'), plt.xlim([0, 30]),plt.title("distance grad")
        plt.subplot(1,3,1),plt.plot(l,color='b'),plt.xlim([0,30]),plt.title("distance")
        plt.subplot(1, 3, 3), plt.plot(count, color='r'), plt.xlim([0, 30]), plt.title("count")
        plt.show()

    return height
