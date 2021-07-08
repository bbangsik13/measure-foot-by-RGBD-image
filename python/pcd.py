import cv2
import numpy as np
import open3d as o3d


def rgbd2pcd(color,depth,normal=False,axis=False,draw=False):
    intrinsic = o3d.camera.PinholeCameraIntrinsic(640, 576, 502.724945, 502.854401, 323.970764, 326.964050)  # (width,height,fx,fy,cx,cy) 객체 생성=>intrinsic(내부 파라미터=>카메라 특성)
    extrinsic = np.identity(4)
    color_np = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    depth_np = depth
    o3d_depth = o3d.geometry.Image(depth_np)  # raw image를 open3d image로 변환
    o3d_color = o3d.geometry.Image(color_np)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, convert_rgb_to_intensity=False)  # open3d image로 변환된 color영상과 depth영상을 rgbd image로 변환
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(image=rgbd, intrinsic=intrinsic, extrinsic=extrinsic)  # rgbd 이미지로 point cloud 생성

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.06, origin=[0, 0, 0])#x,y,z축=>r,g,b  &   기준점(0,0,0)
    mesh_frame = mesh_frame.sample_points_poisson_disk(750)
    if draw==True:
        if normal==False:
            if axis==True:
                o3d.visualization.draw_geometries([pcd+mesh_frame],window_name="axis and pcd",width=720,height=480)
            else:o3d.visualization.draw_geometries([pcd],window_name="pcd",width=720,height=480)
        else:
            pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            if axis==True:
                o3d.visualization.draw_geometries([pcd+mesh_frame],point_show_normal=True,window_name="axis and normal vector and pcd",width=720,height=480)
            else:o3d.visualization.draw_geometries([pcd],point_show_normal=True,window_name="normal vector and pcd",width=720,height=480)
    return pcd


def cut_pcd(pcd_or,height,draw=False,app=False):
    rgb = np.asarray(pcd_or.colors)
    xyz = np.asarray(pcd_or.points)
    xyzrgb = np.zeros((xyz.shape[0], 6))
    xyzrgb[:, :3] = xyz
    xyzrgb[:, 3:6] = rgb
    xyzrgb = xyzrgb[xyzrgb[:, 2] < height/1000]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyzrgb[:, :3])
    pcd.colors = o3d.utility.Vector3dVector(xyzrgb[:, 3:6])
    if draw or app:o3d.visualization.draw_geometries([pcd], window_name="cut", width=720, height=480)
    return pcd
