import cv2
import numpy as np
from matplotlib import pyplot as plt
import open3d as o3d
import time

from detect import detect_a4, detect_not_a4
from watershed import watershed_a4,get_hull,get_a4,watershed_a4foot
from pcd import rgbd2pcd,cut_pcd
from transform_measure import get_plane,get_projection_matrix,measure,get_height
from scale import scale
from mouse import mouse

def execute_auto(color,depth,debug,draw,app):
    start = time.time()
    if debug==True:
        plt.subplot(1,2,1),plt.imshow(color),plt.title('input color image')
        plt.subplot(1, 2, 2), plt.imshow(depth), plt.title('input depth image')
        plt.show()
    a4_mask=detect_a4(color,debug=debug)
    leg_mask=detect_not_a4(color,depth,debug=debug)
    mask=watershed_a4(color,a4_mask,leg_mask,debug=debug)
    canvas,vertex=get_hull(mask,debug=debug)

    ps_color,ps_depth,rec=get_a4(color,depth,canvas,vertex,debug=debug)#크기가 color랑 같아야 함
    o3d_a4_foot = rgbd2pcd(ps_color, ps_depth,draw=draw)
    ws_color,ws_depth=watershed_a4foot(ps_color,ps_depth,rec,debug=debug)
    a4_color, a4_depth = watershed_a4foot(ps_color, ps_depth, rec, debug=debug, a4=True)

    o3d_ws_foot=rgbd2pcd(ws_color,ws_depth,draw=draw,normal=False)
    o3d_a4 = rgbd2pcd(a4_color, a4_depth, draw=draw)

    mesh, pt_map = o3d_ws_foot.hidden_point_removal(camera_location=np.array([0., 0., 0]), radius=125.)
    o3d_ws_foot = o3d_ws_foot.select_by_index(pt_map)

    o3d_a4 = get_plane(o3d_a4, debug=debug, draw=draw)
    M=get_projection_matrix(o3d_a4,debug=debug,draw=draw)
    o3d_ws_foot=o3d_ws_foot.transform(M)
    o3d_a4_foot=o3d_a4_foot.transform(M)
    if np.min(np.array(o3d_ws_foot.points)[:,2])<-0.05:
        R=np.eye(4)
        R[1,1]=-1
        R[2,2]=-1
        o3d_ws_foot.transform(R)
        o3d_a4_foot.transform(R)

    width_scale,lenght_scale= scale(o3d_a4_foot,debug=debug,draw=draw)
    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(0.1, center=(0, 0, 0))
    if (np.min(np.array(o3d_ws_foot.points)[:, 2]) < -0.05):
        R = np.eye(4)
        R[2, 2] = -1
        R[1, 1] = -1
        o3d_ws_foot.transform(R)
        o3d_a4_foot.transform(R)
    if draw == True: o3d.visualization.draw_geometries([mesh, o3d_ws_foot], window_name="project", width=720, height=480)

    o3d_ws_foot,o3d_a4_foot,width,length=measure(o3d_ws_foot,o3d_a4_foot,draw=draw)

    width*=width_scale
    length*=lenght_scale

    #o3d.io.write_point_cloud("./pcd/transform_a4_foot.ply",o3d_a4_foot,write_ascii=True)
    total=time.time() - start

    if draw or app: o3d.visualization.draw_geometries([mesh, o3d_ws_foot], window_name="transform foot", width=720, height=480)
    start = time.time()
    height=get_height(o3d_ws_foot,debug)
    print("발의 폭은 %dmm, 발의 길이는 %dmm,발의 높이는 %dmm"%(width,length,height))
    if (app == True) and (draw == False) and (debug == False): print('걸린 시간 %f sec' % (total + time.time() - start))
    o3d_ws_foot=cut_pcd(o3d_ws_foot,height,draw=draw,app=app)
    return o3d_ws_foot



def execute_with_mouse(color,depth,debug,draw,app):

    if debug==True:
        plt.subplot(1,2,1),plt.imshow(color),plt.title('input color image')
        plt.subplot(1, 2, 2), plt.imshow(depth), plt.title('input depth image')
        plt.show()

    canvas, vertex = mouse(color)

    ps_color,ps_depth,rec=get_a4(color,depth,canvas,vertex,debug=debug)#크기가 color랑 같아야 함
    o3d_a4_foot = rgbd2pcd(ps_color, ps_depth,draw=draw)
    ws_color,ws_depth=watershed_a4foot(ps_color,ps_depth,rec,debug=debug)
    a4_color, a4_depth = watershed_a4foot(ps_color, ps_depth, rec, debug=debug, a4=True)

    o3d_ws_foot=rgbd2pcd(ws_color,ws_depth,draw=draw,normal=False)
    o3d_a4 = rgbd2pcd(a4_color, a4_depth, draw=draw)

    mesh, pt_map = o3d_ws_foot.hidden_point_removal(camera_location=np.array([0., 0., 0.02]), radius=125.)
    o3d_ws_foot = o3d_ws_foot.select_by_index(pt_map)

    o3d_a4 = get_plane(o3d_a4, debug=debug, draw=draw)
    M=get_projection_matrix(o3d_a4,debug=debug,draw=draw)
    o3d_ws_foot=o3d_ws_foot.transform(M)
    o3d_a4_foot=o3d_a4_foot.transform(M)
    if np.min(np.array(o3d_ws_foot.points)[:,2])<-0.05:
        R=np.eye(4)
        R[1,1]=-1
        R[2,2]=-1
        o3d_ws_foot.transform(R)
        o3d_a4_foot.transform(R)

    mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    mesh.scale(0.1, center=(0, 0, 0))
    if (np.min(np.array(o3d_ws_foot.points)[:, 2]) < -0.05):
        R = np.eye(4)
        R[2, 2] = -1
        R[1, 1] = -1
        o3d_ws_foot.transform(R)
        o3d_a4_foot.transform(R)
    if draw == True: o3d.visualization.draw_geometries([mesh, o3d_ws_foot], window_name="project", width=720, height=480)

    o3d_ws_foot,o3d_a4_foot,width,length=measure(o3d_ws_foot,o3d_a4_foot,draw=draw)

    #o3d.io.write_point_cloud("./pcd/transform_a4_foot.ply",o3d_a4_foot,write_ascii=True)

    if draw or app: o3d.visualization.draw_geometries([mesh, o3d_ws_foot], window_name="transform foot", width=720, height=480)

    height=get_height(o3d_ws_foot,debug)
    print("발의 폭은 %dmm, 발의 길이는 %dmm,발의 높이는 %dmm"%(width,length,height))
    o3d_ws_foot=cut_pcd(o3d_ws_foot,height,draw=draw,app=app)

    return o3d_ws_foot

def execute(color,depth,debug,draw,app):
    o3d_ws_foot = execute_auto(color, depth, debug, draw, app)
    print("결과가 마음에 들지 않으면 1을 누르세요")
    a = input()
    if a == '1': o3d_ws_foot = execute_with_mouse(color, depth, debug, draw, app)
    return o3d_ws_foot