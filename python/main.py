"""
 demonstrate the foot 3D scanning

 (c) bbangsik13@gmail.com

 1. 확실하게 a4인 부분과 확실하게 a4가 아닌 부분을 뽑아 watershed의 marker로 사용해서 a4부분만 뽑는다
 2. a4의 꼭짓점을 뽑아 perspective transform을 통해 a4와 발만 직사각형 형태로 뽑는다
 3. 2에서 다시 watershed를 적용해 발만 뽑거나 a4만 뽑을 수 있음
 4(중요). a4를 기준으로 3차원 축에 정렬

 6. 길이 측정
 #######################완료############################

 5. 발목 위를 제거

 7. a4를 기준으로 하여 여러 이미지들을 pcd합
########################앞으로 할 것##########################

connected component with states

감지 잘 안되는 경우 마우스 이벤트로

왜곡 보정
"""
import cv2
import open3d as o3d
from execute import execute
import cProfile

if __name__ =="__main__":
    debug=False
    draw=False
    app=True

    #color=cv2.imread('../color2depth5.png')
    #depth=cv2.imread('../depth5.png',cv2.IMREAD_UNCHANGED)
    #color=cv2.imread('../foot/color2depth0.png')
    #depth=cv2.imread('../foot/depth0.png',cv2.IMREAD_UNCHANGED)

    #color = cv2.imread('../data/5-19/color2depth1.png')
    #depth = cv2.imread('../data/5-19/depth1.png', cv2.IMREAD_UNCHANGED)
    #o3d_ws_foot = execute(color, depth, debug, draw, app)

    #o3d.io.write_point_cloud("../pcd/non_ps_a4_xy.ply",o3d_a4_foot,write_ascii=True)
    #pcd=rgbd2pcd(color,depth,draw=draw)
    #pcd=get_plane(pcd,debug=debug,draw=draw)
    #transform(pcd,draw=draw)
    #o3d.io.write_point_cloud('../pcd/non_ps_foot_0.ply',o3d_a4_foot,write_ascii=True)
    #o3d.io.write_point_cloud('../pcd/non_ps_ws_foot_0.ply', o3d_ws_foot, write_ascii=True)
    #num=0
    #o3d.io.write_point_cloud('../pcd/non_ps_foot_%d.ply'%(num),o3d_a4_foot,write_ascii=True)
    #o3d.io.write_point_cloud('../pcd/non_ws_foot_%d.ply'%(num), o3d_ws_foot, write_ascii=True)

    '''for num in range(0,15):
        print(num,'번째')
        color = cv2.imread('../data/6-10/color_to_depth%d.png'%(num))
        depth = cv2.imread('../data/6-10/depth%d.png'%(num), cv2.IMREAD_UNCHANGED)
        o3d_ws_foot=execute(color,depth,debug,draw,app)'''


        #o3d.io.write_point_cloud('../pcd/non_ps_foot_%d.ply'%(num),o3d_a4_foot,write_ascii=True)
        #o3d.io.write_point_cloud('../pcd/non_ws_foot_%d.ply'%(num), o3d_ws_foot, write_ascii=True)
    '''for people in range(10, 11):
        for num in range(0, 2):

            color = cv2.imread('../data/final/people%d/color2depth%d.png'%(people,num))
            depth = cv2.imread('../data/final/people%d/depth%d.png'%(people,num), cv2.IMREAD_UNCHANGED)
            o3d_ws_foot=execute(color,depth,debug,draw,app)
    '''
    color = cv2.imread('../data/foot/color2depth0.png')
    depth = cv2.imread('../data/foot/depth0.png', cv2.IMREAD_UNCHANGED)
    #cProfile.run("execute(color, depth, debug, draw, app)")
    execute(color, depth, debug, draw, app)


