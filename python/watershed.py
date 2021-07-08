import cv2
import numpy as np
from matplotlib import pyplot as plt
import time
from detect import gradient

def watershed_a4(img,a4_mask,leg_mask,debug=False):#빛에 민감함, 마킹을 a4를 둘러쌓듯이 하면 덜하지만 자동으로 하기에는 무리(a4에서 흘러 넘침)
    mask = cv2.bitwise_or(a4_mask, leg_mask).astype(np.int32)
    if debug==True:
        plt.subplot(2, 1, 1), plt.imshow(img),plt.title('input')
        plt.subplot(2,1,2),plt.imshow(mask),plt.title('marker')
        plt.show()
    mask=cv2.watershed(img,mask)#opencv watershed는 window창 끝도 shed로 인식하여 문제가 생김=>지움
    mask=mask.astype(np.uint8)
    mask[mask==1]=255#a4
    mask[mask == 2] = 0#그 외
    mask[:,mask.shape[1]-1]=0
    mask[:,0]=0
    mask[mask.shape[0]-1,:]=0
    mask[0,:]=0
    if debug==True:
        plt.imshow(mask),plt.title('watershed'),plt.show()
    #cv2.imwrite('./incorrect_floor_and_a4.png', mask)
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    grad=gradient(gray)
    if debug==True:plt.imshow(grad),plt.show()
    _, thr2 = cv2.threshold(grad, 3, 255, cv2.THRESH_BINARY_INV)  # a4의 밝기가 일정한 것을 이용해서 오차 줄임
    mask = cv2.bitwise_and(mask, thr2)
    #cv2.imwrite('./floor_and_a4.png', mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if debug==True:
        plt.imshow(mask),plt.title('morphology'),plt.show()
        #cv2.imwrite('./noisy_floor_and_a4.png', mask)
    return mask

def watershed_foot(color,depth,rec,debug=False):#rec=[좌상단 x좌표,좌상단 y좌표, 가로,세로]
    mask=np.zeros((color.shape[0],color.shape[1]),dtype='uint8')
    cv2.rectangle(mask,(0,0),(rec[0]+10,rec[1]+10),1,-1)
    #cv2.rectangle(mask,(rec[0]+(rec[2]//2)-2,rec[1]+(rec[3]//2)-2),(rec[0]+(rec[2]//2)+2,rec[1]+(rec[3]//2)+2),2,-1)#어차피 중앙일테니,hsv 피부색 역투영 사용하는 방법도

    color_hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    skin=cv2.imread('./skin.png')
    skin=cv2.cvtColor(skin,cv2.COLOR_BGR2HSV)
    skin = cv2.calcHist([skin], [0, 1], None, [180, 256], [0, 180, 0, 256])
    bp=cv2.calcBackProject([color_hsv],[0,1],skin,[0,180,0,256],1)
    _,bp=cv2.threshold(bp,1,255,cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bp = cv2.morphologyEx(bp, cv2.MORPH_ERODE, kernel)
    bp[bp==255]=2
    mask=cv2.bitwise_or(bp,mask)


    if debug==True: plt.imshow(mask),plt.title('mark'), plt.show()
    mask = mask.astype(np.int32)
    mask=cv2.watershed(color,mask)
    mask=mask.astype(np.uint8)
    mask[mask==1]=0#a4
    mask[mask == 2] = 255#그 외
    mask[:,mask.shape[1]-1]=0
    mask[:,0]=0
    mask[mask.shape[0]-1,:]=0
    mask[0,:]=0
    if debug==True: plt.imshow(mask),plt.title('watershed'), plt.show()
    color=cv2.bitwise_and(color,color,mask=mask)
    depth = cv2.bitwise_and(depth, depth, mask=mask)
    if debug == True:
        plt.subplot(1, 2, 1), plt.imshow(color), plt.title('color a4')
        plt.subplot(1, 2, 2), plt.imshow(depth), plt.title('depth a4')
        plt.show()
    return color,depth

def get_hull(img, debug=False,app=False):  # watershed된 a4(발 extract)의 convex hull그리면 rectangle에 근사=> 4개의 coner 검출하면 꼭짓점일 것임
    mask = np.zeros_like(img)
    _, img = cv2.threshold(img, 1, 255, cv2.THRESH_BINARY)
    contours, hierachy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    len_max = 0
    for i in range(len(contours)):
        if cv2.arcLength(contours[i], True) > len_max:
            mask = img.copy()
            cnt = contours[i]  # 1이 손모양 주변의 contour
            hull = cv2.convexHull(cnt)
            cv2.drawContours(mask, [hull], 0, 1, 1)
            len_max = cv2.arcLength(cnt, True)
            # print(len_max)

    mask[mask == 255] = 0
    mask[mask == 1] = 255
    canvas=mask.copy()
    if debug == True:
        plt.subplot(1, 2, 1), plt.imshow(img), plt.title('a4')
        plt.subplot(1, 2, 2), plt.imshow(mask), plt.title('convex hull')
        plt.show()
    corners = cv2.goodFeaturesToTrack(mask, 4, 0.01, 10)
    corners = np.int0(corners)
    pts = []
    for i in corners:
        x, y = i.ravel()
        pts.append([x, y])
    pts.sort(key=lambda x: x[1])  # 상위 2개 상단, 하위 2개 하단
    if (pts[0][0] > pts[1][0]):
        tmp = pts[0]
        pts[0] = pts[1]
        pts[1] = tmp
    if (pts[2][0] > pts[3][0]):
        tmp = pts[2]
        pts[2] = pts[3]
        pts[3] = tmp
    l1 = np.sum(np.sqrt((np.array(pts[0]) - np.array(pts[2])) ** 2))
    l2 = np.sum(np.sqrt((np.array(pts[0]) - np.array(pts[1])) ** 2))
    if l1 > l2:
        vertex = [pts[2], pts[0], pts[1], pts[3]]
    else:
        vertex = [pts[3], pts[0], pts[1], pts[2]]
    if debug or app:
        for i in range(4): cv2.circle(mask, (vertex[i][0], vertex[i][1]), 10, (255, 0, 0), 1)
        plt.subplot(1, 2, 1), plt.imshow(img), plt.title('img')
        plt.subplot(1,2,2),plt.imshow(mask), plt.title('corner')
        plt.show()
        # print(vertex)
    return canvas,vertex


def get_a4(color, depth,canvas, vertex, debug=False):  # 210 x 297가 a4사이즈
    mask=np.zeros_like(canvas).astype(np.int32)
    canvas=cv2.merge((canvas,canvas,canvas))
    x=(vertex[0][0]+vertex[1][0]+vertex[2][0]+vertex[3][0])//4
    y = (vertex[0][1] + vertex[1][1] + vertex[2][1] + vertex[3][1]) // 4
    mask=cv2.circle(mask,(x,y),5,2,-1)
    mask = cv2.circle(mask, (5, 5), 5, 1, -1)
    if debug==True:plt.imshow(mask), plt.show()
    mask=cv2.watershed(canvas,mask)
    mask = mask.astype(np.uint8)
    if debug==True:plt.imshow(mask),plt.show()
    mask[mask == 1] = 0  # a4
    mask[mask == 2] = 255  # 그 외
    mask[:, mask.shape[1] - 1] = 0
    mask[:, 0] = 0
    mask[mask.shape[0] - 1, :] = 0
    mask[0, :] = 0


    #tmp = np.stack([mask == 255, depth == 0], axis=-1)
    #tmp = tmp.all(axis=-1)
    #plt.imshow(tmp),plt.show()
    #depth[tmp]=depth[vertex[1][0],vertex[1][1]]

    color = cv2.bitwise_and(color,color,mask=mask)
    depth = cv2.bitwise_and(depth, depth, mask=mask)

    rec=vertex

    if debug:
        plt.subplot(2, 1, 1), plt.imshow(color),plt.title('color')
        plt.subplot(2, 1, 2), plt.imshow(depth),plt.title('depth')
        plt.show()
    return color, depth, rec

def watershed_a4foot(color,depth,rec,debug=False,a4=False):#rec=[좌상단 x좌표,좌상단 y좌표, 가로,세로]
    mask=np.zeros((color.shape[0],color.shape[1]),dtype='uint8')
    cv2.rectangle(mask,(0,0),(rec[1][0]+10,rec[1][1]+10),1,-1)
    cv2.rectangle(mask, (0, rec[0][1]-10), (rec[0][0] + 10, color.shape[1]), 1, -1)
    cv2.rectangle(mask, (rec[2][0]-10, 0), (color.shape[1], rec[2][1]+10), 1, -1)
    cv2.rectangle(mask, (rec[3][0]-10, rec[3][1]-10), (color.shape[1], color.shape[0]), 1, -1)
    #cv2.rectangle(mask,(rec[0]+(rec[2]//2)-2,rec[1]+(rec[3]//2)-2),(rec[0]+(rec[2]//2)+2,rec[1]+(rec[3]//2)+2),2,-1)#어차피 중앙일테니,hsv 피부색 역투영 사용하는 방법도

    color_hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    skin=cv2.imread('./skin.png')
    skin=cv2.cvtColor(skin,cv2.COLOR_BGR2HSV)
    skin = cv2.calcHist([skin], [0, 1], None, [180, 256], [0, 180, 0, 256])
    bp=cv2.calcBackProject([color_hsv],[0,1],skin,[0,180,0,256],1)
    _,bp=cv2.threshold(bp,1,255,cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bp = cv2.morphologyEx(bp, cv2.MORPH_ERODE, kernel)
    bp[bp==255]=2
    mask=cv2.bitwise_or(bp,mask)

    if debug==True: plt.imshow(mask),plt.title('mark'), plt.show()
    mask = mask.astype(np.int32)
    mask=cv2.watershed(color,mask)
    mask=mask.astype(np.uint8)
    if a4==True:
        mask[mask == 1] = 255  # a4
        mask[mask == 2] = 0  # 그 외
    else:
        mask[mask==1]=0#a4
        mask[mask == 2] = 255#그 외

    mask[:,mask.shape[1]-1]=0
    mask[:,0]=0
    mask[mask.shape[0]-1,:]=0
    mask[0,:]=0
    if debug==True: plt.imshow(mask),plt.title('watershed'), plt.show()
    color=cv2.bitwise_and(color,color,mask=mask)
    color = cv2.bitwise_or(color, color, mask=mask)
    depth = cv2.bitwise_and(depth, depth, mask=mask)
    depth = cv2.bitwise_or(depth, depth, mask=mask)
    if debug == True:
        plt.subplot(1, 2, 1), plt.imshow(color), plt.title('color a4')
        plt.subplot(1, 2, 2), plt.imshow(depth), plt.title('depth a4')
        plt.show()
    return color,depth
