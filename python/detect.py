import cv2
import numpy as np
from matplotlib import pyplot as plt
import time

def gradient(arr):#grayscale uint8 이미지의 gradient를 구함
    img_y = np.gradient(arr, axis=0)  # y차원 기울기
    img_x = np.gradient(arr, axis=1)  # x차원 기울기
    return np.sqrt(np.square(img_x) + np.square(img_y)).astype(np.uint8)  # 기울기크기(int)

def detect_a4(img,debug=False):#확실하게 a4인 부분 detect
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grad = gradient(gray)
    hist_gray = cv2.calcHist([gray], [0], None, [255], [0, 255])
    hist_gray[0][0] = 0
    hist_grad = np.gradient(hist_gray, axis=0)
    hist_grad = np.abs(hist_grad)
    ret=np.argmax(hist_grad)#가장 밝은 값이 바닥인 경우가 있어서
    if ret<150:ret=200 #의 경우가 더 잘 되긴함
    #if debug==True:print(ret)
    while (1):
        #if debug==True:print(ret)
        _, thr1 = cv2.threshold(gray, ret, 255, cv2.THRESH_BINARY)
        _, thr2 = cv2.threshold(grad, 10, 255, cv2.THRESH_BINARY_INV)#a4의 밝기가 일정한 것을 이용해서 오차 줄임
        thr = cv2.bitwise_and(thr1, thr2)
        while (1):  # contour가 2개(하나는 배경) 남을 때까지로 변경
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            thr = cv2.morphologyEx(thr, cv2.MORPH_ERODE, kernel)#erode로 a4검출, 바닥의 밝기는 일정하지 않아(반사등) 노이즈처럼 뜸
            cnt, labels, stats, centroids = cv2.connectedComponentsWithStats(thr)
            if (False):
                cv2.imshow('sure a4',thr)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            if (cnt == 2): break
            if (cnt == 1): break
        if (cnt == 1): ret = ret - 1#빈화면인 경우 threshold값 줄이고 다시
        if (cnt == 2):
            if(len(thr[thr == 255])>10):
                break
            else: ret=ret-1
    out = cv2.bitwise_and(gray, gray, mask=thr).astype(np.uint8)
    _, out = cv2.threshold(out, 1, 1023, cv2.THRESH_BINARY)
    out = out//255*1#마커값
    if (debug == True):
        plt.subplot(2, 1, 1), plt.imshow(img), plt.title('input')
        plt.subplot(2,1,2),plt.imshow(out),plt.title('sure a4')
        plt.show()
    return out

def detect_not_a4(color,dep,debug=False):#otsu는 정규분포형 히스토그램이면 사용불가=>depth의 1이 아닌 최소값을 기순으로 line그리기, 어차피 최소값이 다리일 것, hsv 히스토그램 역투영으로 발 검출?
    mask=np.zeros((dep.shape[0],dep.shape[1]),dtype='uint8')
    if(dep.shape[0]>dep.shape[1]):T=dep.shape[1]//6
    else:T=dep.shape[0]//6
    mask=cv2.rectangle(mask,(0,0),(dep.shape[1]-1,dep.shape[0]-1),2,T)
    #mask = cv2.rectangle(mask, (0, 0), (dep.shape[1] - 1, dep.shape[0] - 1), 2, 10)
    color_hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
    skin=cv2.imread('skin.png')
    skin=cv2.cvtColor(skin,cv2.COLOR_BGR2HSV)
    skin = cv2.calcHist([skin], [0, 1], None, [180, 256], [0, 180, 0, 256])
    bp=cv2.calcBackProject([color_hsv],[0,1],skin,[0,180,0,256],1)
    _,bp=cv2.threshold(bp,1,255,cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    bp = cv2.morphologyEx(bp, cv2.MORPH_ERODE, kernel)
    bp[bp==255]=2
    mask=cv2.bitwise_or(bp,mask)
    if(debug==True):
        plt.subplot(2,1,1),plt.imshow(color),plt.title('input')
        plt.subplot(2, 1, 2), plt.imshow(mask), plt.title('sure not a4')
        plt.show()
    return mask
