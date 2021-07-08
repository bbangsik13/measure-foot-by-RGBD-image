#마우스 이벤트로 a4의 꼭짓점을 지정해서 a4만 추출 get_hull함수와 같은 출력을 내도록

import cv2
import numpy as np


def mouse(color):
    print("a4검출을 위해 a4의 꼭짓점 4군데를 왼쪽 마우스로 클릭하세요. 다 클릭하면 아무 버튼이나 누르세요")
    img = color.copy()
    pts = []
    mask = np.zeros((color.shape[0], color.shape[1]), dtype='uint8')
    def click_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if (len(pts) < 4):
                cv2.circle(img, (x, y), 3, (0, 0, 255), -1)
                pts.append([x, y])
                cv2.imshow('img', img)

    cv2.imshow('img', img)
    cv2.setMouseCallback('img', click_event)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

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
    vertex = np.array(vertex, dtype='int32')
    pts = vertex.reshape((-1, 1, 2))
    mask = cv2.polylines(mask, [pts], True, 255, 1)
    #cv2.imshow('mask', mask)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    return mask, vertex

if __name__=="__main__":
    color = cv2.imread('../s_color2depth1.png')
    #img = color.copy()
    #pts = []
    #mask = np.zeros((color.shape[0], color.shape[1]), dtype='uint8')
    mask,pts=mouse(color)
    print(pts)




