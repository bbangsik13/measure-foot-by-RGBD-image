# measure-foot-by-RGBD-image

자세한 내용은 pdf파일을 참고

1.data는 azure kinect SDK로 얻어온다.
2.python 파일의 main.py의 path를 수정하고 실행한다.
3.detect.py는 이미지(gray)의 gradient(edge), a4의 seed, a4가 아닌 부분의 seed를 구할 수 있다.
4.execute.py는 자동으로 실행하는 부분과 마우스 이벤트로 실행하는 부분으로 나뉜다.
5.mouse.py는 a4의 네 꼭짓점을 지정해서 a4의 영역을 반환한다.
6.pcd.py는 rgbd를 pcd로, 발목부분 위의 point를 제거하는 부분으로 나뉜다.
7.scale.py는 a4의 가로 세로 길이를 통해 발의 길이와 폭을 보정하는 함수이다.
8.transform_measure.py는 xy평면에 정렬하기 위해 RANSAC으로 a4바닥의 평면 방정식을 구하고 평면의 절편을 kabsch algorithm으로 xy평면에 projection 후에 eigen을 통해 장축(발의 길이)을 구해 xy평면(2D라 봐도 무방)에서 rotation을 하고 axis aligned box를 통해 축에 정렬시킨다. 그후 길이와 폭을 구한다. 높이는 yz평면에 projection후에 발목(길이가 급격하게 변화하는 부분)을 구할 수 있다.
9.watershed.py watershed함수와 convexhull로 a4의 네 꼭짓점을 구하는 부분으로 나뉜다.

## Abstract

  color(RGB) image와 depth image를 입력으로 주어 발에 해당하는 pcd와 그 발에 해당하는 특징(길이 폭 높이)를 결과로 반환한다.
  이때 color에서 a4의 네 꼭짓점이 모두 보여야 한다.


## instruction
  
  KINECT camera로 color(RGB) image와 depth image를 찍고 camera intrinsic으로 바로 point cloud를 만드는 것이 아니라 발만 segment해서(color와 depth 모두 같은 영역) point cloud를 만든다. 이 방식의 장점은 부족한 3D segment 알고리즘 대신 2D segment 알고리즘을 사용할 수 있다. 또한 segment를 하고 point cloud를 만드는 것이기 때문에 point cloud를 만들고 segment하는 것 보다 빠르다.(azure kinect SDK로 진행하였다)
  segment는 watershed 알고리즘을 통해 진행하였다. 자동으로 seed를 설정하기 위해 threshold, morphology, bitwise논리 연산 등을 사용하였다.
  사용자 입력(grab cut이 watershed보다 더 강력함)을 사용하면 더 좋은 결과가 있겠지만 상당히 불편하다. 특히 컴퓨터에서 마우스로 지정하기는 더더욱 힘들다. 따라서 확실하게 발(a4)인 부분과 그렇지 않은 부분을 seed로 지정하고 watershed 알고리즘을 적용하였다(grabcut은 더 좋은 결과가 있지만 더 정확한 사용자 입력이 요구되므로 사용하지 않았다).
  A4는 seed를 효과적으로 지정하는데 도움이 되고, 크기가 균일하기 때문에 측정하는데 오차를 보정하는데 사용할 수 있다.
  자동으로 하면서 어느정도 오차가 있기 때문에 사용자가 결과를 보고 만족하지 않으면 사용자 입력을 통해 다시 구할 수 있다.
  발의 pcd를 구하고 transform을 통해 xy축에 정렬하고 특성을 구한다. 이때 a4의 길이를 통해 왜곡에 의한 값을 보정해준다.
