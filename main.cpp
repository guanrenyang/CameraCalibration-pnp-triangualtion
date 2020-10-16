#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include <bits/stdc++.h>
using namespace cv;
using namespace std;

int main() {


    Size boardSize=Size(6,9);
    Size imageSize;
    int count=0;
    //scan corner
    vector<Point2f> image_points;//保存每一幅图的角点
    vector<vector<Point2f>> image_points_seq; // 保存检测到的所有角点

    for(count=0;count<41;count++) {
        Mat tempImage = imread("../chess/" + to_string(count) + ".jpg");
        if(count==0)
        {
            imageSize.width=tempImage.cols;
            imageSize.height=tempImage.rows;
        }


        if (findChessboardCorners(tempImage, boardSize, image_points) == 0) {
            cout << "picture :"+to_string(count)+"error: can't find Chess Board Corner";
            exit(1);
        } else {
            Mat grayImage;
            cvtColor(tempImage, grayImage, COLOR_BGR2GRAY);
            cornerSubPix(grayImage, image_points, Size(5, 5), Size(-1, -1), TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 30, 0.1));///

            image_points_seq.push_back(image_points);

            drawChessboardCorners(grayImage, boardSize, image_points, false);

            /*imshow("Win"+to_string(count), grayImage);
            waitKey(0);*/
        }





    }

    Size square_size = Size(17, 17);
    vector<vector<Point3f>> object_points;  //保存标定板上角点的三维坐标

    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0));
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); //摄像机的5个畸变系数：k1,k2,p1,p2,k3,畸变矩阵较小，设为0矩阵不会产生严重的误差
    vector<Mat> tvecsMat;  //每幅图像的旋转向量
    vector<Mat> rvecsMat;  //每幅图像的平移向量

    //初始化交点图的三维坐标
    for(int t=0;t<count;t++)
    {
        vector<cv::Point3f> realPoint;
        for(int i = 0; i < boardSize.height; i++) {
            for(int j = 0; j < boardSize.width; j++) {
                Point3f tempPoint;
                //假设标定板放在世界坐标系中z=0的平面上
                tempPoint.x = i * square_size.width;
                tempPoint.y = j * square_size.height;
                tempPoint.z = 0;
                realPoint.push_back(tempPoint);
            }
        }
        object_points.push_back(realPoint);
    }

    calibrateCamera(object_points, image_points_seq, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

    /*
    for(int i=0;i<cameraMatrix.rows;i++)
    {
        for(int j=0;j<cameraMatrix.cols;j++)
        {
            cout<<cameraMatrix.at<double>(i,j)<<' ';
        }
        cout<<endl;
    }*/

    cout << "相机内参数矩阵：" << endl;
    cout << cameraMatrix << endl << endl;
    cout << "畸变系数：\n";
    cout << distCoeffs << endl << endl << endl;


    /*PNP算法反解相机坐标*/
    for(int i=0;i<count;i++)
    {
        string filename="../chess/";
        filename+=to_string(i);
        filename+=".jpg";

        solvePnP(object_points[i],)

    }


    return 0;
}
