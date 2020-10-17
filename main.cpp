#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/ml.hpp"
#include <bits/stdc++.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;
using namespace std;
Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
            (
                    ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
                    ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
            );
}

int main() {

    ofstream pnpResultFile("../pnpResult.txt");
    ofstream triangulationFile("../triangulationFile.txt");


    cout<<"----------相机标定----------"<<endl;
    /*相机标定*/
    Size boardSize=Size(6,9);
    Size imageSize;
    int count=0;
    //scan corner
    vector<Point2f> image_points;//保存每一幅图的角点
    vector<vector<Point2f>> image_points_seq; // 保存检测到的所有角点(棋盘坐标系/图像坐标系）
    vector<vector<Point2f>> realImagePoints;// 保存矫正后的图像坐标系角点

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
    vector<vector<Point3f>> object_points;  //保存标定板上角点的三维坐标(世界坐标系）

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
    //标定相机
    calibrateCamera(object_points, image_points_seq, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);

    /*矫正图片和角点*/
    for(int i=0;i<count;i++)
    {
        vector<Point2f> tempImagePoints;

        //读取图像并把每一幅图像转化为灰度图
        Mat tempImg=imread("../chess/"+to_string(i)+".jpg");
        Mat grayImg;
        Mat realGrayImg;
        cvtColor(tempImg,grayImg,COLOR_BGR2GRAY);

        //矫正角点与图片
        undistortPoints(image_points_seq[i],tempImagePoints,cameraMatrix,distCoeffs,noArray(),cameraMatrix);
        undistort(grayImg,realGrayImg,cameraMatrix,distCoeffs);

        realImagePoints.push_back(tempImagePoints);

        //绘制矫正后的角点与图片
        drawChessboardCorners(realGrayImg,boardSize,tempImagePoints,false);
        /*imshow("image"+to_string(i),realGrayImg);
        waitKey(500);
         */
    }

    /*
    for(int i=0;i<cameraMatrix.rows;i++)
    {
        for(int j=0;j<cameraMatrix.cols;j++)
        {
            cout<<cameraMatrix.at<double>(i,j)<<' ';
        }
        cout<<endl;
    }*/
    /*
    cout << "相机内参数矩阵：" << endl;
    cout << cameraMatrix << endl << endl;
    cout << "畸变系数：\n";
    cout << distCoeffs << endl << endl << endl;
    */

    /*PNP算法反解相机坐标*/
    cout<<"----------Pnp----------"<<endl;
    pnpResultFile<<"----------Pnp----------"<<endl;
    vector<Mat> rvecMat,tvecArray;
    for(int i=0;i<count;i++)
    {
        Mat rvecArray,tempTvecArray,tempRvecMat;

        solvePnP(object_points[i],image_points_seq[i],cameraMatrix,distCoeffs,rvecArray,tempTvecArray);

        Rodrigues(rvecArray,tempRvecMat);//旋转矩阵的格式为3*3,平移向量的格式为3*1

        rvecMat.push_back(tempRvecMat);
        tvecArray.push_back(tempTvecArray);
    }

    for(int i=0;i<rvecMat.size();i++)
    {
        cout<<"图片"+to_string(i)+"的旋转矩阵为"<<endl<<rvecMat[i]<<endl;
        cout<<"图片"+to_string(i)+"的平移向量为"<<endl<<tvecArray[i]<<endl;
        cout<<"---------------------"<<endl;

        pnpResultFile<<"图片"+to_string(i)+"的旋转矩阵为"<<endl<<rvecMat[i]<<endl;
        pnpResultFile<<"图片"+to_string(i)+"的平移向量为"<<endl<<tvecArray[i]<<endl;
        pnpResultFile<<"---------------------"<<endl;
    }


    /*三角测量*/
    cout<<"----------三角测量----------"<<endl;
    triangulationFile<<"----------三角测量----------"<<endl;

    Mat T1 = (Mat_<float> (3,4) <<
            rvecMat[0].at<double>(0,0), rvecMat[0].at<double>(0,1), rvecMat[0].at<double>(0,2), tvecArray[0].at<double>(0,0),
            rvecMat[0].at<double>(1,0), rvecMat[0].at<double>(1,1), rvecMat[0].at<double>(1,2), tvecArray[0].at<double>(1,0),
            rvecMat[0].at<double>(2,0), rvecMat[0].at<double>(2,1), rvecMat[0].at<double>(2,2), tvecArray[0].at<double>(2,0)
    );

    Mat T2 = (Mat_<float> (3,4) <<
            rvecMat[1].at<double>(0,0), rvecMat[1].at<double>(0,1), rvecMat[1].at<double>(0,2), tvecArray[1].at<double>(0,0),
            rvecMat[1].at<double>(1,0), rvecMat[1].at<double>(1,1), rvecMat[1].at<double>(1,2), tvecArray[1].at<double>(1,0),
            rvecMat[1].at<double>(2,0), rvecMat[1].at<double>(2,1), rvecMat[1].at<double>(2,2), tvecArray[1].at<double>(2,0)
    );

    vector<Point2f> pts_1, pts_2;
    vector<Point3d> points;
    for (int i=0;i<realImagePoints[0].size();i++)
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back(pixel2cam(realImagePoints[0][i],cameraMatrix));
        pts_2.push_back(pixel2cam(realImagePoints[1][i],cameraMatrix));
    }

    Mat pts_4d;

    triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );

    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化  此处的归一化是指从齐次坐标变换到非齐次坐标。而不是变换到归一化平面。
        Point3d p (
                x.at<float>(0,0),
                x.at<float>(1,0),
                x.at<float>(2,0)
        );
        points.push_back(p);
    }
    for(int i=0;i<points.size();i++)
    {
        triangulationFile<<"the computed coordinate of dot "+to_string(i)+" "<<points[i]<<endl;
    }

    triangulationFile<<endl<<endl<<endl;

    //每个像素点的假定世界坐标与三角测量计算坐标的区别
    cout<<"the difference between World Coordinates we presumed and that solved by triangulation:"<<endl;
    triangulationFile<<"the difference between World Coordinates we presumed and that solved by triangulation:"<<endl;
    for(int i=0;i<points.size();i++)
    {
        Point3d temp;
        temp.x=points[i].x-object_points[0][i].x;
        temp.y=points[i].y-object_points[0][i].y;
        temp.z=points[i].z-object_points[0][i].z;


        cout<<"difference of dot "+to_string(i)+" "<<temp<<endl;
        triangulationFile<<"difference of dot "+to_string(i)+" "<<temp<<endl;

    }
    return 0;
}

