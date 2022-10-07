#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include<string.h>

#include <iostream>

using namespace std;
using namespace cv;
float getDis(Point2f point0, Point2f pointA)
{
    float distance;
    distance = powf((point0.x - pointA.x), 2) + powf((point0.y - pointA.y), 2);
    distance = sqrtf(distance);
    return distance;
}
Point2f center(Point2f point0, Point2f point1, Point2f point2, Point2f point3)
{
    Point2f PointDis;
    PointDis.x = (point0.x + point1.x + point2.x + point3.x) / 4;
    PointDis.y = (point0.y + point1.y + point2.y + point3.y) / 4;
    return PointDis;
}
float getang(Point2f point0, Point2f pointA)
{
    float angle,a,b;
    a=pointA.y-point0.y;
    b=pointA.x-point0.x;
    angle=atan(a/b);
    return angle;
}
int main(int argc, char *argv[])
{
    if (argc != 2)
        return -1;
    string path = argv[1];
    VideoCapture cap(path);
    if (!cap.isOpened())
    {
        cout << "failed to open the video!" << endl;
        return -1;
    }
    Mat img, binary, morimg;
    while (cap.read(img))
    {
        vector<Mat> channels(3);
        split(img, channels.data());

        Mat minus_channel = channels[0] - channels[1];
        threshold(minus_channel, binary, 60, 255, THRESH_BINARY);

        Mat image = getStructuringElement(MORPH_RECT, Size(8, 8));
        morphologyEx(binary, morimg, MORPH_CLOSE, image);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(morimg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

        vector<RotatedRect> rune_centers;
        vector<Point2f> fourPoints(4);
        vector<Point2f> PointDis(2);

        /// center ///
        for (size_t i = 0; i < contours.size(); i++)
        {
            if ((hierarchy[i][0] != -1 ||
                 hierarchy[i][1] != -1) &&
                hierarchy[i][2] == -1 &&
                hierarchy[i][3] == -1 &&
                contourArea(contours[i]) < 190 &&
                110 < contourArea(contours[i]))
            {
                RotatedRect temp = minAreaRect(contours[i]);
                // drawContours(img, contours, i, Scalar(0, 255, 0), 2);
                rune_centers.push_back(temp);
            }
        }
        for (size_t i = 0; i < rune_centers.size(); i++)
        {
            rune_centers[i].points(fourPoints.data());

            float aline = getDis(fourPoints[0], fourPoints[1]);
            float bline = getDis(fourPoints[1], fourPoints[2]);
            PointDis[0] = center(fourPoints[0], fourPoints[1], fourPoints[2], fourPoints[3]);
            if (aline > bline)
            {
                swap(aline, bline);
            }

            float a = aline / bline;

            for (int j = 0; j < 4; j++)
            {
                if (0.6 < a)
                {
                    line(img, fourPoints[j], fourPoints[(j + 1) % 4], Scalar(0, 0, 255), 2);
                }
            }
        }
        /// other///
        vector<RotatedRect> rune_armors;
        for (size_t i = 0; i < contours.size(); i++)
        {
            if (hierarchy[i][0] == -1 &&
                hierarchy[i][1] == -1 &&
                hierarchy[i][2] == -1 &&
                hierarchy[i][3] != -1 &&
                contourArea(contours[i]) > 400 &&
                contourArea(contours[i]) < 900)
            {
                RotatedRect temp = minAreaRect(contours[i]);
                // drawContours(img, contours, i, Scalar(0, 255, 0), 2);
                rune_armors.push_back(temp);
            }
        }
        for (size_t i = 0; i < rune_armors.size(); i++)
        {
            rune_armors[i].points(fourPoints.data());

            float aline = getDis(fourPoints[0], fourPoints[1]);
            float bline = getDis(fourPoints[1], fourPoints[2]);
            PointDis[1] = center(fourPoints[0], fourPoints[1], fourPoints[2], fourPoints[3]);
            if (aline > bline)
            {
                swap(aline, bline);
            }

            float a = aline / bline;

            for (int j = 0; j < 4; j++)
            {
                if (0.45 < a && a < 0.65)
                {
                    line(img, fourPoints[j], fourPoints[(j + 1) % 4], Scalar(0, 255, 255), 2);
                }
            }
        }

         float distance = getDis(PointDis[0], PointDis[1]);
        if (105<distance&&distance<140)
        {      
        line(img, PointDis[0], PointDis[1], Scalar(0, 0, 255), 2);
        float angle=getang(PointDis[0], PointDis[1]);
        cout << "The distance is " << distance << endl;
        cout<<"The angle is "<<angle<<endl;
        putText(img,"The distance is " + to_string(distance),Point(1,20),FONT_HERSHEY_DUPLEX,0.75,Scalar(255,255,255),1);
        putText(img,"The angle is " + to_string(angle),Point(1,40),FONT_HERSHEY_DUPLEX,0.75,Scalar(255,255,255),1);
        }








        imshow("test2", img);
        // imshow("test3", binary);
        // imshow("test4", morimg);
        if (waitKey(80) == 27) // Esc
            if (waitKey(0) == 27)
                break;
    }
}
