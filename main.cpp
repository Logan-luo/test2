#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <string.h>
#include <iostream>
#include <KalmanFilterX.hpp>
#define PAI 3.14159265359

using namespace std;
using namespace cv;

inline float rad2deg(float rad) { return rad * 180.f / PAI; }
inline float deg2rad(float deg) { return deg / 180.f * PAI; }

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
    float angle, a, b;
    a = pointA.y - point0.y;
    b = pointA.x - point0.x;
    angle = atan(a / b);
    return angle;
}

int main(int argc, char *argv[])
{

    /* (phi, delta_phi) */
    // Mat processNoise(2, 1, CV_32F);
    // Mat measurement = Mat::zeros(1, 1, CV_32F);
    // // kal for test//
    // create object
    // ini
    // set A
    // set H

    if (argc != 2)
        return -1;
    string path = argv[1];
    VideoCapture cap(path);
    //float fps = cap.get(CAP_PROP_FPS); // calculate the fps to get the time between each capture


    //KalmanFilter KF(1, 0.1);
    //KalmanFilterX<2U, 2U> KF(1, 0.1); // TODO
    // Matx<float, 2, 1> x(0, 0);
    // Matx<float, 2, 2> A(1, 1 / fps,
    //                     0, 1);
    // Matx<float, 2, 2> H(1, 0,
    //                     0, 1);
    // KF.transitionMatrix(A);
    // KF.measurementMatrix(H);

    if (!cap.isOpened())
    {
        cout << "failed to open the video!" << endl;
        return -1;
    }
    Mat img, binary, morimg;
    float angle = 0.f, fuangle = 0.f, testangle = 0.f;
    while (cap.read(img))
    {

        // // kal for test//

        // // kal for test //
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
                contourArea(contours[i]) < 180 &&
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
        if (105 < distance && distance < 140)
        {
            line(img, PointDis[0], PointDis[1], Scalar(0, 0, 255), 2);
            angle = getang(PointDis[0], PointDis[1]);
            fuangle = 5 * (angle - testangle) + angle; // calculate the future angle after 5
            testangle = angle;
            while (fuangle > PAI || fuangle < -PAI)
            {
                if (fuangle > PAI)
                    fuangle = fuangle - PAI;
                else
                    fuangle = fuangle + PAI;
            }

            // float fuangle1 = fuangle, angle1 = angle;
            putText(img, "The distance is " + to_string(distance), Point(1, 20), FONT_HERSHEY_COMPLEX, 0.75, Scalar(255, 255, 255));
            putText(img, "The angle is " + to_string(angle), Point(1, 40), FONT_HERSHEY_COMPLEX, 0.75, Scalar(255, 255, 255));
            putText(img, "The future angle is " + to_string(fuangle), Point(1, 60), FONT_HERSHEY_COMPLEX, 0.75, Scalar(255, 255, 255));
            // Show predicted line
            Matx21f raw_vec = {PointDis[1].x - PointDis[0].x,
                               PointDis[1].y - PointDis[0].y};
            Matx22f rot_mat = {cosf(fuangle - angle), -sinf(fuangle - angle),
                               sinf(fuangle - angle), cosf(fuangle - angle)};
            Matx21f vec_after_rot = rot_mat * raw_vec;

            line(img, PointDis[0], Point2f(PointDis[0].x + vec_after_rot(0), PointDis[0].y + vec_after_rot(1)),
                 Scalar(0, 0, 255), 2);
            // This line is without KalmanFilter

            // x = {angle, (angle - testangle) * fps};
            // KF.predict();
            // Matx<float, 2, 1> x_hat = KF.correct(x);
            // Matx<float, 2, 2> A = {1, 5 / fps,
            //                        0, 1};
            // Matx<float, 2, 1> x_predict = A * x_hat;
            // cout << x_predict << endl;
            // x_predict(0);
            // rot_mat = {cosf(x_predict(0)), -sinf(x_predict(0)),
            //                    sinf(x_predict(0)), cosf(x_predict(0))};
            //  vec_after_rot = rot_mat * raw_vec;
            // line(img, PointDis[0], Point2f(PointDis[0].x + vec_after_rot(0), PointDis[0].y + vec_after_rot(1)),
            //      Scalar(0, 255, 255), 2);
            // double measAngle = measurement.at<float>(0);

            // // float a=KalmanFilterX.correct();

            imshow("test2", img);
            // imshow("test3", binary);
            // imshow("test4", morimg);
            if (waitKey(80) == 27) // Esc
                if (waitKey(0) == 27)
                    break;
        }
    }
}
