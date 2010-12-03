#define CV_NO_BACKWARD_COMPATIBILITY

#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <cstdio>


using namespace std;
using namespace cv;

const double _der_gaussian[5] = {-3 * .0625, -5 * .0625, 0, 5 * .0625, 3 * .0625};
const double _threshold = 30;

struct Edgel {
 int x, y;
 int gx, gy;

 Edgel(int r, int c, int gy, int gx) {
  this->y = r;
  this->x = c;
  this->gy = gy;
  this->gx = gx;
 }
};

void detectAndDraw( Mat& img,
                   double scale);

int main( int argc, const char** argv )
{
 Mat frame, frameCopy, image;

 double scale = 1;

 image = imread( "lena.jpg", 1 );

 cvNamedWindow( "result", 1 );

 detectAndDraw( image, scale );
 waitKey(0);
 cvDestroyWindow("result");

 return 0;
}

Edgel getEdgel(Mat &img, int r, int c) {
 return Edgel(r, c, 0, 0);
}

void detectEdgelsVertical(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 for(int j = c; j < C; j += 5) {
  for(int i = r; i < R - 5; ++i) {
   double v = 0;
   for(int k = 0; k < 5; ++k)
    v += img.at<uchar>(i + k, j) * _der_gaussian[k];
   v = abs(v);
   if(v > _threshold)
    edgels.push_back(getEdgel(img, i + 2, j));
  }
 }
}

void detectEdgelsHorizontal(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 for(int i = r; i < R; i += 5) {
  for(int j = c; j < C - 5; ++j) {
   double v = 0;
   for(int k = 0; k < 5; ++k)
    v += img.at<uchar>(i, j + k) * _der_gaussian[k];
   v = abs(v);
   if(v > _threshold)
    edgels.push_back(getEdgel(img, i, j + 2));
  }
 }
}

void detectEdgelsLocal(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 detectEdgelsHorizontal(img, r, c, R, C, edgels);
 detectEdgelsVertical(img, r, c, R, C, edgels);
}

void detectEdgels(Mat &img, vector<Edgel> &edgels)
{
 for(int i = 0; i < img.rows; i += 40)
  for(int j = 0; j < img.cols; j += 40) {
   vector<Edgel> v;
   detectEdgelsLocal(img, i, j, min(i + 40, img.rows), min(j + 40, img.cols), v);
   edgels.insert(edgels.end(), v.begin(), v.end());
  }
}

void detectAndDraw( Mat& img,
                   double scale)
{
 vector<Edgel> edgels;
 const static Scalar colors[] =  { CV_RGB(0,0,255),
     CV_RGB(0,128,255),
     CV_RGB(0,255,255),
     CV_RGB(0,255,0),
     CV_RGB(255,128,0),
     CV_RGB(255,255,0),
     CV_RGB(255,0,0),
     CV_RGB(255,0,255)} ;
 Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

 cvtColor( img, gray, CV_BGR2GRAY );

 resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
 equalizeHist( smallImg, smallImg );

 
 double t = (double)cvGetTickCount();
 detectEdgels(smallImg, edgels);
 t = (double)cvGetTickCount() - t;
 printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );

 int i = 0;
 for(vector<Edgel>::const_iterator r = edgels.begin(); r != edgels.end(); r++, i++)
 {
     Mat smallImgROI;
     Point center;
     Scalar color = colors[i%8];
     int radius = 2;
     center.x = cvRound(r->x*scale);
     center.y = cvRound(r->y*scale);
     circle( img, center, radius, color, 3, 8, 0 );
 }
 cv::imshow( "result", img );
}
