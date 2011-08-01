#define CV_NO_BACKWARD_COMPATIBILITY

#include "edgels.h"
#include "debug.h"
#include "cv.h"
#include "highgui.h"
#include "geom.h"
#include "consts.h"
#include "features.h"
#include "segments.h"

#include <iostream>
#include <cstdio>
#include <cmath>
#include <set>

using namespace std;
using namespace cv;


void init() {
 init_gradient_matrices();
}

void detectAndDraw(Mat& img);

int main( int argc, const char** argv )
{
 Mat frame, frameCopy, image;
 init();
 image = imread( "marker.jpg", 1 );

 cvNamedWindow( "result", 1 );

 detectAndDraw(image);
 waitKey(0);
 cvDestroyWindow("result");

 return 0;
}


vector<Corner> find_corners(Mat &img, vector<Segment> &s) {
 vector<Corner> r;
 int N = s.size();

 for(int i = 0; i < N; ++i) {
  for(int j = i + 1; j < N; ++j) {
   if(diff_angle(s[i].angle(), s[j].angle()) < CORNER_ANGLE_THRESHOLD)
    continue;

   Corner c(s[i], s[j]);
   geom::Point<double> v1 = c._a - c.corner, v2 = c._b - c.corner;

   if(v1.mag() + v2.mag() > CORNER_SEGMENT_DISTANCE_THRESHOLD)
    continue;

 //TODO: Detect color inside the corner
   double g = 0;

   v1 = c.a - c.corner, v2 = c.b - c.corner;
   v1 = v1 / (v1.mag() / 2);
   v2 = v2 / (v2.mag() / 2);

   geom::Point<double> t1 = v1;
   for(int i = 0; i < 5; ++i) {
    geom::Point<double> t2 = v2;
    for(int j = 0; j < 5; ++j) {
     geom::Point<double> p = c.corner + t1 + t2;
     g += img.at<uchar>(p.y, p.x);
     t2 += v2;
    }
    t1 += v1;
   }

   g /= 25;

   if(g > 80)
    continue;

   r.push_back(c);

   break;
  }
 }

 return r;
}

vector<Box> merge_corners(Mat &img, vector<Corner> &c) {
 int N = c.size();

 vector<int> parents(N);
 for(int i = 0; i < N; ++i) parents[i] = i;

 for(int i = 0; i < N; ++i) {
  for(int j = 0; j < N; ++j) if(i != j) {
   bool found = false;
   for(int k = 0; k < 2 && !found; ++k)
    for(int l = 0; l < 2 && !found; ++l) {
     double ang = (c[i].corner - c[j].corner).atan2();
     double dis = (c[i].corner - c[j].corner).mag();

     if(diff_angle(c[i].angle(k), c[j].angle(l)) < CORNER_MERGE_ANGLE_THRESHOLD &&
        diff_angle(c[i].angle(k), ang) < CORNER_MERGE_ANGLE_THRESHOLD &&
        diff_angle(c[j].angle(l), ang) < CORNER_MERGE_ANGLE_THRESHOLD) {

      double dist = dis - (c[i].length(k) + c[j].length(l));
      if(dist < CORNER_MERGE_DISTANCE_THRESHOLD &&
         dis > (c[i].end(k) - c[j].corner).mag() &&
         dis > (c[j].end(l) - c[i].corner).mag()) {
       int n = i, m = j;
       while(parents[n] != n) n = parents[n];
       while(parents[m] != m) m = parents[m];

       int r = parents.size();
       parents.push_back(r);
       parents[n] = r;
       parents[m] = r;
       found = true;
      }
     }
    }
  }
 }

 map<int, int> boxes;

 for(int i = 0; i < N; ++i) {
  int n = i;
  while(parents[n] != n) n = parents[n];
  parents[i] = n;
  if(!boxes.count(n)) {
   int r = boxes.size();
   boxes[n] = r;
  }
 }
 vector<Box> res(boxes.size());

 for(int i = 0; i < N; ++i) {
  int n = boxes[parents[i]];

  res[n].add_corner(c[i]);
 }

 return res;
}

void detectAndDraw(Mat& img)
{
 vector<Edgel> edgels;
 vector<Segment> segments;
 Mat gray, smallImg( cvRound (img.rows/SCALE), cvRound(img.cols/SCALE), CV_8UC1 );

 cvtColor( img, gray, CV_BGR2GRAY );

 resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
 equalizeHist( smallImg, smallImg );

 double t = (double)cvGetTickCount();
 detectEdgels(smallImg, edgels, segments);
 t = (double)cvGetTickCount() - t;
 printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );

 drawGrid(img, CV_RGB(255, 200, 200));
 //drawEdgels(edgels, img, CV_RGB(100, 255, 100));
 //drawSegments(segments, img, CV_RGB(250, 50, 55), 1);

 cout << segments.size() << " ==> ";
 vector<Segment> tsegments = merge_segments(segments);
 cout << tsegments.size() << endl;
 //tsegments = merge_segments(tsegments);
 //tsegments = remove_small_segments(tsegments);

 vector<Corner> corners = find_corners(smallImg, tsegments);
 vector<Box> boxes = merge_corners(smallImg, corners);

 drawSegments(tsegments, img, CV_RGB(55, 150, 255), 2);
 drawSegments(segments, img, CV_RGB(250, 50, 55), 2);

 //drawCorners(corners, img, CV_RGB(255, 0, 0));
 //drawBoxes(boxes, img);

 cout << "Segments: " << tsegments.size() << endl;


 cv::imshow( "result", img );
}
