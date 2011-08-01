#ifndef EDGELS_H
#define EDGELS_H

#include "cv.h"
#include "highgui.h"
#include "geom.h"
#include "consts.h"
#include "features.h"
#include "ransac.h"

using namespace std;
using namespace cv;

geom::Matrix<double> LX(3, 3);
geom::Matrix<double> LY(3, 3);

void init_gradient_matrices() {
 LX[0][0] = LX[2][0] = -1;
 LX[0][2] = LX[2][2] = 1;
 LX[1][0] = -2;
 LX[1][2] = 2;

 LY[0][0] = LY[0][2] = -1;
 LY[2][0] = LY[2][2] = 1;
 LY[0][1] = -2;
 LY[2][1] = 2;
}

/* Apply filter at a specific location */
double filter(Mat &img, int r, int c, geom::Matrix<double> &m) {
 double v = 0;

 for(int i = r - m.R / 2; i <= r + m.R / 2; ++i) {
  for(int j = c - m.C / 2; j <= c + m.C / 2; ++j) {
   if(i < 0 || j < 0 || i >= img.rows || j >= img.cols)
    continue;
   v += img.at<uchar>(i, j) * m[i - (r - m.R / 2)][j - (c - m.C / 2)];
  }
 }

 return v;
}

Edgel getEdgel(Mat &img, int r, int c) {
 double gx = filter(img, r, c, LX);
 double gy = filter(img, r, c, LY);
 return Edgel(r, c, gy, gx);
}

void detectEdgelsVertical(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 for(int j = c; j < C; j += 5) {
  for(int i = r; i < R - 5; ++i) {
   double v = 0;
   for(int k = 0; k < 5; ++k)
    v += img.at<uchar>(i + k, j) * GAUSSIAN_DERIVATIVE[k];
   v = abs(v);
   if(v > EDGE_THRESHOLD)
    edgels.push_back(getEdgel(img, i + 2, j));
  }
 }
}

void detectEdgelsHorizontal(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 for(int i = r; i < R; i += 5) {
  for(int j = c; j < C - 5; ++j) {
   double v = 0;
   for(int k = 0; k < 5; ++k)
    v += img.at<uchar>(i, j + k) * GAUSSIAN_DERIVATIVE[k];
   v = abs(v);
   if(v > EDGE_THRESHOLD)
    edgels.push_back(getEdgel(img, i, j + 2));
  }
 }
}

void detectEdgelsLocal(Mat &img, int r, int c, int R, int C, vector<Edgel> &edgels) {
 detectEdgelsHorizontal(img, r, c, R, C, edgels);
 detectEdgelsVertical(img, r, c, R, C, edgels);
}

void detectEdgels(Mat &img, vector<Edgel> &edgels, vector<Segment> &segments)
{
 for(int i = 0; i < img.rows; i += 40)
  for(int j = 0; j < img.cols; j += 40) {
   vector<Edgel> v;
   detectEdgelsLocal(img, i, j, min(i + 40, img.rows), min(j + 40, img.cols), v);
   edgels.insert(edgels.end(), v.begin(), v.end());
   Ransac(v, segments);
  }
}

#endif
