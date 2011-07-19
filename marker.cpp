#define CV_NO_BACKWARD_COMPATIBILITY

#include "cv.h"
#include "highgui.h"
#include "geom.h"
#include "consts.h"
#include "features.h"

#include <iostream>
#include <cstdio>
#include <cmath>
#include <set>

using namespace std;
using namespace cv;

geom::Matrix<double> LX(3, 3);
geom::Matrix<double> LY(3, 3);

/* difference between two angles */
double diff_angle(double a1, double a2) {
 double d = abs(a1 - a2);
 d = min(2 * PI - d, d);
 d = min(PI - d, d);

 return d;
}

/* distance from a line to a point */
double line_point_distance(Edgel &ea, Edgel &eb, Edgel &ep) {
 geom::Point<double> a(ea.x, ea.y);
 geom::Point<double> b(eb.x, eb.y);
 geom::Point<double> p(ep.x, ep.y);

 return abs(((p - a) * (b - a)) / (b - a).mag());
}

void init() {
 /* Gradient Matrices */
 LX[0][0] = LX[2][0] = -1;
 LX[0][2] = LX[2][2] = 1;
 LX[1][0] = -2;
 LX[1][2] = 2;

 LY[0][0] = LY[0][2] = -1;
 LY[2][0] = LY[2][2] = 1;
 LY[0][1] = -2;
 LY[2][1] = 2;
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

/* Find the number of edgels that support the segment joining edges m and n
   Called from RANSAC
   ANGLE_THRESHOLD
   LINE_DISTANCE_THRESHOLD - depends on the SCALE of the image
 */
vector<int> get_votes(vector<Edgel> &edgels, int m, int n) {
 int N = edgels.size();
 vector<int> r;

 double angle = atan2(double(edgels[m].y - edgels[n].y),
   double(edgels[m].x - edgels[n].x));

 if(diff_angle(angle, edgels[n].angle()) > ANGLE_THRESHOLD)
  return r;
 if(diff_angle(angle, edgels[m].angle()) > ANGLE_THRESHOLD)
  return r;

 r.push_back(m);
 r.push_back(n);

 /* If orientation match */
 for(int i = 0; i < N; ++i) {
  if(i == m || i == n)
   continue;

  if(diff_angle(angle, edgels[i].angle()) > ANGLE_THRESHOLD)
   continue;

  if(line_point_distance(edgels[m], edgels[n], edgels[i]) > LINE_DISTANCE_THRESHOLD)
   continue;

  r.push_back(i);
  /* range */
 }

 return r;
}

/* Find segments from edgels */
void Ransac(vector<Edgel> &edgels, vector<Segment> &segments) {
 srand(time(NULL));

 while(edgels.size() >= VOTES_THRESHOLD) {
  int N = edgels.size();

  int mx_votes = 0;
  int mx_n = -1;
  int mx_m = -1;

  /* Pick two random edgels and connect them */
  for(int k = 0; k < 25; ++k) {
   int n = rand() % N;

   int m = rand() % N;
   if(m == n)
    continue;

   vector<int> votes = get_votes(edgels, m, n);

   if(votes.size() > mx_votes) {
    mx_votes = votes.size();
    mx_n = n;
    mx_m = m;
   }
  }

  if(mx_votes >= VOTES_THRESHOLD) {
//   cout << mx_votes << endl;
   vector<int> votes = get_votes(edgels, mx_m, mx_n);

   /* Find the the endpoints of the segment */
   Edgel mxe = max(edgels[mx_n], edgels[mx_m]);
   Edgel mne = min(edgels[mx_n], edgels[mx_m]);

   for(int i = 0; i < votes.size(); ++i) {
    mxe = max(mxe, edgels[votes[i]]);
    mne = min(mne, edgels[votes[i]]);
   }

   /* Remove the edgels along the segment */
   vector<Edgel> temp = edgels;
   edgels.clear();
   for(int i = 0, j = 0; i < votes.size(); ++i) {
    for(; j < votes[i]; ++j) {
     edgels.push_back(temp[j]); // this was edgels instead of temp WTF???
    }
    ++j;
   }

   Segment s = Segment(mne.x, mne.y, mxe.x, mxe.y);

   /* FIXME: test */
//   if(s.length2() < 10) {
//    cout << "two small segments errr" << endl;
//    continue;
//   }

   segments.push_back(Segment(mne.x, mne.y, mxe.x, mxe.y));
  } else {
   break;
  }
 }
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

/* Join segments a and b
   returns [segment between a and b, segment joining a and b]
   */
pair<Segment, Segment> join_segments(Segment &a, Segment &b) {
 geom::Point<int> ap[2];
 ap[0] = geom::Point<int>(a.x1, a.y1);
 ap[1] = geom::Point<int>(a.x2, a.y2);
 geom::Point<int> bp[2];
 bp[0] = geom::Point<int>(b.x1, b.y1);
 bp[1] = geom::Point<int>(b.x2, b.y2);

 if(ap[0] > ap[1]) swap(ap[0], ap[1]);
 if(bp[0] > bp[1]) swap(bp[0], bp[1]);

 /* The directions of the segments does not matter */
 if(ap[0] < bp[0])
  return make_pair(Segment(ap[1], bp[0]), Segment(ap[0], bp[1]));
 else
  return make_pair(Segment(ap[0], bp[1]), Segment(ap[1], bp[0]));
}

Segment merge_segments(Segment &a, Segment &b) {
 geom::Point<int> p[4];
 p[0] = geom::Point<int>(a.x1, a.y1);
 p[1] = geom::Point<int>(a.x2, a.y2);
 p[2] = geom::Point<int>(b.x1, b.y1);
 p[3] = geom::Point<int>(b.x2, b.y2);

 sort(p, p + 4);

 return Segment(p[0], p[3]);
}


vector<Segment> merge_segments(vector<Segment> s) {
 vector<pair<double, int> > a;
 vector<pair<double, pair<int, int> > > js;

 int N = s.size();

 for(int i = 0; i < N; ++i)
  a.push_back(make_pair(s[i].direction(), i));

 sort(a.begin(), a.end());

 for(int i = 0; i < N; ++i) {
  for(int j = i + 1; j != i; j = (j + 1) % N) {
   if(diff_angle(a[i].first, a[j].first) > SEGMENT_ANGLE_THRESHOLD)
    break;

   pair<Segment, Segment> join = join_segments(s[a[i].second], s[a[j].second]);

   if(diff_angle(a[i].first, join.first.direction()) > SEGMENT_ANGLE_THRESHOLD)
    continue;

   double l2 = join.first.length2();

   if(l2 > SEGMENT_MERGE_LENGTH_SQ_THRESHOLD)
    continue;

   /* TODO: Find gradients along the line joining the two segments */
   js.push_back(make_pair(l2, make_pair(a[i].second, a[j].second)));
  }
 }

 sort(js.begin(), js.end());

 cout << "merging ... " << js.size() << endl;

 vector<int> parents(N);
 for(int i = 0; i < N; ++i) parents[i] = i;

 for(int i = 0; i < js.size(); ++i) {
  int n = js[i].second.first;
  int m = js[i].second.second;

  while(parents[n] != n) n = parents[n];
  while(parents[m] != m) m = parents[m];

  int r = parents.size();
  s.push_back(merge_segments(s[n], s[m]));
  parents.push_back(r);
  parents[n] = r;
  parents[m] = r;
 }

 set<int> new_segments;

 for(int i = 0; i < parents.size(); ++i) {
  int n = i;
  while(parents[n] != n) n = parents[n];
  new_segments.insert(n);
 }

 vector<Segment> res;
 for(set<int>::const_iterator it = new_segments.begin(); it != new_segments.end(); ++it) {
  res.push_back(s[*it]);
 }

 return res;
}

vector<Segment> remove_small_segments(vector<Segment> s) {
 vector<Segment> r;
 int N = s.size();

 for(int i = 0; i < N; ++i) {
  if(s[i].length2() < SMALL_SEGMENT_LENGTH_SQ_THRESHOLD)
   continue;
  r.push_back(s[i]);
 }

 return r;
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
 for(vector<Segment>::const_iterator r = segments.begin(); r != segments.end(); r++)
 {
     Point a;
     Point b;
     a.x = r->x1 * SCALE;
     a.y = r->y1 * SCALE;
     b.x = r->x2 * SCALE;
     b.y = r->y2 * SCALE;
     Scalar color = CV_RGB(100, 150, 255);
//     line( img, a, b, color, 2);
 }

 cout << segments.size() << " ==> ";
 vector<Segment> tsegments = merge_segments(segments);
 cout << tsegments.size() << endl;
 //tsegments = merge_segments(tsegments);
 tsegments = remove_small_segments(tsegments);

 vector<Corner> corners = find_corners(smallImg, tsegments);

 for(vector<Segment>::const_iterator r = tsegments.begin(); r != tsegments.end(); r++)
 {
     Point a;
     Point b;
     a.x = r->x1 * SCALE;
     a.y = r->y1 * SCALE;
     b.x = r->x2 * SCALE;
     b.y = r->y2 * SCALE;
     Scalar color = CV_RGB(255, 100, 100);
     line( img, a, b, color, 2);
 }

 for(vector<Corner>::const_iterator r = corners.begin(); r != corners.end(); r++)
 {
     Point a;
     Point b;
     Point c;
     a.x = r->a.x * SCALE;
     a.y = r->a.y * SCALE;
     b.x = r->b.x * SCALE;
     b.y = r->b.y * SCALE;
     c.x = r->corner.x * SCALE;
     c.y = r->corner.y * SCALE;
     Scalar color = CV_RGB(100, 255, 100);
     line( img, a, c, color, 2);
     line( img, c, b, color, 2);
 }

 cout << "Segments: " << tsegments.size() << endl;


 for(vector<Edgel>::const_iterator r = edgels.begin(); r != edgels.end(); r++)
 {
     Point center;
     Scalar color = CV_RGB(100, 255, 100);
     center.x = cvRound(r->x*SCALE);
     center.y = cvRound(r->y*SCALE);
//     circle( img, center, 1, color, 1);
 }
 cv::imshow( "result", img );
}
