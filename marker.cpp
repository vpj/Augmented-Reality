#define CV_NO_BACKWARD_COMPATIBILITY

#include "edgels.h"
#include "debug.h"
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

 //drawSegments(segments, img, CV_RGB(100, 150, 255));

 cout << segments.size() << " ==> ";
 vector<Segment> tsegments = merge_segments(segments);
 cout << tsegments.size() << endl;
 //tsegments = merge_segments(tsegments);
 tsegments = remove_small_segments(tsegments);

 vector<Corner> corners = find_corners(smallImg, tsegments);
 vector<Box> boxes = merge_corners(smallImg, corners);

 //drawSegments(tsegments, img, CV_RGB(255, 100, 100));
 drawCorners(corners, img, CV_RGB(255, 0, 0));
 //drawBoxes(boxes, img);

 cout << "Segments: " << tsegments.size() << endl;

// drawEdgels(edgels, img, CV_RGB(100, 255, 100));

 cv::imshow( "result", img );
}
