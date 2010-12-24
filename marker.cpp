#define CV_NO_BACKWARD_COMPATIBILITY

#include "cv.h"
#include "highgui.h"

#include <iostream>
#include <cstdio>
#include <cmath>
#include <set>

using namespace std;
using namespace cv;

const double PI = 3.14159265358979323846;
const double GAUSSIAN_DERIVATIVE[5] = {-3 * .0625, -5 * .0625, 0, 5 * .0625, 3 * .0625};
const double EDGE_THRESHOLD = 20;
const double ANGLE_THRESHOLD = PI / 20;
const double LINE_DISTANCE_THRESHOLD = .2;
const double VOTES_THRESHOLD = 7;
const double SEGMENT_ANGLE_THRESHOLD = PI / 20;
const double SEGMENT_MERGE_LENGTH_SQ_THRESHOLD = 10000;
const double SEGMENT_LENGTH_SQ_THRESHOLD = 10000;
const double CORNER_ANGLE_THRESHOLD = PI / 100;
const double CORNER_SEGMENT_DISTANCE_THRESHOLD = 100;

namespace vpj {
 template<class T>
 class Point {
  public:
   T x, y;

   Point(const Point &p) { x = p.x, y = p.y; }

   Point(T x, T y) { Point::x = x; Point::y = y; }

   Point() { x = y = 0; }

   /* You'll have to calculate atan2 with respect to a midpoint */
   double atan2() const { return std::atan2((double)y, (double)x); }

   Point<T> operator + (const Point<T> &p) const { return Point<T>(x + p.x, y + p.y); }
   void operator += (const Point<T> &p) { x += p.x, y += p.y; }

   Point<T> operator - (const Point<T> &p) const { return Point<T>(x - p.x, y - p.y); }
   void operator -= (const Point<T> &p) { x -= p.x, y -= p.y; }

   T operator ^ (const Point<T> &p) const { return x * p.x + y * p.y; } // dot product

   T operator * (const Point<T> &p) const { return x * p.y - y * p.x; } // cross product

   /* You'll have to calculate atan2 with respect to a midpoint */
//   bool operator < (const Point<T> &p)  const { return (Point::atan2() < p.atan2()); }

   bool operator < (const Point<T> &p)  const { return (y < p.y ? true : ((y == p.y && x < p.x) ? true : false)); }
   bool operator > (const Point<T> &p)  const { return (y > p.y ? true : ((y == p.y && x > p.x) ? true : false)); }
   bool operator == (const Point<T> &p)  const { return (y == p.y && x == p.x); }
   bool operator <= (const Point<T> &p)  const { return (y < p.y ? true : ((y == p.y && x <= p.x) ? true : false)); }
   bool operator >= (const Point<T> &p)  const { return (y > p.y ? true : ((y == p.y && x >= p.x) ? true : false)); }

   operator Point<double>() { return Point<double>(x, y); };

   Point<double> operator / (const double s) const { return Point<double>(x / s, y / s); }

   double mag() const { return sqrt((double)x * x + (double)y * y); }
   double dis(const Point<T> &p) const {return (Point<T>(x - p.x, y - p.y)).mag();}

   T mag2() const { return x * x + y * y; }
   T dis2(const Point<T> &p) const {return (Point<T>(x - p.x, y - p.y)).mag2();}

   // Rotate by ang (in radians) anti-clockwise
   Point<double> rotate(double ang) const {
    return Point<double>(x * cos(ang) - y * sin(ang), x * sin(ang) + y * cos(ang));
   }
 };


 template<class T>
 class Line { //ax + by = c
  public:
   T a, b, c;

   Line(const Line &l) { a = l.a, b = l.b, c = l.c; }

   Line(T a, T b, T c) { Line::a = a, Line::b = b, Line::c = c; }

   Line() { a = b = c = 0; }

   Line(const Point<T> &p1, const Point<T> &p2) {
    a = p2.y - p1.y;
    b = p1.x - p2.x;
    c = a * p1.x + b * p1.y;
   }

   Line(const Point<T> &p) {
    a = p.y;
    b = -p.x;
    c = 0;
   }

   bool operator == (const Line<T> &l) const { return (a * l.c == l.a * c && b * l.c == l.b * c); }

   bool onLine(const Point<T> &p) const { return (a * p.x + b * p.y == c); }

   /* Give the parallel line going through Point p */
   Line<T> para(const Point<T> &p) const {return Line<T>(a, b, a * p.x + b * p.y); }

   /* Give the perpendicular line going through Point p */
   Line<T> perp(const Point<T> &p) const { return Line<T>(-b, a, -b * p.x + a * p.y); }

   bool isParallel(const Line<T> &l) const { return (a * l.b == l.a * b); }

   Point<double> intersection(const Line<T> &l) const {
    return Point<double>(((double)l.b * c - b * l.c), ((double)a * l.c - l.a * c)) / ((double)a * l.b - l.a * b);
   }
 };

 template<class T>
 class Matrix
 {
 private:
  vector< vector<T> > a;

 public:
  int R, C;

  Matrix(const Matrix<T> &m)
  {
   R = m.R, C = m.C;

   a = vector< vector<T> >(R, vector<T>(C, 0));

   for(int r = 0; r < R; r++)
    for(int c = 0; c < C; c++)
     a[r][c] = m[r][c];
  }

  Matrix(int _R, int _C)
  {
   R = _R, C = _C;

   a = vector< vector<T> >(R, vector<T>(C, 0));
  }

  vector<T>& operator [](int r)
  {
   return a[r];
  }

  const vector<T>& operator [](int r) const
  {
   return a[r];
  }
 };
}

struct Edgel {
 int x, y;
 double gx, gy;

 Edgel(int r, int c, int gy, int gx) {
  this->y = r;
  this->x = c;
  this->gy = gy;
  this->gx = gx;
 }

 double angle() {
  return atan2(gy, gx) + PI / 2;
 }

 bool operator <(const Edgel &a) const {
  if(this->y < a.y) return true;
  if(this->y > a.y) return false;
  if(this->x < a.x) return true;
  return false;
 }
};

vpj::Matrix<double> LX(3, 3);
vpj::Matrix<double> LY(3, 3);

struct Segment {
 int x1, y1, x2, y2;

 Segment (int x1, int y1, int x2, int y2) {
  this->x1 = x1;
  this->y1 = y1;
  this->x2 = x2;
  this->y2 = y2;
 }

 Segment(vpj::Point<int> a, vpj::Point<int> b) {
  this->x1 = a.x;
  this->y1 = a.y;
  this->x2 = b.x;
  this->y2 = b.y;
 }

 double direction() {
  double d = atan2(y2 - y1,  x2 - x1);
  if(d < 0)
   d += PI;
  return d;
 }

 double length2() {
  return (y1 - y2) * (y1 -  y2) + (x1 - x2) * (x1 - x2);
 }

 double angle() {
  return atan2(double(y2 - y1), double(x2 - x1));
 }
};

struct Corner {
 vpj::Point<double> corner;
 vpj::Point<double> a, b;
 vpj::Point<double> _a, _b;

 Corner (Segment i, Segment j) {
  vpj::Point<double> a(i.x1, i.y1), b(i.x2, i.y2);
  vpj::Point<double> c(j.x1, j.y1), d(j.x2, j.y2);

  vpj::Line<double> l1(a, b);
  vpj::Line<double> l2(c, d);

  corner = l1.intersection(l2);
  vpj::Point<double> a1 = a - corner,
   b1 = b - corner,
   c1 = c - corner,
   d1 = d - corner;

  if(a1.mag() > b1.mag())
   this->a = a, this->_a = b;
  else
   this->a = b, this->_a = a;

  if(c1.mag() > d1.mag())
   this->b = c, this->_b = d;
  else
   this->b = d, this->_b = c;
 }
};

void init() {
 LX[0][0] = LX[2][0] = -1;
 LX[0][2] = LX[2][2] = 1;
 LX[1][0] = -2;
 LX[1][2] = 2;

 LY[0][0] = LY[0][2] = -1;
 LY[2][0] = LY[2][2] = 1;
 LY[0][1] = -2;
 LY[2][1] = 2;
}

void detectAndDraw( Mat& img,
                   double scale);

int main( int argc, const char** argv )
{
 Mat frame, frameCopy, image;

 double scale = .25;

 image = imread( "marker.jpg", 1 );

 cvNamedWindow( "result", 1 );

 detectAndDraw( image, scale );
 waitKey(0);
 cvDestroyWindow("result");

 return 0;
}

double diff_angle(double a1, double a2) {
 double d = abs(a1 - a2);
 d = min(2 * PI - d, d);
 d = min(PI - d, d);

 return d;
}

double line_point_distance(Edgel &ea, Edgel &eb, Edgel &ep) {
 vpj::Point<double> a(ea.x, ea.y);
 vpj::Point<double> b(eb.x, eb.y);
 vpj::Point<double> p(ep.x, ep.y);

 return abs(((p - a) * (b - a)) / (b - a).mag());
}

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

  if(line_point_distance(edgels[m], edgels[n], edgels[n]) > LINE_DISTANCE_THRESHOLD)
   continue;

  r.push_back(i);
  /* range */
 }

 return r;
}

void Ransac(vector<Edgel> &edgels, vector<Segment> &segments) {
 vector<bool> v(edgels.size());

 srand(time(NULL));

 while(edgels.size() >= VOTES_THRESHOLD) {
  int N = edgels.size();

  int mx_votes = 0;
  int mx_n = -1;
  int mx_m = -1;

  for(int k = 0; k < 25; ++k) {
   int n = rand() % N;
   if(v[n])
    continue;

   int m = rand() % N;
   if(m == n || v[m])
    continue;

   vector<int> votes = get_votes(edgels, m, n);

   if(votes.size() > mx_votes) {
    mx_votes = votes.size();
    mx_n = n;
    mx_m = m;
   }
  }

  if(mx_votes >= VOTES_THRESHOLD) {
   vector<int> votes = get_votes(edgels, mx_m, mx_n);
   Edgel mxe = max(edgels[mx_n], edgels[mx_m]);
   Edgel mne = min(edgels[mx_n], edgels[mx_m]);

   for(int i = 0; i < votes.size(); ++i) {
    mxe = max(mxe, edgels[votes[i]]);
    mne = min(mne, edgels[votes[i]]);
   }

   vector<Edgel> temp = edgels;
   edgels.clear();
   for(int i = 0, j = 0; i < votes.size(); ++i) {
    for(; j < votes[i]; ++j)
     edgels.push_back(edgels[j]);
    ++j;
   }

   Segment s = Segment(mne.x, mne.y, mxe.x, mxe.y);
   if(s.length2() < 10) {
    cout << "two small segments errr" << endl;
    continue;
   }

   segments.push_back(Segment(mne.x, mne.y, mxe.x, mxe.y));
  } else {
   break;
  }
 }
}

double filter(Mat &img, int r, int c, vpj::Matrix<double> &m) {
 double v = 0;

 for(int i = r - m.R / 2; i <= r + m.R / 2; ++i) {
  for(int j = c - m.C / 2; j <= c + m.C / 2; ++j) {
   if(r < 0 || c < 0 || r >= img.rows || c >= img.cols)
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
   Ransac(v, segments);
   edgels.insert(edgels.end(), v.begin(), v.end());
  }
}

pair<Segment, Segment> join_segments(Segment &a, Segment &b) {
 vpj::Point<int> ap[2];
 ap[0] = vpj::Point<int>(a.x1, a.y1);
 ap[1] = vpj::Point<int>(a.x2, a.y2);
 vpj::Point<int> bp[2];
 bp[0] = vpj::Point<int>(b.x1, b.y1);
 bp[1] = vpj::Point<int>(b.x2, b.y2);

 if(ap[0] > ap[1]) swap(ap[0], ap[1]);
 if(bp[0] > bp[1]) swap(bp[0], bp[1]);

 if(ap[0] < bp[0])
  return make_pair(Segment(ap[1], bp[0]), Segment(ap[0], bp[1]));
 else
  return make_pair(Segment(ap[0], bp[1]), Segment(ap[1], bp[0]));
}

Segment merge_segments(Segment &a, Segment &b) {
 vpj::Point<int> p[4];
 p[0] = vpj::Point<int>(a.x1, a.y1);
 p[1] = vpj::Point<int>(a.x2, a.y2);
 p[2] = vpj::Point<int>(b.x1, b.y1);
 p[3] = vpj::Point<int>(b.x2, b.y2);

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

   if(diff_angle(a[i].first, join.second.direction()) > SEGMENT_ANGLE_THRESHOLD)
    continue;

   double l2 = join.first.length2();

   if(l2 > SEGMENT_MERGE_LENGTH_SQ_THRESHOLD)
    continue;

   /* Find gradients along the line joining the two segments */
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
  if(s[i].length2() < SEGMENT_LENGTH_SQ_THRESHOLD)
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
   vpj::Point<double> v1 = c._a - c.corner, v2 = c._b - c.corner;
   if(v1.mag() + v2.mag() > 200) // CORNER_SEGMENT_DISTANCE_THRESHOLD)
    continue;
   vpj::Point<double> v = (v1 + v2);
   double mag = v.mag() / 2;
   v.x /= mag;
   v.y /= mag;
   vpj::Point<double> p = c.corner + v;
   int g = 0;

   for(int k = 0; k < 3; ++k) {
    g += img.at<uchar>(p.y, p.x);
   }
   g /= 3;

   if(g < 80)
    continue;

   r.push_back(Corner(s[i], s[j]));
  }
 }

 return r;
}

void detectAndDraw( Mat& img,
                   double scale)
{
 vector<Edgel> edgels;
 vector<Segment> segments;
 Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

 cvtColor( img, gray, CV_BGR2GRAY );

 resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
 equalizeHist( smallImg, smallImg );

 double t = (double)cvGetTickCount();
 detectEdgels(smallImg, edgels, segments);
 t = (double)cvGetTickCount() - t;
 printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );

 cout << segments.size() << " ==> ";
 vector<Segment> tsegments = merge_segments(segments);
 cout << segments.size() << endl;
 tsegments = merge_segments(tsegments);
 tsegments = merge_segments(tsegments);
 tsegments = remove_small_segments(tsegments);
 vector<Corner> corners = find_corners(smallImg, tsegments);

 for(vector<Segment>::const_iterator r = segments.begin(); r != segments.end(); r++)
 {
     Point a;
     Point b;
     a.x = r->x1 * scale;
     a.y = r->y1 * scale;
     b.x = r->x2 * scale;
     b.y = r->y2 * scale;
     Scalar color = CV_RGB(100, 100, 255);
//     line( img, a, b, color, 2);
 }
 for(vector<Segment>::const_iterator r = tsegments.begin(); r != tsegments.end(); r++)
 {
     Point a;
     Point b;
     a.x = r->x1 * scale;
     a.y = r->y1 * scale;
     b.x = r->x2 * scale;
     b.y = r->y2 * scale;
     Scalar color = CV_RGB(255, 100, 100);
     line( img, a, b, color, 2);
 }

 for(vector<Corner>::const_iterator r = corners.begin(); r != corners.end(); r++)
 {
     Point a;
     Point b;
     Point c;
     a.x = r->a.x * scale;
     a.y = r->a.y * scale;
     b.x = r->b.x * scale;
     b.y = r->b.y * scale;
     c.x = r->corner.x * scale;
     c.y = r->corner.y * scale;
     Scalar color = CV_RGB(100, 255, 100);
     line( img, a, c, color, 2);
     line( img, c, b, color, 2);
 }

 cout << "Segments: " << tsegments.size() << endl;


 for(vector<Edgel>::const_iterator r = edgels.begin(); r != edgels.end(); r++)
 {
     Point center;
     Scalar color = CV_RGB(100, 255, 100);
     center.x = cvRound(r->x*scale);
     center.y = cvRound(r->y*scale);
//     circle( img, center, 1, color, 1);
 }

 cv::imshow( "result", img );
}
