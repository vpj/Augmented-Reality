#include <iostream>
#include <vector>
#include <set>
#include <cmath>

using namespace std;

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

