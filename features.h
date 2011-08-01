#ifndef FEATURES_H
#define FEATURES_H

#include "geom.h"

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

struct Segment {
 int x1, y1, x2, y2;

 Segment (int x1, int y1, int x2, int y2) {
  this->x1 = x1;
  this->y1 = y1;
  this->x2 = x2;
  this->y2 = y2;
 }

 Segment(geom::Point<int> a, geom::Point<int> b) {
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
 geom::Point<double> corner;
 geom::Point<double> a, b;
 geom::Point<double> _a, _b; // Closer to the corner

 Corner (Segment i, Segment j) {
  geom::Point<double> a(i.x1, i.y1), b(i.x2, i.y2);
  geom::Point<double> c(j.x1, j.y1), d(j.x2, j.y2);

  geom::Line<double> l1(a, b);
  geom::Line<double> l2(c, d);

  corner = l1.intersection(l2);
  geom::Point<double> a1 = a - corner,
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

 double angle(int n) {
  double d;
  if(n == 0)
   d = atan2(this->a.y - this->corner.y,  this->a.x - this->corner.x);
  else
   d = atan2(this->b.y - this->corner.y,  this->b.x - this->corner.x);
  if(d < 0) d += PI;
  return d;
 }

 double length(int n) {
  if(n == 0)
   return (this->a - this->corner).mag();
  else
   return (this->b - this->corner).mag();
 }

 geom::Point<double> end(int n) {
  if(n == 0)
   return this->a;
  else
   return this->b;
 }
};

struct Box {
 vector<Corner> corners;

 Box() {
 }

 void add_corner(Corner c) {
  this->corners.push_back(c);
 }
};
#endif
