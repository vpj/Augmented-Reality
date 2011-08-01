#ifndef DEBUG_H
#define DEBUG_H

void drawGrid(Mat &img, Scalar color) {
 double Y = img.rows, X = img.cols;

 for(int i = 0; i < img.rows; i += 40 * SCALE) {
  Point a, b;
  a.x = 0, b.x = X;
  a.y = b.y = i;
  line(img, a, b, color, 1);
 }

 for(int i = 0; i < img.cols; i += 40 * SCALE) {
  Point a, b;
  a.y = 0, b.y = Y;
  a.x = b.x = i;
  line(img, a, b, color, 1);
 }
}

void drawSegments(vector<Segment> &s, Mat &img, Scalar color, int thickness) {
 for(vector<Segment>::const_iterator r = s.begin(); r != s.end(); r++)
 {
     Point a, b;
     a.x = r->x1 * SCALE;
     a.y = r->y1 * SCALE;
     b.x = r->x2 * SCALE;
     b.y = r->y2 * SCALE;
     line( img, a, b, color, thickness);
 }
}

void drawCorners(vector<Corner> &c, Mat &img, Scalar color) {
 for(vector<Corner>::const_iterator r = c.begin(); r != c.end(); r++)
 {
     Point a, b, c;
     a.x = r->a.x * SCALE;
     a.y = r->a.y * SCALE;
     b.x = r->b.x * SCALE;
     b.y = r->b.y * SCALE;
     c.x = r->corner.x * SCALE;
     c.y = r->corner.y * SCALE;
     line( img, a, c, color, 2);
     line( img, c, b, color, 2);
 }
}

void drawEdgels(vector<Edgel> &e, Mat &img, Scalar color) {
 for(vector<Edgel>::const_iterator r = e.begin(); r != e.end(); r++)
 {
     Point a, b;
     double x = r->gx;
     double y = r->gy;
     double l = sqrt(x * x + y * y);
     x /= 50 * SCALE, y /= 50 * SCALE;
//     double t = x;
//     x = -y;
//     y = t;

     a.x = r->x*SCALE;
     a.y = r->y*SCALE;
     b.x = (r->x + x) * SCALE;
     b.y = (r->y + y) * SCALE;
     line( img, a, b, color, 1);
 }
}

void drawBoxes(vector<Box> &b, Mat &img) {
 for(vector<Box>::const_iterator r = b.begin(); r != b.end(); r++)
 {
  if(r->corners.size() < 2) continue;

  Scalar color = CV_RGB(rand() % 256, rand() % 256, rand() % 256);
  for(vector<Corner>::const_iterator s = r->corners.begin(); s != r->corners.end(); s++) {
   Point center;
   center.x = cvRound(s->corner.x*SCALE);
   center.y = cvRound(s->corner.y*SCALE);
   circle( img, center, r->corners.size() * 2, color, 3);
  }
 }
}
#endif
