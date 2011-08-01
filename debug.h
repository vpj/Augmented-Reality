#ifndef DEBUG_H
#define DEBUG_H

void drawSegments(vector<Segment> &s, Mat &img, Scalar color) {
 for(vector<Segment>::const_iterator r = s.begin(); r != s.end(); r++)
 {
     Point a;
     Point b;
     a.x = r->x1 * SCALE;
     a.y = r->y1 * SCALE;
     b.x = r->x2 * SCALE;
     b.y = r->y2 * SCALE;
     line( img, a, b, color, 2);
 }
}

void drawCorners(vector<Corner> &c, Mat &img, Scalar color) {
 for(vector<Corner>::const_iterator r = c.begin(); r != c.end(); r++)
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
     line( img, a, c, color, 2);
     line( img, c, b, color, 2);
 }
}

void drawEdgels(vector<Edgel> &e, Mat &img, Scalar color) {
 for(vector<Edgel>::const_iterator r = e.begin(); r != e.end(); r++)
 {
     Point center;
     center.x = cvRound(r->x*SCALE);
     center.y = cvRound(r->y*SCALE);
     circle( img, center, 1, color, 1);
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
