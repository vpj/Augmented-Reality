#ifndef RANSAC_H
#define RANSAC_H

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

/* distance from a line to a point */
geom::Point<double> point_projection(Edgel &ea, Edgel &eb, Edgel &ep) {
 geom::Point<double> a(ea.x, ea.y);
 geom::Point<double> b(eb.x, eb.y);
 geom::Point<double> p(ep.x, ep.y);

 geom::Point<double> sb = b - a;
 geom::Point<double> sp = p - a;
 sb = sb / sb.mag();

 double d = sp ^ sb;
 double id = 1 / d;
 return a +  sb / id;
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
   geom::Point<double> _np(edgels[n].x, edgels[n].y);
   geom::Point<double> _mp(edgels[m].x, edgels[m].y);

   if((_np - _mp).mag2() < 3)
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
   Edgel emxm = edgels[mx_m];
   Edgel emxn = edgels[mx_n];

   for(int i = 0; i < votes.size(); ++i) {
    mxe = max(mxe, edgels[votes[i]]);
    mne = min(mne, edgels[votes[i]]);
   }

   /* Remove the edgels along the segment */
   sort(votes.begin(), votes.end());
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

   segments.push_back(
     Segment(point_projection(emxn, emxm, mne),
             point_projection(emxn, emxm, mxe)));
  } else {
   break;
  }
 }
}

#endif
