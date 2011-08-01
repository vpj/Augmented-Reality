#ifndef SEGMENTS_H
#define SEGMENTS_H

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
 vector<double> dir;
 vector<pair<double, pair<int, int> > > js;

 int N = s.size();

 for(int i = 0; i < N; ++i)
  dir.push_back(s[i].direction());

 for(int i = 0; i < N; ++i) {
  for(int j = i + 1; j < N; ++j) {
   if(diff_angle(dir[i], dir[j]) > SEGMENT_ANGLE_THRESHOLD)
    continue;

   pair<Segment, Segment> join = join_segments(s[i], s[j]);

   if(diff_angle(dir[i], join.first.direction()) > SEGMENT_ANGLE_THRESHOLD)
    continue;

   double l2 = join.first.length2();

   if(l2 > SEGMENT_MERGE_LENGTH_SQ_THRESHOLD)
    continue;

   /* TODO: Find gradients along the line joining the two segments */
   js.push_back(make_pair(l2, make_pair(i, j)));
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

#endif
