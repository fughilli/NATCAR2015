#ifndef BEZIER_H
#define BEZIER_H

typedef struct
{
    float x, y;
} Point;

// lirp = linear interpolation
// pOut = (1 - t)*p1 + t*p2
void lirp(float t, Point *p1, Point *p2, Point *pOut);

// p0 is implicitly 0,0. p1 is the near line camera point, p2 is the far line
// camera point. pOut is the output
void Bezier(float t, Point *p1, Point *p2, Point *pOut);

#endif // BEZIER_H
