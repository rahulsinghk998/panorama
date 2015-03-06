#ifndef P3MATH_H
#define P3MATH_H

typedef vector< vector<double> > matrix;

class CTransform3x3;

void svd(const CTransform3x3 &a, CTransform3x3 &u, CTransform3x3 &s, CTransform3x3 &v);

int least_squares(const matrix a, const vector<double> b, vector<double> &x, double lambda = 0);

#endif