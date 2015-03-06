///////////////////////////////////////////////////////////////////////////
//
// NAME
//  Transform.cpp -- coordinate transformation matrices
//
// SEE ALSO
//  Transform.h         longer description
//
// Copyright © Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

#include "Transform.h"
#include <math.h>
#ifndef M_PI 
#define M_PI    3.1415926536
#endif // M_PI

CVector3::CVector3()
{
	// Default constructor (zero)
	for (int i = 1; i < 3; i++)
		m_array[i] = 0;
}

CVector3 CVector3::Normalize() const
{
	// Normalize vector
	CVector3 v0 = *this;
	CVector3 v1;
	double length = 0;

	for (int i=0; i<3; i++)
		length += v0[i]*v0[i];

	length = sqrt(length);

	for (int i=0; i<3; i++)
		v1[i] = v0[i] / length;

	return v1;
}

CVector3 CVector3::operator*(double d) const
{
	// Vector-scalar product
	CVector3 v0 = *this;
	CVector3 v1;

	for (int i=0; i<3; i++)
		v1[i] = v0[i] * d;

	return v1;
}

CTransform3x3 CVector3::operator*(const CVector3 &v) const
{
	// Vector outer product
	CVector3 v0 = *this;
	CTransform3x3 m;

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			m[i][j] = v0[i] * v[j];

	return m;
}

double CVector3::dot(const CVector3 &v) const {
	// Dot product
	CVector3 v0 = *this;
	double d = 0;

	for (int i=0; i<3; i++)
		d += v0[i]*v[i];

	return d;
}

CVector3 CVector3::cross(const CVector3 &v) const {
	// Cross product
	CVector3 v0 = *this;
	CVector3 result;

	result[0] = v0[1]*v[2] - v0[2]*v[1];
	result[1] = v0[2]*v[0] - v0[0]*v[2];
	result[2] = v0[0]*v[1] - v0[1]*v[0];

	return result;
}

CTransform3x3::CTransform3x3()
{
    // Default constructor (identity)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m_array[i][j] = (i == j);
}

CTransform3x3 CTransform3x3::Translation(float tx, float ty)
{
    // Translation matrix
    CTransform3x3 M;
    M[0][2] = tx;
    M[1][2] = ty;
    return M;
}

CTransform3x3 CTransform3x3::Rotation(float degrees)
{
    // Rotation matrix
    CTransform3x3 M;
    double iQuad1 = fmod(fmod(degrees / 90, 4) + 4 + 0.5, 4) - 0.5;
    double iQuad0 = iQuad1 - (int)(iQuad1 + 0.5);
    double rad    = iQuad0 * M_PI / 4.0;
    double c      = (float) cos(rad);
    double s      = (float) sin(rad);
    while (iQuad0 < iQuad1)
    {
        double t = c; c = -s; s = t;
        iQuad0 += 1;
    }
    M[0][0] = c, M[0][1] = -s;
    M[1][0] = s, M[1][1] = c;
    return M;
}

CTransform3x3 CTransform3x3::Transpose(void) const
{
	// Matrix transpose, just swap rows and columns
	CTransform3x3 M0 = *this, M1;

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			M1[i][j] = M0[j][i];

	return M1;
}

CTransform3x3 CTransform3x3::Inverse(void)
{
    // Matrix inverse  (use un-pivoted Gaussian elimination for now)
    const int n = 3;
    CTransform3x3 M0 = *this, M1;

    // Forward elimination
    int i;
    for (i = 0; i < n; i++)
    {
        // Normalize the row
        double row_inv = 1.0 / M0[i][i];
        for (int j = 0; j < n; j++)
        {
            M0[i][j] *= row_inv;
            M1[i][j] *= row_inv;
        }
        M0[i][i] = 1.0;

        // Subtract from lower ones
        for (int k = i+1; k < n; k++)
        {
            double mult = M0[k][i];
            for (int j = 0; j < n; j++)
            {
                M0[k][j] -= mult * M0[i][j];
                M1[k][j] -= mult * M1[i][j];
            }
        }
    }

    // Backward elimination
    for (i = n-1; i > 0; i--)
    {
        // Subtract from upper ones
        for (int k = 0; k < i; k++)
        {
            double mult = M0[k][i];
            for (int j = 0; j < n; j++)
            {
                M0[k][j] -= mult * M0[i][j];
                M1[k][j] -= mult * M1[i][j];
            }
        }
    }
    return M1;
}

CTransform3x3 CTransform3x3::operator*(double d) {
	// Multiply matrix by scalar
	const CTransform3x3 &M = *this;
	CTransform3x3 result;

	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			result[i][j] = M[i][j] * d;

	return result;
}

CVector3 CTransform3x3::operator*(const CVector3& v) const
{
	// Transform vector
	const CTransform3x3 &M = *this;
	CVector3 v2;
	for (int i=0; i < 3; i++)
	{
		double sum = 0.0;
		for (int k = 0; k < 3; k++)
			sum += M[i][k] * v[k];
		v2[i] = sum;
	}
	return v2;
}

CTransform3x3 CTransform3x3::operator*(const CTransform3x3& M1) const
{
    // Matrix multiply
    const int n = 3;
    const CTransform3x3& M0 = *this;
	CTransform3x3 M2;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            double sum = 0.0;
            for (int k = 0; k < n; k++)
                sum += M0[i][k] * M1[k][j];
            M2[i][j] = sum;
        }
    }
    return M2;
}

CTransform3x3 CTransform3x3::operator+(const CTransform3x3& M1) const
{
    // Matrix addition
    const int n = 3;
    const CTransform3x3& M0 = *this;
	CTransform3x3 M2;
    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
			M2[i][j] = M0[i][j] + M1[i][j];
        }
    }
    return M2;
}
