///////////////////////////////////////////////////////////////////////////
//
// NAME
//  FeatureAlign.h -- image registration using feature matching
//
// SEE ALSO
//  FeatureAlign.h      longer description
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "FeatureAlign.h"
#include "P3Math.h"
#include <math.h>
#include <time.h>

/******************* TO DO *********************
 * alignPair:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		nRANSAC: number of RANSAC iterations
 *		RANSACthresh: RANSAC distance threshold
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		repeat for nRANSAC iterations:
 *			choose a minimal set of feature matches
 *			estimate the transformation implied by these matches
 *			count the number of inliers
 *		for the transformation with the maximum number of inliers,
 *		compute the least squares motion estimate using the inliers,
 *		and store it in M
 */
int alignPair(const FeatureSet &f1, const FeatureSet &f2,
			  const vector<FeatureMatch> &matches, MotionModel m,
			  float f, int width, int height,
			  int nRANSAC, double RANSACthresh, CTransform3x3& M, vector<int> &inliersBest)
{
	// BEGIN TODO
	// write this entire method
	// vector<int> bestGroup;
	vector< vector<int> > vInliers;

     for(int i = 0; i < nRANSAC; i++)
     {
		 

		 int thisIdx1 = rand() % matches.size();
		 int thisIdx2 = rand() % matches.size();
		 if (thisIdx1 == thisIdx2) continue;

		 // CTransform3x3 thisTrans;
		 // thisTrans[0][0] = thisTrans[1][1] = thisTrans[2][2] = 1;
		 // thisTrans[0][1] = thisTrans[1][0] = thisTrans[2][0] = thisTrans[2][1] = 0;
		 // thisTrans[0][2] = f2[matches[thisIdx].id2-1].x - f1[matches[thisIdx].id1-1].x;
		 // thisTrans[1][2] = f2[matches[thisIdx].id2-1].y - f1[matches[thisIdx].id1-1].y;
		 vector<int> inliers;
		 inliers.push_back(thisIdx1);
		 inliers.push_back(thisIdx2);

		 CTransform3x3 Mtmp;
		 leastSquaresFit(f1, f2, matches, m, f, width, height, inliers, Mtmp);

		 countInliers(f1, f2, matches, m, f, width, height, Mtmp, RANSACthresh, inliers);

		 vInliers.push_back(inliers);

	 }

	 int imaxSize = -1, maxIdx;
	 for (int i = 0; i < vInliers.size(); ++i)
	 {
	 	if (imaxSize < (int)vInliers[i].size())
	 	{
	 		imaxSize = vInliers[i].size();
	 		maxIdx = i;
	 	}
	 }
	 leastSquaresFit(f1, f2, matches, m, f, width, height, vInliers[maxIdx], M);
	 inliersBest = vInliers[maxIdx];

	// END TODO

	return 0;
}

/******************* TO DO *********************
 * countInliers:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		M: transformation matrix
 *		RANSACthresh: RANSAC distance threshold
 *		inliers: inlier feature IDs
 *	OUTPUT:
 *		transform the features in f1 by M
 *
 *		count the number of features in f1 for which the transformed
 *		feature is within Euclidean distance RANSACthresh of its match
 *		in f2
 *
 *		store these features IDs in inliers
 *
 *		this method should be similar to evaluateMatch from project 1,
 *		except you are comparing each distance to a threshold instead
 *		of averaging them
 */
int countInliers(const FeatureSet &f1, const FeatureSet &f2,
				 const vector<FeatureMatch> &matches, MotionModel m,
				 float f, int width, int height,
				 CTransform3x3 M, double RANSACthresh, vector<int> &inliers)
{
	inliers.clear();
	int count = 0;

	for (unsigned int i=0; i<f1.size(); i++) {
		// BEGIN TODO
		// determine if the ith feature in f1, when transformed by M,
		// is within RANSACthresh of its match in f2 (if one exists)
		//
		// if so, increment count and append i to inliers
		
		CVector3 p,q;
		p[0] = f1[i].x - width / 2.0;	p[1] = f1[i].y - height / 2.0;	p[2] = f;
		q = M * p;
	
		double xNew = q[0] + width / 2.0;
		double yNew = q[1] + height / 2.0;
		int f2pos = matches[i].id - 1;
		double dist = pow(xNew-f2[f2pos].x,2) + pow(yNew-f2[f2pos].y,2);
		if(dist < RANSACthresh * RANSACthresh)
		{
			inliers.push_back(i);
			count++; 
		}

		// END TODO
	}

	return count;
}

/******************* TO DO *********************
 * leastSquaresFit:
 *	INPUT:
 *		f1, f2: source feature sets
 *		matches: correspondences between f1 and f2
 *		m: motion model
 *		f: focal length
 *		width: image width
 *		height: image height
 *		inliers: inlier feature IDs
 *		M: transformation matrix (output)
 *	OUTPUT:
 *		compute the transformation from f1 to f2 using only the inliers
 *		and return it in M
 */
int leastSquaresFit(const FeatureSet &f1, const FeatureSet &f2,
					const vector<FeatureMatch> &matches, MotionModel m,
					float f, int width, int height,
					const vector<int> &inliers, CTransform3x3& M)
{
	// BEGIN TODO
	// write this entire method
	CTransform3x3 mat, tempMat;
	CVector3 v1, v2;
	
	for(size_t i = 0; i < inliers.size(); i++)
	{
		int f1pos = inliers[i];
		int f2pos = matches[f1pos].id - 1;
		v1[0] = f1[f1pos].x-width/2.0;	v1[1] = f1[f1pos].y-height/2.0;	v1[2] = f;
		v2[0] = f2[f2pos].x-width/2.0;	v2[1] = f2[f2pos].y-height/2.0;	v2[2] = f;
		tempMat = v1 * v2;
		mat = mat + tempMat;
	}

	CTransform3x3 u, s, v;
	svd(mat, u, s, v);

	M = v * u.Transpose();

	// END TODO

	return 0;
}
