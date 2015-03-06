///////////////////////////////////////////////////////////////////////////
//
// NAME
//  GlobalAlign.h -- global image alignment
//
// SPECIFICATION
//  int initGlobalAlign(const vector<FeatureSet> &fs, int minMatches, MotionModel m, float f, int width, int height, int nRANSAC, double RANSACthresh, AlignMatrix &am, vector<CTransform3x3> &ms);
//  int bundleAdjust(const vector<FeatureSet> &fs, MotionModel m, float f, int width, int height, const AlignMatrix &am, vector<CTransform3x3> &ms, int bundleAdjustIters);
//
// PARAMETERS
//  fs                  vector of feature sets
//  minMatches          minimum matches for overlapping images
//  m                   motion model
//  f                   focal length
//  width               image width
//  height              image height
//  nRANSAC             number of RANSAC iterations
//  RANSACthresh        RANSAC distance threshold
//  am                  matrix of pairwise alignments
//  ms                  vector of transformations (output)
//  bundleAdjustIters   number of bundle adjustment iterations
//
// DESCRIPTION
//  These routines compute the global alignment of a set of images.
//  The initGlobalAlign routine aligns each image relative to an image
//  whose global alignment has already been finalized, beginning with
//  the first image.  The bundleAdjust routine begins with the
//  transformations output by initGlobalAlign, and performs bundle
//  adjustment, computing the optimal alignment of all images
//  simultaneously.
//
// SEE ALSO
//  GlobalAlign.cpp    implementation
//
///////////////////////////////////////////////////////////////////////////

#include "FeatureAlign.h"

struct AlignmentImage {
	int imageID;     // image id
	int parentID;    // parent image id
	bool added;      // added to the active image set
	int nBest;       // number of inliers from best neighbor
	CTransform3x3 r; // global transformation
	int heapIndex;   // index in the image heap
};

struct AlignmentPair {
	vector<FeatureMatch> matches; // set of feature matches
	vector<int> inliers;          // set of inlier match indices
	CTransform3x3 r;              // local transformation
};

typedef vector< vector<AlignmentPair> > AlignMatrix;

class ImageHeap {
private:
	// pointers to the images in the set
	vector<AlignmentImage*> images;

public:
	// Initialize the image heap.
	ImageHeap(vector<AlignmentImage> &baseImages);

	// Return a pointer to the remaining image with the best match,
	// and remove it from the heap.
	AlignmentImage* extractMax();

	// Increase the match quality of an image
	void increaseKey(int index, int nBest);
};

// Initialize global alignment of all images.
int initGlobalAlign(const vector<FeatureSet> &fs, int minMatches, MotionModel m, float f, int width, int height, int nRANSAC, double RANSACthresh, AlignMatrix &am, vector<CTransform3x3> &ms);

// Use bundle adjustment to compute global alignment.
int bundleAdjust(const vector<FeatureSet> &fs, MotionModel m, float f, int width, int height, const AlignMatrix &am, vector<CTransform3x3> &ms, int bundleAdjustIters);

// Fix the rotation matrices to eliminate twist.
void fixRotations(vector<CTransform3x3> &ms);