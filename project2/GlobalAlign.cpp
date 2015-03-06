///////////////////////////////////////////////////////////////////////////
//
// NAME
//  GlobalAlign.h -- global image alignment
//
// SEE ALSO
//  GlobalAlign.h      longer description
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "GlobalAlign.h"
#include "P3Math.h"
#include <math.h>
#include <time.h>
//#include "ANN/ANN.h"

float RatioThresh;

// Initialize the image heap.
ImageHeap::ImageHeap(vector<AlignmentImage> &baseImages) {
	int n = baseImages.size(); 
	images.clear();

	for (int i=1; i<n; i++) {
		images.push_back(&(baseImages[i]));
		baseImages[i].heapIndex = i-1;
	}
}

// Return a pointer to the remaining image with the best match,
// and remove it from the heap.
AlignmentImage* ImageHeap::extractMax() {
	AlignmentImage *result = images[0];
	images[0] = images[images.size()-1];
	images[0]->heapIndex = 0;
	images.pop_back();

	int n = images.size();
	int index = 0;
	bool done = false;
	AlignmentImage *temp;

	// this is just an ordinary heapify operation
	while (!done) {
		int leftChild = 2*index + 1;
		int rightChild = 2*index + 2;

		if (leftChild < n) {
			if (rightChild < n) {
				if ((images[leftChild]->nBest > images[index]->nBest) && (images[leftChild]->nBest >= images[rightChild]->nBest)) {
					// swap index with left child
					temp = images[leftChild];
					images[leftChild] = images[index];
					images[index] = temp;
					images[index]->heapIndex = index;
					images[leftChild]->heapIndex = leftChild;
					index = leftChild;
				}
				else if ((images[rightChild]->nBest > images[index]->nBest) && (images[rightChild]->nBest > images[leftChild]->nBest)) {
					// swap index with right child
					temp = images[rightChild];
					images[rightChild] = images[index];
					images[index] = temp;
					images[index]->heapIndex = index;
					images[rightChild]->heapIndex = rightChild;
					index = rightChild;
				}
				else {
					// heap property satisfied
					done = true;
				}
			}
			else {
				// no right child
				if (images[leftChild]->nBest > images[index]->nBest) {
					// swap index with left child
					temp = images[leftChild];
					images[leftChild] = images[index];
					images[index] = temp;
					images[index]->heapIndex = index;
					images[leftChild]->heapIndex = leftChild;
					index = leftChild;
				}
				else {
					done = true;
				}
			}
		}
		else {
			// no children
			done = true;
		}
	}

	return result;
}

// Increase the match quality of an image
void ImageHeap::increaseKey(int index, int nBest) {
	if (images[index]->nBest >= nBest) {
		// the current match quality is greater
		return;
	}

	images[index]->nBest = nBest;

	bool done = false;
	AlignmentImage *temp;

	while (!done) {
		if (index > 0) {
			int parent = (index-1) / 2;

			if (images[index]->nBest > images[parent]->nBest) {
				// swap index with parent
				temp = images[parent];
				images[parent] = images[index];
				images[index] = temp;
				images[index]->heapIndex = index;
				images[parent]->heapIndex = parent;
				index = parent;
			}
			else {
				//  heap property satisfied
				done = true;
			}
		}
		else {
			// no parent
			done = true;
		}
	}
}

double distEuclidean(const vector<double> &v1, const vector<double> &v2) {
	int m = v1.size();
	int n = v2.size();

	if (m != n) {
		// Here's a big number.
		return 1e100;
	}

	double dist = 0;

	for (int i=0; i<m; i++) {
		dist += pow(v1[i]-v2[i], 2);
	}

	// I don't feel like taking the square root.
	return dist;
}

int MatchFeatures(const FeatureSet &f1, const FeatureSet &f2, vector<FeatureMatch> &matches)
{
	//cout<<"ratio between the first and the second best match: "<<Match_threshold_for_ratio<<endl;
	int count = 0;

	int m = f1.size();
	int n = f2.size();

	matches.clear();

	double d;
	double dBest[2];
	int idBest;

	RatioThresh = 0.5;

	FeatureMatch feamatch;
	for (int i=0; i<m; i++) {
		dBest[0] = 1e100; // to store the smallest distance
		dBest[1] = 1e100 + 1; // to store the second smallest distance
		// just initializing large number 
		idBest = 0;

		for (int j=0; j<n; j++) {
			d = distEuclidean(f1[i].data, f2[j].data);

			if (d < dBest[0]) 
			{
				dBest[1] = dBest[0];
				dBest[0] = d;
				idBest = f2[j].id;
			}
			else if (d < dBest[1])
			{
				dBest[1] = d;
			}
		}

		// Distance of a good match should be relatively smaller than the second smallest distance
		if (sqrt(dBest[0] / dBest[1]) < RatioThresh)
		{
			feamatch.id = idBest;
			matches.push_back(feamatch);
			count++;
		}
		else
		{
			feamatch.id = -1;
			matches.push_back(feamatch);
		}
	}

	return count;
}

// Initialize global alignment of all images.
int initGlobalAlign(const vector<FeatureSet> &fs, int minMatches, MotionModel m, float f, int width, int height, int nRANSAC, double RANSACthresh, AlignMatrix &am, vector<CTransform3x3> &ms) {
	int n = fs.size();

	// create the n-by-n alignment matrix
	am.resize(n);
	for (int i=0; i<n; i++)
		am[i].resize(n);

	CTransform3x3 transMat;

	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			if (i != j) {
				printf("matching image %d with image %d, ", i, j);

				// BEGIN TODO
				// write code to fill in the information for am[i][j]
				//
				// you'll need to call your feature matching routine,
				// then your pair alignment routine
				am[i][j].matches.clear();
				int count = MatchFeatures(fs[i], fs[j], am[i][j].matches);
				if(count < minMatches)
				{
					am[i][j].inliers.clear();
					am[i][j].r = transMat;
				}
				else
				{
					alignPair(fs[i],fs[j],am[i][j].matches,m,f,width,height,nRANSAC,RANSACthresh,am[i][j].r,am[i][j].inliers);
				}

				// END TODO

				printf("%d inliers\n", am[i][j].inliers.size());

				if ((int) am[i][j].inliers.size() < minMatches)
					am[i][j].inliers.clear();
			}
		}
	}

	vector<AlignmentImage> nodes(n);
	for (int i=1; i<n; i++) {
		nodes[i].added = false;
		nodes[i].nBest = 0;
		nodes[i].imageID = i;
		nodes[i].parentID = -1;
	}

	// create the image heap
	ImageHeap heap(nodes);

	// add the first image and update the match quality of
	// its neighbors
	nodes[0].added = true;

	for (int j=1; j<n; j++) {
		int nMatches = am[0][j].inliers.size();
		if (nodes[j].nBest < nMatches) {
			heap.increaseKey(nodes[j].heapIndex, nMatches);
			nodes[j].parentID = 0;
		}
	}

	AlignmentImage *nextImage;

	// add the rest of the images
	for (int i=0; i<n-1; i++) {
		nextImage = heap.extractMax();
		
		if (nextImage->nBest == 0) {
			// image set seems to be disconnected
			return -1;
		}

		nextImage->added = true;
		int id = nextImage->imageID;
		int pid = nextImage->parentID;

		// compute the global alignment of the extracted image
		nextImage->r = am[pid][id].r * nodes[pid].r;

		// update the match quality for its neighbor images
		for (int j=0; j<n; j++) {
			if ((id != j) && (!nodes[j].added)) {
				int nMatches = am[id][j].inliers.size();
				if (nodes[j].nBest < nMatches) {
					heap.increaseKey(nodes[j].heapIndex, nMatches);
					nodes[j].parentID = id;
				}
			}
		}
	}

	ms.clear();

	// put the global transformations into the output array
	for (int i=0; i<n; i++) {
		ms.push_back(nodes[i].r);
	}

	return 0;
}

// Use bundle adjustment to compute global alignment from initial
// alignment.
int bundleAdjust(const vector<FeatureSet> &fs, MotionModel m, float f, int width, int height, const AlignMatrix &am, vector<CTransform3x3> &ms, int bundleAdjustIters) {
	int n = ms.size();
	int nEqs = 0;
	double lastTotalError = 0;
	double lambda = 1;
	CVector3 p,q;
	vector<CTransform3x3> msTemp(n);

	// count the total number of inliers
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			nEqs += 3*am[i][j].inliers.size();
		}
	}

	// compute the initial total error
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			int s = am[i][j].inliers.size();
			for (int k=0; k<s; k++) {
				int index1 = am[i][j].inliers[k];
				int index2 = am[i][j].matches[index1].id - 1;

				p[0] = fs[i][index1].x - (width/2.0);
				p[1] = fs[i][index1].y - (height/2.0);
				p[2] = f;

				q[0] = fs[j][index2].x - (width/2.0);
				q[1] = fs[j][index2].y - (height/2.0);
				q[2] = f;

				p = ms[i].Transpose() * p.Normalize();
				q = ms[j].Transpose() * q.Normalize();

				lastTotalError += pow(p[0]-q[0],2);
				lastTotalError += pow(p[1]-q[1],2);
				lastTotalError += pow(p[2]-q[2],2);
			}
		}
	}

	printf("initial error %f\n", f*sqrt(3*lastTotalError/nEqs));

	// create the least squares matrices
	matrix A(nEqs);
	for (int i=0; i<nEqs; i++) {
		A[i].resize(3*n);
		for (int j=0; j<3*n; j++) {
			A[i][j] = 0;
		}
	}
	vector<double> b(nEqs);

	for (int iter=0; iter<bundleAdjustIters; iter++) {
		printf("performing iteration %d of bundle adjustment, ", iter);

		int currentEq = 0;
		double totalError = 0;

		// clear the A matrix
		for (int i=0; i<nEqs; i++)
			for (int j=0; j<3*n; j++)
				A[i][j] = 0;

		// fill in the entries
		for (int i=0; i<n; i++) {
			for (int j=0; j<n; j++) {
				int s = am[i][j].inliers.size();
				for (int k=0; k<s; k++) {
					int index1 = am[i][j].inliers[k]; 
					int index2 = am[i][j].matches[index1].id - 1; 

					int var1 = 3*i;
					int var2 = 3*j;

					// BEGIN TODO
					// fill in the three equations for this match
					//
					// that is, fill in the entries for the rows
					// A[currentEq], A[currentEq+1], and A[currentEq+2]
					//
					// hint: only columns var1, var1+1, var1+2, and
					// columns var2, var2+1, var2+2 will be nonzero
					p[0] = fs[i][index1].x - (width/2.0);
					p[1] = fs[i][index1].y - (height/2.0);
					p[2] = f;

					q[0] = fs[j][index2].x - (width/2.0);
					q[1] = fs[j][index2].y - (height/2.0);
					q[2] = f;

					p = ms[i].Transpose() * p;
					q = ms[j].Transpose() * q;

					A[currentEq][var1]=0;		A[currentEq][var1+1]=-p[2];		A[currentEq][var1+2]=p[1];
					A[currentEq+1][var1]=p[2];	A[currentEq+1][var1+1]=0;		A[currentEq+1][var1+2]=-p[0];
					A[currentEq+2][var1]=-p[1];	A[currentEq+2][var1+1]=p[0];	A[currentEq+2][var1+2]=0;

					A[currentEq][var2]=0;		A[currentEq][var2+1]=q[2];		A[currentEq][var2+2]=-q[1];
					A[currentEq+1][var2]=-q[2];	A[currentEq+1][var2+1]=0;		A[currentEq+1][var2+2]=q[0];
					A[currentEq+2][var2]=q[1];	A[currentEq+2][var2+1]=-q[0];	A[currentEq+2][var2+2]=0;

					for(int tmpn = 0; tmpn < 3; tmpn++)
						b[currentEq+tmpn] = p[tmpn] + q[tmpn];

					// END TODO

					currentEq += 3;
				}
			}
		}

		// solve the system of equations
		vector<double> x;
		least_squares(A,b,x,lambda);

		// update the rotation matrices
		for (int i=0; i<n; i++) {
			double wx = x[3*i+0];
			double wy = x[3*i+1];
			double wz = x[3*i+2];

			// BEGIN TODO
			// update the ith rotation matrix
			// and store it in msTemp[i]
			//
			// you'll need to use the Rodriguez rule from the slides
			CTransform3x3 mm;
			mm[0][0] = 1;
			mm[0][1] = -wz;
			mm[0][2] = wy;

			mm[1][0] = wz;
			mm[1][1] = 1;
			mm[1][2] = -wx;

			mm[2][0] = -wy;
			mm[2][1] = wx;
			mm[2][2] = 1;

			msTemp[i] = mm * ms[i];

			// END TODO
		}

		// compute the new total error
		for (int i=0; i<n; i++) {
			for (int j=0; j<n; j++) {
				int s = am[i][j].inliers.size();
				for (int k=0; k<s; k++) {
					int index1 = am[i][j].inliers[k];
					int index2 = am[i][j].matches[index1].id - 1;

					p[0] = fs[i][index1].x - (width/2.0);
					p[1] = fs[i][index1].y - (height/2.0);
					p[2] = f;

					q[0] = fs[j][index2].x - (width/2.0);
					q[1] = fs[j][index2].y - (height/2.0);
					q[2] = f;

					p = msTemp[i].Transpose() * p.Normalize();
					q = msTemp[j].Transpose() * q.Normalize();

					totalError += pow(p[0]-q[0],2);
					totalError += pow(p[1]-q[1],2);
					totalError += pow(p[2]-q[2],2);
				}
			}
		}

		if (totalError > lastTotalError) {
			// error increased, don't use new transformations
			lambda *= 10;
			printf("error increased to %f, increasing lambda to %f\n", f*sqrt(3*totalError/nEqs), lambda);
		}
		else if (totalError < lastTotalError) {
			// error decreased, use new transformations
			lambda /= 10;
			ms = msTemp;
			lastTotalError = totalError;
			printf("error decreased to %f, decreasing lambda to %f\n", f*sqrt(3*totalError/nEqs), lambda);
		}
		else {
			// error converged, we can stop
			printf("error converged at %f\n", f*sqrt(3*totalError/nEqs));
			break;
		}
	}

	return 0;
}

// Fix the rotation matrices to eliminate twist.
void fixRotations(vector<CTransform3x3> &ms) {
	int n = ms.size();
	CTransform3x3 temp = ms[0].Transpose();

	for (int i=0; i<n; i++) {
		ms[i] = ms[i] * temp;
	}
}