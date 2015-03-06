#include "ImageLib/ImageLib.h"
#include "jacob.h"
#include "P3Math.h"
#include <math.h>

void svd(const CTransform3x3 &a, CTransform3x3 &u, CTransform3x3 &s, CTransform3x3 &v) {
	int nrot;
	double temp;

	// allocate space for the necessary matrices
	double *eigenvalues = Jacobi::vector(1,3);
	double **input_matrix = Jacobi::matrix(1,3,1,3);
	double **eigenvectors = Jacobi::matrix(1,3,1,3);

	// compute ATA and AAT
	CTransform3x3 aat = a * a.Transpose();
	CTransform3x3 ata = a.Transpose() * a;

	// compute U, the eigenvectors of AAT
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			input_matrix[i+1][j+1] = aat[i][j];

	Jacobi::jacobi(input_matrix, 3, eigenvalues, eigenvectors, &nrot);

	// sort the eigenvalues
	for (int i=1; i<=3; i++) {
		for (int j=i+1; j<=3; j++) {
			if (eigenvalues[i] < eigenvalues[j]) {
				temp = eigenvalues[i];
				eigenvalues[i] = eigenvalues[j];
				eigenvalues[j] = temp;

				for (int k=1; k<=3; k++) {
					temp = eigenvectors[k][i];
					eigenvectors[k][i] = eigenvectors[k][j];
					eigenvectors[k][j] = temp;
				}
			}
		}
	}

	// fill in the entries of U
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			u[i][j] = eigenvectors[i+1][j+1];

	// fill in the entries of S
	for (int i=0; i<3; i++) {
		if (eigenvalues[i+1] > 0)
			s[i][i] = sqrt(eigenvalues[i+1]);
		else
			s[i][i] = 0;

		for (int j=i+1; j<3; j++)
			s[i][j] = s[j][i] = 0;
	}

	// compute V from A, S, and U
	v = a.Transpose() * u;

	// normalize vectors for nonzero singular values
	int j;
	for (j=0; j<3; j++) {
		double norm = 0;
		for (int i=0; i<3; i++)
			norm += v[i][j]*v[i][j];
		norm = sqrt(norm);

		if (norm > 1e-6)
			for (int i=0; i<3; i++)
				v[i][j] /= norm;
		else
			break;
	}

	CVector3 vec1,vec2,vec3;

	switch (j) {
	case 0:
		// all singular values are zero, any rotation will suffice
		v = CTransform3x3();
		break;
	case 1:
		// compute second column orthogonal to first
		vec1[0] = v[0][0]; vec1[1] = v[1][0]; vec1[2] = v[2][0];
		if ((vec2[1] != 0) || (vec2[2] != 0)) {
			vec2[0] = 1; vec2[1] = 0; vec2[2] = 0;
		}
		else {
			vec2[0] = 0; vec2[1] = 1; vec2[2] = 0;
		}
		vec3 = (vec1.cross(vec2)).Normalize();
		v[0][1] = vec3[0]; v[1][1] = vec3[1]; v[2][1] = vec3[2];
	case 2:
		// compute last column orthogonal to first two
		vec1[0] = v[0][0]; vec1[1] = v[1][0]; vec1[2] = v[2][0];
		vec2[0] = v[0][1]; vec2[1] = v[1][1]; vec2[2] = v[2][1];
		vec3 = (vec1.cross(vec2)).Normalize();
		v[0][2] = vec3[0]; v[1][2] = vec3[1]; v[2][2] = vec3[2];
	case 3:
		break;
	}

	// free the allocated memory
	Jacobi::free_vector(eigenvalues,1,3);
	Jacobi::free_matrix(input_matrix,1,3,1,3);
	Jacobi::free_matrix(eigenvectors,1,3,1,3);
}

int least_squares(const matrix a, const vector<double> b, vector<double> &x, double lambda) {
	int m = a.size();
	int n = a[0].size();
	int nrot;
	x.resize(n);

	if (b.size() != m) {
		// vector size mismatch
		return -1;
	}

	// allocate space for the necessary matrices
	double *eigenvalues = Jacobi::vector(1,n);
	double **ata = Jacobi::matrix(1,n,1,n);
	double **eigenvectors = Jacobi::matrix(1,n,1,n);

	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			ata[i+1][j+1] = 0;
		}
	}

	// fill in the entries of ata
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			for (int k=0; k<m; k++) {
				ata[i+1][j+1] += a[k][i]*a[k][j];
			}
		}

		// condition the diagonal
		ata[i+1][i+1] *= (1+lambda);
	}

	// solve for the eigenvalues and eigenvectors
	Jacobi::jacobi(ata, n, eigenvalues, eigenvectors, &nrot);

	// create some intermediate matrices for computing inv(ata)*at*b
	matrix ataInv(n);
	for (int i=0; i<n; i++)
		ataInv[i].resize(n);
	vector<double> atb(n);

	// compute inv(ata)
	for (int i=0; i<n; i++) {
		for (int j=0; j<n; j++) {
			ataInv[i][j] = 0;
			for (int k=0; k<n; k++) {
				ataInv[i][j] += eigenvectors[i+1][k+1]*eigenvectors[j+1][k+1]/eigenvalues[k+1];
			}
		}
	}

	// compute at*b
	for (int i=0; i<n; i++) {
		atb[i] = 0;
		for (int j=0; j<m; j++) {
			atb[i] += a[j][i]*b[j];
		}
	}

	// compute the solution x
	for (int i=0; i<n; i++) {
		x[i] = 0;
		for (int j=0; j<n; j++) {
			x[i] += ataInv[i][j]*atb[j];
		}
	}
	
	// free the allocated memory
	Jacobi::free_vector(eigenvalues,1,n+1);
	Jacobi::free_matrix(ata,1,n+1,1,n+1);
	Jacobi::free_matrix(eigenvectors,1,n+1,1,n+1);

	return 0;
}