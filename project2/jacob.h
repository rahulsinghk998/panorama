


namespace Jacobi
{
	void nrerror(char error_text[]);

	double *vector(long nl, long nh);
	double **matrix(long nrl, long nrh, long ncl, long nch);
	void free_vector(double *v, long nl, long nh);
	void free_matrix(double **m, long nrl, long nrh, long ncl, long nch);

	void jacobi(double **a, int n, double d[], double **v, int *nrot);
};
