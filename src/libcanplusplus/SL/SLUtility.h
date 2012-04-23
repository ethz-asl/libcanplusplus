/* declares important variables and functions in utility.c */


#ifndef __SLutility__
#define __SLutility__

#include "stdio.h"
#include "stdlib.h"

/* byte swapping routines */
#ifdef  alpha
#define BYTESWAP
#endif

#ifdef i386
#define BYTESWAP
#endif

#ifdef i386mac
#define BYTESWAP
#endif

#ifdef i486
#define BYTESWAP
#endif

#ifdef i486xeno
#define BYTESWAP
#endif

#ifdef i586
#define BYTESWAP
#endif

#ifdef x86_64
#define BYTESWAP
#endif

#ifdef x86_64mac
#define BYTESWAP
#endif

#ifdef x86_64xeno
#define BYTESWAP
#endif


#ifndef WORDSWAP

#define MSB(x)	(((x) >> 8) & 0xff)  /*!< most signif byte of 2-byte integer */
#define LSB(x)	((x) & 0xff)	     /*!< least signif byte of 2-byte integer*/
#define MSW(x) (((x) >> 16) & 0xffff)/*!< most signif word of 2-word integer */
#define LSW(x) ((x) & 0xffff) 	     /*!< least signif byte of 2-word integer*/

#define WORDSWAP(x) (MSW(x) | (LSW(x) << 16))

#define LLSB(x)	((x) & 0xff)		/*!< 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)

#define LONGSWAP(x) ((LLSB(x) << 24) |		\
		     (LNLSB(x) << 16)|		\
		     (LNMSB(x) << 8) |		\
		     (LMSB(x)))

#endif


#define MY_STOP     0
#define MY_CONTINUE 1
#define MY_ERROR   -9999

#define TEMPFILE ".temp0.temp"
#define ALT1_TEMPFILE ".temp1.temp"
#define ALT2_TEMPFILE ".temp2.temp"

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define sqr(x)  ((x)*(x))
#define cube(x)  ((x)*(x)*(x))
#define macro_sign(x) ((x) > 0 ? (1) : (x) < (0) ? (-1) : (0))

#ifndef Pi
#define Pi		3.14159265358979323846264
#endif

#ifndef PI
#define PI		3.14159265358979323846264
#endif

#ifndef G
#define G		9.81
#endif

#ifndef E
#define E		2.71828182845905
#endif

#ifdef VX
#define rint round
#endif

#define SINCOS(v,sv,cv)				\
  *(cv)=cos(v);					\
  *(sv)=sqrt(1.- (*(cv))*(*(cv)));		\
  if (fmod(fmod(v,2*PI)+2.*PI,2.*PI) > PI) {    \
    *(sv) *= -1.;				\
  }


typedef double*  Vector;
typedef double** Matrix;
typedef float*   fVector;
typedef float**  fMatrix;
typedef int*     iVector;
typedef int**    iMatrix;
typedef long*    liVector;
typedef long**   liMatrix;

/* additional info for my matrices and vectors */

#define N_MAT_INFO 3
#define NR       0
#define NC       1
#define MAT_TYPE 2

#define IS_FULL  1
#define IS_SYMM  2
#define IS_DIAG  3

/* all combinations of matrices */

/* bugfix -- adsouza (11/16/2001)
   #define FULL_FULL 11
   #define FULL_DIAG 12
   #define FULL_SYMM 13
   #define DIAG_FULL 21
   #define DIAG_DIAG 22
   #define DIAG_SYMM 23
   #define SYMM_FULL 31
   #define SYMM_DIAG 32
   #define SYMM_SYMM 33
   end of old version */

#define FULL_FULL 11
#define FULL_SYMM 12
#define FULL_DIAG 13
#define SYMM_FULL 21
#define SYMM_SYMM 22
#define SYMM_DIAG 23
#define DIAG_FULL 31
#define DIAG_SYMM 32
#define DIAG_DIAG 33
/*end of new version*/

#ifdef __cplusplus
extern "C" {
#endif

  /*  function definitions */

  /* utility to read file names */
  int
  getFile(char *fname);

  /* finds a keyword in a file and positions the file pointer after it */
  int
  find_keyword(FILE *fp, char *name);

  /* a simple beep */
  int      beep(int how_many);

  /* the sign function */
  double   sign(double);

  /* fopens a file and strips away C comments */
  FILE *fopen_strip(char *filename);
  FILE *fopen_strip_comments(char *filename);
  void  remove_temp_file(void);





  /* a save rand() function */
  double   my_ran0(int *idum);

  /* returns random number with gaussian distribution */
  double   my_gasdev(int *idum);

  /* manages input of a (double) value */
  int      get_double(char *comment, double defaultvalue, Vector value);

  /* manages input of a (float) value */
  int      get_float(char *comment, float defaultvalue, float *value);

  /* manages input of a (int) value */
  int      get_int(char *comment, int defaultvalue, int *value);

  /* manages input of a string value */
  int      get_string(char *comment, char *defaultvalue, char *value);

  /* avoids normal atan probems */
  double   atan2_save(double x, double y);

  /* returns a randomly varied value */
  double   uniform(double value, double std);

  /* returns a random number in the given range */
  long     random_number(long int upper, long int lower);

  /* returns TRUE or FALSE with given probability for TRUE */
  int      prob_experiment(double prob);

  /* various calloc functions */
  int   start_calloc_track(int max_track, void ***ptr);
  void  stop_calloc_track(void);
  void  free_calloc_track(void **ptr);
  void *my_calloc(size_t number, size_t length, int flag);
  void *my_calloc_track(size_t number, size_t length);


  /* ============================================================================
   * MATRIX/MATRIX MULTIPLICATION METHODS
   * ============================================================================
   */
  /* multiplies two arbitrary matrices: c=a*b */
  int      mat_mult(Matrix a, Matrix b, Matrix c);
  /* multiply submatrices of a and b: c=a[1:ar,1:ac]*b[1:br,1:bc] */
  int      mat_mult_size(Matrix a, int ar, int ac, Matrix b,
			 int br, int bc, Matrix c);
  /* multiplies a matrix with the transpose of the other: c=a*b' */
  int     mat_mult_normal_transpose(Matrix a, Matrix b, Matrix c);
  int     mat_mult_normal_transpose_size(Matrix a, int ar, int ac,
					 Matrix b, int br, int bc,
					 Matrix c);
  /* multiplies transpose of a matrix with the other: c=a'*b */
  int     mat_mult_transpose_normal(Matrix a, Matrix b, Matrix c);
  int     mat_mult_transpose_normal_size(Matrix a, int ar, int ac,
					 Matrix b, int br, int bc,
					 Matrix c);
  /* multiplies transpose of a matrix with transpose of the other: c=a'*b'*/
  int     mat_mult_transpose_transpose(Matrix a, Matrix b, Matrix c);
  int     mat_mult_transpose_transpose_size(Matrix a, int ar, int ac,
					    Matrix b, int br, int bc,
					    Matrix c);
  /* consistency method (wraps mat_mult()): c=a*b */
  int     mat_mult_normal_normal(Matrix a, Matrix b, Matrix c);
  /* Perform an elementwise matrix multiplication (like matlab's c=a.*b) */
  int     mat_dmult_normal_normal(Matrix a, Matrix b, Matrix c);
  /* Perform an elementwise matrix multiplication: c=a.*b' */
  int     mat_dmult_normal_transpose(Matrix a, Matrix b, Matrix c);
  /* Perform an elementwise matrix multiplication: c=a'.*b */
  int     mat_dmult_transpose_normal(Matrix a, Matrix b, Matrix c);
  /* Perform an elementwise matrix multiplication: c=a'.*b' */
  int     mat_dmult_transpose_transpose(Matrix a, Matrix b, Matrix c);
  /* Wrapper for mat_dmult_normal_normal(): c=a.*b */
  int     mat_dmult(Matrix a, Matrix b, Matrix c);


  /* ============================================================================
   * MATRIX/VECTOR MULTIPLICATION METHODS
   * ============================================================================
   */
  /* multiplies matrix with vector: c=a*b */
  int      mat_vec_mult(Matrix a, Vector b, Vector c);
  /* multiplies submatrix with subvector: c=a[1:ar,1:ac]*b[1:br] */
  int      mat_vec_mult_size(Matrix a, int ar, int ac,
			     Vector b,int br, Vector c);
  /* multiplies vector with matrix: c=a'*b */
  int      vec_mat_mult(Vector a, Matrix b, Vector c);
  int      vec_mat_mult_size(double * a, int ar,  double ** b, int br, int bc, double *c);



  /* sets the vector to the function of the other one */
  int      vec_equal_apply_math(Vector a, double (*func)(double),Vector c);
  int     ivec_equal_apply_math(iVector a, int (*func)(int), iVector c);

  /* ============================================================================
   * MATRIX/VECTOR ADDITION METHODS
   * ============================================================================
   */
  /* adds two arbitrary matrices */
  int      mat_add(Matrix a, Matrix b, Matrix c);
  /* add submatrices of a and b: c=a[1:nr,1:nc]+b[1:nr,1:nc] */
  int      mat_add_size(Matrix a, Matrix b, int nr, int nc, Matrix c);
  /* Add a vector to each row of a matrix */
  int      mat_add_vec_rows(Matrix a, Vector b, Matrix c);
  /* Add a vector to each column of a matrix */
  int      mat_add_vec_columns(Matrix a, Vector b, Matrix c);
  /* adds two arbitrary vectors */
  int      vec_add(Vector a, Vector b, Vector c);
  int      vec_add_size(Vector a, Vector b, int nr, Vector c);
  /* adds scalar to vectors */
  int      vec_add_scalar(Vector a, double scalar, Vector c);
  /* adds a scalar to matrix */
  int      mat_add_scalar(Matrix a, double scalar, Matrix c);

  /* ============================================================================
   * MATRIX/VECTOR SUBTRACTION METHODS
   * ============================================================================
   */
  /* subtractss two arbitrary matrices */
  int      mat_sub(Matrix a, Matrix b, Matrix c);
  /* Subtract a vector from each row of a matrix */
  int      mat_sub_vec_rows(Matrix a, Vector b, Matrix c);
  /* Subtract a vector from each column of a matrix */
  int      mat_sub_vec_columns(Matrix a, Vector b, Matrix c);
  /* subtracts two arbitrary vectors */
  int      vec_sub(Vector a, Vector b, Vector c);
  int      vec_sub_size(Vector a, Vector b, int nr, Vector c);

  /* ============================================================================
   * MATRIX/VECTOR ELEMENT SUM METHODS
   * ============================================================================
   */
  /* sum's up the elements of the vector */
  int      vec_sum(Vector a, double* s);
  /* compute the weighted sum of a vector */
  int      vec_sum_weighted(Vector a, Vector w, double *s);
  /* sums along the rows of a matrix */
  int      mat_sum_rows(Matrix a, Vector b);
  /* compute a weighted sum of the rows of a matrix */
  int      mat_sum_rows_weighted(Matrix a, Vector w, Vector b);
  /* sums along the columns of a matrix */
  int      mat_sum_columns(Matrix a, Vector b);
  /* compute a weighted sum of the columns of a matrix */
  int      mat_sum_columns_weighted(Matrix a, Vector w, Vector b);

  /* the euclidian norm between two vectors */
  double   vec_euc2(Vector a, Vector b);
  double   vec_euc2_size(Vector a, Vector b, int nr);



  /* ============================================================================
   * MATRIX/VECTOR MULTIPLICATION WITH A SCALAR
   * ============================================================================
   */
  /* product of a vector with a scalar value */
  void     vec_mult_scalar(Vector a, double scalar, Vector c);
  void     vec_mult_scalar_size(Vector a, int nr, double scalar, Vector c);
  /* product of a matrix with a scalar value */
  int      mat_mult_scalar(Matrix a, double scalar, Matrix c);

  /* takes the transpose of a matrix */
  int      mat_trans(Matrix a, Matrix c);
  int      mat_trans_size(Matrix a, int nr, int nc, Matrix c);

  /* applies the given function to each matrix element and assign it to c */
  int      mat_equal_apply_math(Matrix a, double (*func)(double), Matrix c);
  int      fmat_equal_apply_math(fMatrix a, float (*func)(float), fMatrix c);
  int      imat_equal_apply_math(iMatrix a, int (*func)(int), iMatrix c);

  /* mahalanobis distance, assuming a positive definite matrix */
  double   mat_mahal(Matrix a, Vector b);
  double   mat_mahal_size(Matrix a, int nr, Vector b);


  /* mahalanobis distance, assuming nothing */
  double   mat_mahal_irr(Vector c, Matrix a, Vector b);

  int      mat_mahal_matrix(Matrix a, Matrix b, Matrix c);
  /* ============================================================================
   * EQUATING ELEMENTS OF VECTOR/MATRIX
   * ============================================================================
   */
  /* zeros a matrix */
  void     mat_zero(Matrix a);
  void     mat_fzero(fMatrix a);
  void     mat_izero(iMatrix a);
  void     mat_zero_size(Matrix a,int nr, int nc);
  /* makes a matrix the identity matrix */
  void     mat_eye(Matrix a);
  /* zeros a vector */
  void     vec_zero(Vector a);
  void     ivec_zero(iVector a);
  void     vec_zero_size(Vector a, int nr);
  /* set matrix = scalar */
  int      mat_equal_scalar(double scalar, Matrix c);
  /* sets the vector to the given scalar */
  int      vec_equal_scalar(double scalar, Vector c);
  /* sets matrix  c=a */
  int      mat_equal(Matrix a, Matrix c);
  int      fmat_equal(fMatrix a, fMatrix c);
  int      imat_equal(iMatrix a, iMatrix c);
  int      mat_equal_size(Matrix a, int nr, int nc, Matrix c);
  /* sets a vector equal another vector: c = a */
  int      vec_equal(Vector a, Vector c);
  int      vec_equal_size(Vector a, int nr, Vector c);

  /* ============================================================================
   * INNER/OUTER PRODUCTS
   * ============================================================================
   */
  /* inner product of two vectors */
  double   vec_mult_inner(Vector a, Vector b);
  double   vec_mult_inner_size(Vector a, Vector b, int nr);
  /* outer product of two 3d vectors */
  int      vec_mult_outer(Vector a, Vector b, Vector c);
  int      vec_mult_outer_size(Vector a, Vector b, int nr, Vector c);

  /* multiplies the transpose with matrix itself */
  int      mat_mult_inner(Matrix a, Matrix c);
  /* multiplies the matrix with its transpose */
  int      mat_mult_outer(Matrix a, Matrix c);
  int      mat_mult_outer_size(Matrix a, int nr, int nc, Matrix c);



  /* check for NaN */
  int      my_isnan(double x);

  /* update max and min values */
  void     maxMin(double x, Vector max_x, Vector min_x);

  /* simple printing functions for matrices and vectors */
  void     print_mat(char *comm, Matrix a);
  void     print_mat_size(char *comm, Matrix a, int nr, int nc);
  void     print_vec(char *comm, Vector a);
  void     print_vec_size(char *comm, Vector a, int nr);
  void     print_ivec(char *comm, iVector a);
  void     print_imat(char *comm, iMatrix a);
  void     fprint_mat(FILE *fp, char *comm, Matrix a);
  void     fprint_imat(FILE *fp, char *comm, iMatrix a);
  void     fprint_mat_size(FILE *fp, char *comm, Matrix a, int nr, int nc);
  void     fprint_vec(FILE *fp, char *comm, Vector a);
  void     fprint_vec_size(FILE *fp, char *comm, Vector a, int nr);
  void     fprint_ivec(FILE *fp, char *comm, iVector a);
  void     fprint_ivec_size(FILE *fp, char *comm, iVector a, int nr);


  /* binary read and write */

  int      fread_mat(FILE *fp, Matrix a);
  int      fread_fmat(FILE *fp, fMatrix a);
  int      fread_imat(FILE *fp, iMatrix a);
  int      fwrite_mat(FILE *fp, Matrix a);
  int      fwrite_fmat(FILE *fp, fMatrix a);
  int      fwrite_imat(FILE *fp, iMatrix a);
  int      fread_vec(FILE *fp, Vector a);
  int      fwrite_vec(FILE *fp, Vector a);
  int      fread_ivec(FILE *fp, iVector a);
  int      fwrite_ivec(FILE *fp, iVector a);


  /* read int/float/double with comment line */
  void read_a_string(int silent, FILE *fd, char *string, char *svalue );
  void read_a_float(int silent, FILE *fd, char *string, float *pvalue );
  void read_a_double(int silent,  FILE *fd, char *string, Vector pvalue );
  void read_an_int(int silent, FILE *fd, char *string, int  *pint );
  void read_a_long(int silent, FILE *fd, char *string, long  *pint );

  /* parameter optimization */

  int parm_opt(Vector a,int n_parm, int n_con, Vector tol, void (*f_dLda)(),
	       void (*f_dMda)(), void (*f_M)(), double (*f_L)(), void (*f_dMdada)(),
	       void (*f_dLdada)(), int use_newton, Vector final_cost, Vector err);

  /* numerical recipes in C functions  */
  iVector  my_ivector(int nl, int nh);
  void     my_free_ivector(iVector vec, int nl, int nh);

  liVector my_livector(int nl, int nh);
  void     my_free_livector(liVector vec, int nl, int nh);

  Vector   my_vector(int nl, int nh);
  void     my_free_vector(Vector vec, int nl, int nh);

  fVector  my_fvector(int nl, int nh);
  void     my_free_fvector(fVector vec, int nl, int nh);

  Matrix   my_matrix(int nrl, int nrh, int ncl, int nch);
  Matrix   my_matrix_symm(int nrl, int nrh, int ncl, int nch);
  Matrix   my_matrix_diag(int nrl, int nrh, int ncl, int nch);
  void     my_free_matrix(Matrix mat, int nrl, int nrh, int ncl, int nch);

  fMatrix  my_fmatrix(int nrl, int nrh, int ncl, int nch);
  void     my_free_fmatrix(fMatrix mat, int nrl, int nrh, int ncl, int nch);

  iMatrix  my_imatrix(int nrl, int nrh, int ncl, int nch);
  iMatrix  my_imatrix_symm(int nrl, int nrh, int ncl, int nch);
  iMatrix  my_imatrix_diag(int nrl, int nrh, int ncl, int nch);
  void     my_free_imatrix(iMatrix mat, int nrl, int nrh, int ncl, int nch);

  void     my_nrerror(char *error_text);
  double   gaussian(double value, double std);

  int      mat_add_shape(Matrix *a, int nr, int nc);
  int      imat_add_shape(iMatrix *a, int nr, int nc);
  int      vec_add_shape(Vector *a, int nr);
  int      ivec_add_shape(iVector *a, int nr);

  /* optimization functions */
  void my_frprmn(double *p,int n, double ftol, int *iter, double *fret,
		 double (*func)(double *), void (*dfunc)(double *, double *));

  void
  my_dfpmin(double *p,int n, double ftol,int *iter,double *fret,
	    double (*func)(double *),void (*dfunc)(double *,double *));

  /* matrix inversion functions */
  int my_inv_ludcmp(Matrix mat, int size, Matrix inv_mat);
  int my_inv_ludcmp_solve(Matrix mat, Vector b_vec, int size, Vector x_vec);
  int my_inv_ludcmp_solve_many(Matrix mat, Matrix b_vec, int size, int
			       n_vec, Matrix x_vec);
  int my_inv_ludcmp_det(Matrix mat, int size, Matrix inv_mat, Vector deter);
  int my_ludcmp_det(Matrix mat, int size, Vector deter);

  /* solve linear equations */
  int my_inv_ldlt(Matrix a_mat, Vector b_vec, int n_dim, Vector x_vec);

  /* cholesky decomposition */
  int my_choldc(Matrix a, int n, double p[]);

  /* distribution functions */
  double my_betai(double a, double b, double x);
  double my_betacf(double a, double b, double x);
  double my_erf(double x);
  double my_gammp(double a, double x);
  void   my_gser(Vector gamser, double a, double x, Vector gln);
  double my_gammln(double xx); /*deprecated -- see gammaln_scalar()*/
  void   my_gcf(Vector gammcf, double a, double x, Vector gln);
  double my_factln(int n);
  double prob_t(double t, double n);
  double prob_f(double f, double mm, double nn);
  double t_value(double aalpha, double n);
  /* Compute the digamma function for a scalar a */
  int     digamma_scalar(double a, double *d);
  /* Compute the digamma function elementwise for a vector a */
  int     digamma_vector(Vector a, Vector d);
  /* Compute the digamma function elementwise for a matrix a */
  int     digamma_matrix(Matrix a, Matrix d);
  /* Compute the gammaln function for a scalar a */
  int     gammaln_scalar(double a, double *g);
  /* Compute the entropy of a gamma distribution */
  int     gamma_entropy(double a, double b, double *e);
  /* Compute the entropy of a Dirichlet distribution */
  int     dirichlet_entropy(Vector a, double* e);

  /* byte swapping */
  void   byteswap(char *ptr, size_t n_bytes);
  double byteswap_double(double var);
  int    byteswap_int(int var);
  float  byteswap_float(float var);

  /* discrete lqr controller */

  int dlqr(Matrix fx, Matrix fu, Matrix lxx, Matrix luu, Matrix dd,
	   int n_states, int n_controls, Matrix p);

  /* LU deomposition */
  int my_ludcmp(Matrix a, int n, int *indx,Vector dd);

  /* SVD decomposition */
  void my_svdcmp(double **a, int m, int n, double w[], double **v);
  void my_svbksb(double **u,double w[],double **v,int m,int n, double b[], double x[]);
  int  correct_command_svd(double **a, double *b, double svdr, double svdt, double *x, int n_dim);


  /* eigenvalue functions */
  int  my_hqr(Matrix a, int n, double wr[], double wi[]);
  void my_elmhes (Matrix a, int n);
  void my_balanc(Matrix a, int n);

  /* multilinear interpolation on a grid */
  int interpolate(Matrix v_in, int n_in, Matrix v_out, int n_out,
		  Vector v_q, Vector out);


  /* lookup table functions */
  int init_lookup_table(int ident,char *name, int n_dim_in, int n_dim_out,
			int n_bins,
			Matrix ranges);
  int add_to_table(int ident,Vector in,Vector out,int silent);
  int lookup_in_table(int ident,Vector in,Vector out,Vector center,
		      int interpol, int silent);
  int write_lookup_table_ascii(int ident);
  int write_lookup_table(int ident);
  int read_lookup_table(int ident,char *fname);
  int read_lookup_table_ascii(int ident,char *fname);
  int get_mean_and_center_bin(int ident,Vector in,Vector mean,
			      Vector center, int silent);



  /* kdtree functions */

#define SPLIT_AT_MEDIAN 1
#define SPLIT_AT_MEAN   2
#define SPLIT_AT_MIDDLE 3

#define KDTREE_EXTRA             1
#define KDTREE_INTERPOLATE       2
#define KDTREE_INTERPOLATE_EXTRA 3

  int add_kdtree_extra(int ident, int part_ID, Vector extra, int n_el);
  int add_kdtree_extra_to_vertex(int ident, int v_index, Vector vec, int n_el);
  int add_kdtree_output_to_vertex(int ident, int v_index, Vector vec);
  int add_to_kdtree(int ident,Vector in,Vector out,int *part_ID, int *n_pts);
  int delete_kdtree_data_from_partition(int ident, int part_ID);
  int delete_kdtree_output_from_vertex(int ident, int v_index);
  int get_kdtree_all_vertices_ptr(int ident, Matrix *ptr, int *n_elements);
  int get_kdtree_extra(int ident, int part_ID, Vector extra, int *n_el);
  int get_kdtree_extra_from_vertex(int ident, int v_index, Vector vec,int *n_el);
  int get_kdtree_leaf_info(int ident,int node_ID,Matrix vertices,
			   int *ind_ar, Vector mean_inputs,
			   Vector mean_outputs, Vector median);
  int get_kdtree_leaf_vertices(int ident, int leafID, Matrix vertices,
			       int *n_v, int *ind_ar);
  int get_kdtree_output_from_vertex(int ident, int v_index, Vector vec);
  int get_kdtree_parm(int ident, int *n_leaves, int *n_nodes);
  int get_kdtree_prediction_status(int ident, int part_ID);
  int init_kdtree(int ident,char *name, int n_dim_in, int n_dim_out,
		  int split_type, Matrix ranges);
  int predict_kdtree_output(int ident,Vector in, int opt, int *n_el,
			    int *partID, Vector out);
  int read_kdtree(int ident,char *fname);
  int set_kdtree_prediction_status(int ident, int part_ID, int status);
  int split_kdtree_partition(int ident,int part_ID,int split, int *larger,
			     int *smaller);
  int write_kdtree(int ident);

#ifdef __cplusplus
}
#endif

#endif
