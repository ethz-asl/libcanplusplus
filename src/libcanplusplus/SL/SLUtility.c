/*!
 * @file 	SLUtility.c
 * @author 	Christian Gehring
 * @date 	Feb, 2012
 * @version 1.0
 * @ingroup robotCAN, SharedMemory
 * @brief
 * Copied from utility.c (Stefan Schaal, February 1991-1996)
   program for various C utility functions
  Note: this new version of utilities incorporates a modified Numerical
  Recepies matrix allocation scheme. If the [0] row is not used (in
  either a matrix or a vector), a special row vector is added at [0]
  of the form:

  m[0][0] = number of rows
  m[0][1] = number of colums
  m[0][2] = matrix kind: IS_SYM, IS_DIAG, or IS_FULL

  Vectors have just one entry
  v[0][0]    = number of rows

  In this way, vectors and matrices can be used as normal. If the [0] row
  is free, the additional information is allocated -- otherwise it is
  just a normal NR matrix (vector).

  NOTE: there are a variety of matrix functions *_size which by-pass
  the extra information in a matrix record and cannot check for matrix
  errors. Those function allow compatibility with NR matrices
 */

#define MY_RAND_MAX 32767

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "time.h"
#include "stddef.h"
#include "SLUtility.h"
#ifdef UNIX
#include "sys/stat.h"
#endif
#ifdef VX
#include "sys/stat.h"
#endif

static char tempfile[1000]="";

// variables to track calloc memory
static int track_calloc = FALSE;
static int max_track_calloc = 0;
static void **track_list = NULL;
static unsigned long n_track_calloc=0;
static void fopen_strip_recursive(char *filename, FILE *temp);



/*!*****************************************************************************
 *******************************************************************************
 \note  sign
 \date  02/25/91
 
 \remarks 
 
 calculates the SIGN function
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     expr : argument for SIGN
 return	: SIGN(expr)
 
 ******************************************************************************/
double
sign(double expr)
{
  if (expr > 0)
    return (1.0);
  else
    if (expr < 0)
      return (-1.0);
    else
      return (0.0);
}


/*!*****************************************************************************
 *******************************************************************************
 \note  get_double
 \date  02/25/91
 
 \remarks 
 
 get (double) value, offer default value
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     comment 	 : character comment dislayed with input inquiry
 \param[in]     defaultvalue    : default value dislayed with input inquiry
 \param[out]    value		 : value the user typed in. value = defaultvalue if the
                      user did not type in anything
 
 ******************************************************************************/
int
get_double(char *comment, double defaultvalue, double *value)
{
  int 				i, c, goon;
  char				string[100];

  for (i = 0; i < 100; ++i)
    string[i] = '\0';

  printf("%s [%.5f]: ", comment,defaultvalue);
  goon = TRUE;
  for (i = 0; (c = getchar()) != '\n'; ++i) {
    if (c == ' ')
      goon = FALSE;
    if (goon) {
      string[i] = c;
    }
  }

  if (string[0] == 'q' && i == 1)
    return FALSE;

  if (i > 0)
    sscanf(string, "%lf", &*value);
  else
    *value = defaultvalue;

  return TRUE;


}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_float
 \date  02/25/91
 
 \remarks 
 
 get (float) value, offer default value
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     comment 	 : character comment dislayed with input inquiry
 \param[in]     defaultvalue    : default value dislayed with input inquiry
 \param[out]    value		 : value the user typed in. value = defaultvalue if the
                      user did not type in anything
 
 ******************************************************************************/
int
get_float(char *comment, float defaultvalue, float *value)
{
  int 				i, c, goon;
  char				string[100];

  for (i = 0; i < 100; ++i)
    string[i] = '\0';

  printf("%s [%.5f]: ",comment, defaultvalue);
  goon = TRUE;
  for (i = 0; (c = getchar()) != '\n'; ++i) {
    if (c == ' ')
      goon = FALSE;
    if (goon)
      string[i] = c;
  }

  if (string[0] == 'q' && i == 1)
    return FALSE;

  if (i > 0)
    sscanf(string, "%f", &*value);
  else
    *value = defaultvalue;

  return TRUE;


}

/*!*****************************************************************************
 *******************************************************************************
 \note  get_int
 \date  July 7, 1992
 
 \remarks 
 
 get (int) value, offer default value
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     comment 	 : character comment dislayed with input inquiry
 \param[in]     defaultvalue    : default value dislayed with input inquiry
 \param[out]    value		 : value the user typed in. value = defaultvalue if the
 user did not type in anything
 
 ******************************************************************************/
int
get_int(char *comment, int defaultvalue, int *value)
{
  int 				i, c, goon;
  char				string[100];

  for (i = 0; i < 100; ++i)
    string[i] = '\0';

  printf("%s [%d]: ", comment,defaultvalue);
  goon = TRUE;
  for (i = 0; (c = getchar()) != '\n'; ++i) {
    if (c == ' ')
      goon = FALSE;
    if (goon)
      string[i] = c;
  }
  if (string[0] == 'q' && i == 1)
    return FALSE;

  if (i > 0)
    sscanf(string, "%d", &*value);
  else
    *value = defaultvalue;

  return TRUE;

}
/*!*****************************************************************************
 *******************************************************************************
 \note  get_string
 \date  04/25/92
 
 \remarks 
 
 get string value, offer default value
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     comment 	 : character comment dislayed with input inquiry
 \param[in]     defaultvalue    : default value dislayed with input inquiry
 \param[out]    value		 : value the user typed in. value = defaultvalue if the
 user did not type in anything
 
 ******************************************************************************/
int
get_string(char *comment, char *defaultvalue, char *value)
{
  int 				i, c, goon;
  char				string[100];

  for (i = 0; i < 100; ++i)
    string[i] = '\0';

  printf("%s [%s]: ",comment, defaultvalue);
  goon = TRUE;
  for (i = 0; (c = getchar()) != '\n'; ++i) {
    if (c == ' ')
      goon = FALSE;
    if (goon)
      string[i] = c;
  }

  if (string[0] == 'q' && i == 1)
    return FALSE;

  if (i > 0)
    sscanf(string, "%s", value);
  else
    strcpy(value, defaultvalue);

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  atan2_save
 \date  11/10/91
 
 \remarks 
 
 calculates the atan of y/x by obeying the singularities of this
 function
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     x : denominator for fraction
 \param[in]     y : numerator for fraction
 
 ******************************************************************************/
#define ATAN2_THRES 1.e-6
double
atan2_save(double x, double y)
{

  if (fabs(x) < ATAN2_THRES && fabs(y) < ATAN2_THRES)
    return 0.0;

  if (fabs(x) < ATAN2_THRES) {
    if (y >= 0)
      return (double) (PI / 2.0);
    else
      return (double) (3.0 * PI / 2.0);
  }
  if (fabs(y) < ATAN2_THRES) {
    if (x >= 0)
      return (double) (0.0);
    else
      return (double) (PI);
  }

  return atan2(y,x);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  gaussian
 \date  August 17, 92
 
 \remarks 
 
 randomly adds Gaussian	noise to a given value
 the noise level is interpreted as the 99% cutoff of the gaussian
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     value		: value to which the noise should be added
 \param[in]     std            : the standard deviation of the gaussian noise
 
 ******************************************************************************/
double
gaussian(double value, double std)
     
{

  static int firsttime = TRUE;
  double     temp;
  int 	     seed=1;

  if (std == 0)
    return value;

  /* intitialize the random function with an arbitrary number */

  if (firsttime) {
    firsttime = FALSE;
    seed = -abs(((int) time(NULL)) % MY_RAND_MAX);
  }

  /* get me a value from my_gasdev, 
     which is normal distributed about 0, var=1 */

  temp = my_gasdev(&seed)*std;

  return (value + temp);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  uniform
 \date  11/10/91
 
 \remarks 
 
 randomly adds noise to a given value
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     value		: value to which the noise should be added
 \param[in]     std            : the standard deviation of the noise

 NOTE: the std of a uniform distribution is sqrt(1/3)*|a| where
       a is half the interval of where the data is supposed to come
       from
 
 ******************************************************************************/
double
uniform(double value, double std)
{

  static  int firsttime = TRUE;
  int 	  seed=1;
  double  aux;
  time_t  t;
  double  misc;

  if (std == 0)
    return value;

  /* intitialize the random function with an arbitrary number */

  if (firsttime) {
    firsttime = FALSE;
    seed = -abs(((int) time(NULL)) % MY_RAND_MAX);
  }

  /* the inteval size for the given std must be: */

  aux = sqrt(3.0*sqr(std));

  misc = (my_ran0(&seed)-0.5)/0.5*aux; 

  return value + misc;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  random_number
 \date  06/05/92
 
 \remarks 
 
 generates a random number in the range of the given values. The result
 is a random integer number. This integer has equal probability for the
 entire given interval.
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     upper		: upper bound
 \param[in]     lower		: lower bound
 
 ******************************************************************************/
long
random_number(long upper, long lower)
{
  double	aux1, aux2;
  long		val;

  if (lower > upper) {		/* just a safety check */
    aux1 = upper;
    upper = lower;
    lower = aux1;
  }
  
  aux1 = uniform(0.0,0.2887)+0.5; /* this is a number in [0,1] */
  val  = (long) (aux1*(upper-lower+1)-1.e-10) + lower;

  if (val > upper)
    val = upper;
  if (val < lower)
    val = lower;
  

  return val;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  prob_experiment
 \date  06/05/92
 
 \remarks 
 
 returns TRUE or FALSE with given probability for TRUE
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     prob		: probability for TRUE
 
 ******************************************************************************/
int
prob_experiment(double prob)
{

  int i=1, n;
  double aux;

  aux = uniform(0.0,0.2887)+(prob-0.5);
  if (aux > 0)
    return TRUE;
  else
    return FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  start_calloc_track
 \date  March 2006
 
 \remarks 
 
 stops tracking of calloc pointers
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     max_track	  : max elements to be trackeed
 \param[out]    trackptr         : ptr to array of calloc pointers
 
 returns FALSE is unsuccessful
 
 ******************************************************************************/
int
start_calloc_track(int max_track, void ***ptr)
{

  if (track_calloc) {
    printf("Warning: calloc track list was initated while other track active\n");
  }

  track_list = calloc(max_track+1,sizeof(void *));
  if (track_list == NULL)
    return FALSE;

  track_calloc     = TRUE;
  max_track_calloc = max_track;
  n_track_calloc   = 0;
  track_list[0]    = (void *) 0;
  *ptr             = track_list;

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  stop_calloc_track
 \date  March 2006
 
 \remarks 
 
 initializes tracking of calloc pointers
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 none
 
 ******************************************************************************/
void
stop_calloc_track(void)
{
  track_calloc     = FALSE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  free_calloc_track
 \date  March 2006
 
 \remarks 
 
 frees memory accumulated in the a calloc track list, including the track
 list itself.
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     ptr : pointer to track list
 
 ******************************************************************************/
void
free_calloc_track(void **ptr)
{
  unsigned long i;

  for (i=(unsigned long)ptr[0]; i>=1; --i)
    free(ptr[i]);
  free((void *)ptr);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_calloc
 \date  August 17, 92
 
 \remarks 
 
 does the same as calloc, just includes message handling
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     number		  : how many elements are to be allocated
 \param[in]     length		  : what is the size of the elements
 \param[in]     flag		  : is MY_STOP if failure of calloc shall result in program
 exit, is MY_CONTINUE if program shall go on.
 
 returns NULL like calloc is unsuccessful
 
 ******************************************************************************/
void *
my_calloc(size_t number, size_t length, int flag)

{
  void  *rc;

  rc = my_calloc_track(number,length);
  if (rc == NULL) {

    printf("No more memory in calloc! Increase application size.\n");
    printf("Hit return to continue ...\n");
    getchar();

    if (flag == MY_STOP)
      exit(-1);
  }

  return (rc);

}
/*!*****************************************************************************
 *******************************************************************************
 \note  my_calloc_track
 \date  August 17, 92
 
 \remarks 
 
 does the same as calloc, just keeps track of every pointer that was
 allocated
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     number		  : how many elements are to be allocated
 \param[in]     length		  : what is the size of the elements
 
 returns NULL like calloc is unsuccessful
 
 ******************************************************************************/
void *
my_calloc_track(size_t number, size_t length)

{
  void  *rc;

  rc = calloc(number,length);

  if (rc != NULL && track_calloc) {
    if (n_track_calloc < max_track_calloc) {
      track_list[++n_track_calloc] = rc;
      track_list[0] = (void *) n_track_calloc;
    } else {
      printf("calloc track list is exhausted (n_track_calloc=%li vs. %i=max_track_calloc)\n",
	     n_track_calloc,max_track_calloc);
    }
  }

  return (rc);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult
 \date  August 17, 92
 
 \remarks 
 
 multiplies two arbitrary (compatible) matrices a * b

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult(Matrix a, Matrix b, Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      ar,ac,br,bc;
  int      type;

  ar = a[0][NR];
  ac = a[0][NC];

  br = b[0][NR];
  bc = b[0][NC];

  if (ac != br) {
    printf("Incompatile matrices in >mat_mult<\n");
    return FALSE;
  }

  /* bugfix (adsouza 07/07/2002)
  if (c[0][MAT_TYPE]==IS_DIAG && 
      !(a[0][MAT_TYPE]==IS_DIAG && b[0][MAT_TYPE]==IS_DIAG)) {
    printf("Incompatile matrices in >mat_mult<: need full matric for c\n");
    return FALSE;
  }
  end of old code */
  if ((c[0][MAT_TYPE]==IS_DIAG || c[0][MAT_TYPE]==IS_SYMM) && 
      !(a[0][MAT_TYPE]==IS_DIAG && b[0][MAT_TYPE]==IS_DIAG)) {
    printf("Incompatile matrices in >mat_mult<: need full matric for c\n");
    return FALSE;
  }
  /* end of new code for bugfix */

  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    if (c[0][MAT_TYPE]==IS_DIAG) {
      temp = my_matrix_diag(1,ar,1,bc);
    } else {
      temp = my_matrix(1,ar,1,bc);
    }
  } else {
    temp = c;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:
  
    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  temp[i][j] += a[i][m] * b[m][j];
	}
      }
    }

    break;

  case FULL_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (m <= j) {
	    temp[i][j] += a[i][m] * b[m][j];
	  } else {
	    temp[i][j] += a[i][m] * b[j][m];
	  }
	}
      }
    }

    break;


  case DIAG_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE]==IS_DIAG) {
      for (i=1; i <= ar; ++i) {
	temp[i][i] = a[i][i]*b[i][i];
      }
    } else {
      for (i=1; i <= ar; ++i) {
	for (j=i; j <= bc; ++j){
	  if (i == j) {
	    temp[i][i] = a[i][i]*b[i][i];
	  } else {
	    temp[i][j] = 0.0;
	    if (temp[0][MAT_TYPE]==IS_FULL)
	      temp[j][i] = 0.0;
	  }
	}
      }
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
	if (i!=j)
	  /*temp[j][i] = a[j][j]*b[j][i];*/  /*bugfix (adsouza 07/07/2002)*/
          temp[j][i] = a[j][j]*b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (i<=m) {
	    temp[i][j] += a[i][m] * b[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[m][j];
	  }
	}
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
	if (i!=j)
	  /*temp[j][i] = a[j][i]*b[i][i];*/ /*bugfix (adsouza 07/07/2002)*/
          temp[j][i] = a[i][j]*b[i][i];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j]=temp[j][i]=0.0;
	for (m=1; m <= br; ++m){
	  if (i<=m && m<=j) {
	    temp[i][j] += a[i][m] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[m][j];
	  } else if (i<=m && m > j) {
	    temp[i][j] += a[i][m] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[j][m];
	  } else if (i>m && m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[j][m];
	  }
	}
      }
    }

    break;

    
  default:

    exit(-12);

  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ar,1,bc);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_normal_transpose
 \date  July 05, 2002
 
 \remarks 
 
 multiplies two arbitrary (compatible) matrices a * b' (i.e. a with the
 transpose of b)

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_normal_transpose(Matrix a, Matrix b, Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      ar,ac,br,bc;
  int      type;

  ar = a[0][NR];
  ac = a[0][NC];

  br = b[0][NR];
  bc = b[0][NC];

  if (ac != bc) {
    printf("Incompatile matrices in >mat_mult_normal_transpose<\n");
    return FALSE;
  }

  if ((c[0][MAT_TYPE]==IS_DIAG || c[0][MAT_TYPE]==IS_SYMM) && 
      !(a[0][MAT_TYPE]==IS_DIAG && b[0][MAT_TYPE]==IS_DIAG)) {
    printf("Incompatile matrices in >mat_mult_normal_transpose<: need full matrix for c\n");
    return FALSE;
  }
  
  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    if (c[0][MAT_TYPE]==IS_DIAG) {
      temp = my_matrix_diag(1,ar,1,br);
    } else {
      temp = my_matrix(1,ar,1,br);
    }
  } else {
    temp = c;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:
  
    for (i=1; i <= ar; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j]=0;
	for (m=1; m <= bc; ++m){
	  temp[i][j] += a[i][m] * b[j][m];
	}
      }
    }

    break;

  case FULL_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (m <= j) {
	    temp[i][j] += a[i][m] * b[m][j];
	  } else {
	    temp[i][j] += a[i][m] * b[j][m];
	  }
	}
      }
    }

    break;


  case DIAG_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j] = a[i][i]*b[j][i];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE]==IS_DIAG) {
      for (i=1; i <= ar; ++i) {
	temp[i][i] = a[i][i]*b[i][i];
      }
    } else {
      for (i=1; i <= ar; ++i) {
	for (j=i; j <= bc; ++j){
	  if (i == j) {
	    temp[i][i] = a[i][i]*b[i][i];
	  } else {
	    temp[i][j] = 0.0;
	    if (temp[0][MAT_TYPE]==IS_FULL)
	      temp[j][i] = 0.0;
	  }
	}
      }
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
	if (i!=j)
	  temp[j][i] = a[j][j]*b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j]=0;
	for (m=1; m <= bc; ++m){
	  if (i<=m) {
	    temp[i][j] += a[i][m] * b[j][m];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	  }
	}
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
	if (i!=j)
	  temp[j][i] = a[i][j]*b[i][i];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j]=temp[j][i]=0.0;
	for (m=1; m <= br; ++m){
	  if (i<=m && m<=j) {
	    temp[i][j] += a[i][m] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[m][j];
	  } else if (i<=m && m > j) {
	    temp[i][j] += a[i][m] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[j][m];
	  } else if (i>m && m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[j][m];
	  }
	}
      }
    }

    break;

    
  default:

    exit(-12);

  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ar,1,br);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_normal_transpose_size
 \date  July 05, 2002
 
 \remarks 
 
 multiplies two arbitrary (compatible) matrices a * b' (i.e. a with the
 transpose of b)

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     ar,ac           : number of rows and columns of a
 \param[in]     b		 : matrix b
 \param[in]     br,bc           : number of rows and columns of b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_normal_transpose_size(Matrix a, int ar, int ac, Matrix b, int br, int bc,
			       Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      type;

  if (ac != bc) {
    printf("Incompatile matrices in >mat_mult_normal_transpose<\n");
    return FALSE;
  }

  /* check whether input and output matrices are different */
  if (a == c || b == c) {
    temp = my_matrix(1,ar,1,br);
  } else {
    temp = c;
  }

  for (i=1; i <= ar; ++i) {
    for (j=1; j <= br; ++j){
      temp[i][j]=0;
      for (m=1; m <= bc; ++m){
	temp[i][j] += a[i][m] * b[j][m];
      }
    }
  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ar,1,br);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_transpose_normal
 \date  July 6, 2002
 
 \remarks 
 
 multiplies transpose of a matrix 'a' with the other matrix 'b' (a'*b)

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_transpose_normal(Matrix a, Matrix b, Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      ar,ac,br,bc;
  int      type;

  ar = a[0][NR];
  ac = a[0][NC];

  br = b[0][NR];
  bc = b[0][NC];

  if (ar != br) {
    printf("Incompatile matrices in >mat_mult_transpose_normal<\n");
    return FALSE;
  }

  if ((c[0][MAT_TYPE]==IS_DIAG || c[0][MAT_TYPE]==IS_SYMM) && 
      !(a[0][MAT_TYPE]==IS_DIAG && b[0][MAT_TYPE]==IS_DIAG)) {
    printf("Incompatile matrices in >mat_mult_transpose_normal<: need full matric for c\n");
    return FALSE;
  }
  
  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    if (c[0][MAT_TYPE]==IS_DIAG) {
      temp = my_matrix_diag(1,ac,1,bc);
    } else {
      temp = my_matrix(1,ac,1,bc);
    }
  } else {
    temp = c;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:
  
    for (i=1; i <= ac; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  temp[i][j] += a[m][i] * b[m][j];
	}
      }
    }

    break;

  case FULL_DIAG:

    for (i=1; i <= ac; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[j][i]*b[j][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= ac; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	  }
	}
      }
    }

    break;


  case DIAG_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE]==IS_DIAG) {
      for (i=1; i <= ar; ++i) {
	temp[i][i] = a[i][i]*b[i][i];
      }
    } else {
      for (i=1; i <= ar; ++i) {
	for (j=i; j <= bc; ++j){
	  if (i == j) {
	    temp[i][i] = a[i][i]*b[i][i];
	  } else {
	    temp[i][j] = 0.0;
	    if (temp[0][MAT_TYPE]==IS_FULL)
	      temp[j][i] = 0.0;
	  }
	}
      }
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
	if (i!=j)
	  temp[j][i] = a[j][j]*b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (i<=m) {
	    temp[i][j] += a[i][m] * b[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[m][j];
	  }
	}
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
	if (i!=j)
	  temp[j][i] = a[i][j]*b[i][i];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j]=temp[j][i]=0.0;
	for (m=1; m <= br; ++m){
	  if (i<=m && m<=j) {
	    temp[i][j] += a[i][m] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[m][j];
	  } else if (i<=m && m > j) {
	    temp[i][j] += a[i][m] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[j][m];
	  } else if (i>m && m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[j][m];
	  }
	}
      }
    }

    break;

    
  default:

    exit(-12);

  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ac,1,bc);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_transpose_normal_size
 \date  July 6, 2002
 
 \remarks 
 
 multiplies transpose of a matrix 'a' with the other matrix 'b' (a'*b)

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     ar,ac           : number of rows and columns of a
 \param[in]     b		 : matrix b
 \param[in]     br,bc           : number of rows and columns of b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_transpose_normal_size(Matrix a, int ar, int ac, 
			       Matrix b, int br, int bc,
			       Matrix c)
{
  int      i,j,m;
  Matrix   temp;

  if (ar != br) {
    printf("Incompatile matrices in >mat_mult_transpose_normal_size<\n");
    return FALSE;
  }

  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    temp = my_matrix(1,ac,1,bc);
  } else {
    temp = c;
  }


  for (i=1; i <= ac; ++i) {
    for (j=1; j <= bc; ++j){
      temp[i][j]=0;
      for (m=1; m <= br; ++m){
	temp[i][j] += a[m][i] * b[m][j];
      }
    }
  }

  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ac,1,bc);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_transpose_transpose
 \date  July 06, 2002
 
 \remarks 
 
 multiplies the transpose of two arbitrary matrices a and b (a'*b')

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_transpose_transpose(Matrix a, Matrix b, Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      ar,ac,br,bc;
  int      type;

  ar = a[0][NR];
  ac = a[0][NC];

  br = b[0][NR];
  bc = b[0][NC];

  if (ar != bc) {
    printf("Incompatile matrices in >mat_mult_transpose_transpose<\n");
    return FALSE;
  }

  if ((c[0][MAT_TYPE]==IS_DIAG || c[0][MAT_TYPE]==IS_SYMM) && 
      !(a[0][MAT_TYPE]==IS_DIAG && b[0][MAT_TYPE]==IS_DIAG)) {
    printf("Incompatile matrices in >mat_mult_transpose_transpose<: need full matric for c\n");
    return FALSE;
  }
  
  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    if (c[0][MAT_TYPE]==IS_DIAG) {
      temp = my_matrix_diag(1,ac,1,br);
    } else {
      temp = my_matrix(1,ac,1,br);
    }
  } else {
    temp = c;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:
  
    for (i=1; i <= ac; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j]=0;
	for (m=1; m <= bc; ++m){
	  temp[i][j] += a[m][i] * b[j][m];
	}
      }
    }

    break;

  case FULL_DIAG:

    for (i=1; i <= ac; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j] = a[j][i]*b[j][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= ac; ++i) {
      for (j=1; j <= bc; ++j){
	temp[i][j]=0;
	for (m=1; m <= br; ++m){
	  if (m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	  }
	}
      }
    }

    break;


  case DIAG_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j] = a[i][i]*b[j][i];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE]==IS_DIAG) {
      for (i=1; i <= ar; ++i) {
	temp[i][i] = a[i][i]*b[i][i];
      }
    } else {
      for (i=1; i <= ar; ++i) {
	for (j=i; j <= bc; ++j){
	  if (i == j) {
	    temp[i][i] = a[i][i]*b[i][i];
	  } else {
	    temp[i][j] = 0.0;
	    if (temp[0][MAT_TYPE]==IS_FULL)
	      temp[j][i] = 0.0;
	  }
	}
      }
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][i]*b[i][j];
	if (i!=j)
	  temp[j][i] = a[j][j]*b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= ar; ++i) {
      for (j=1; j <= br; ++j){
	temp[i][j]=0;
	for (m=1; m <= bc; ++m){
	  if (i<=m) {
	    temp[i][j] += a[i][m] * b[j][m];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	  }
	}
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j] = a[i][j]*b[j][j];
	if (i!=j)
	  temp[j][i] = a[i][j]*b[i][i];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= bc; ++j){
	temp[i][j]=temp[j][i]=0.0;
	for (m=1; m <= br; ++m){
	  if (i<=m && m<=j) {
	    temp[i][j] += a[i][m] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[m][j];
	  } else if (i<=m && m > j) {
	    temp[i][j] += a[i][m] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[i][m] * a[j][m];
	  } else if (i>m && m <= j) {
	    temp[i][j] += a[m][i] * b[m][j];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[m][j];
	  } else {
	    temp[i][j] += a[m][i] * b[j][m];
	    if (i!=j)
	      temp[j][i] += b[m][i] * a[j][m];
	  }
	}
      }
    }

    break;

    
  default:

    exit(-12);

  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ac,1,br);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_transpose_transpose_size
 \date  July 06, 2002
 
 \remarks 
 
 multiplies the transpose of two arbitrary matrices a and b (a'*b')

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     ar,ac           : number of rows and columns for a
 \param[in]     b		 : matrix b
 \param[in]     br,bc           : number of rows and columns for b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_transpose_transpose_size(Matrix a, int ar, int ac, Matrix b, int br, int bc, Matrix c)
{
  int      i,j,m;
  Matrix   temp;
  int      type;

  if (ar != bc) {
    printf("Incompatile matrices in >mat_mult_transpose_transpose<\n");
    return FALSE;
  }

  
  /* check whether input and output matrices are different */
  if (a == c || b == c) {
    temp = my_matrix(1,ac,1,br);
  } else {
    temp = c;
  }
  
  for (i=1; i <= ac; ++i) {
    for (j=1; j <= br; ++j){
      temp[i][j]=0;
      for (m=1; m <= bc; ++m){
	temp[i][j] += a[m][i] * b[j][m];
      }
    }
  }
  
  if (a == c || b == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,ac,1,br);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_normal_normal
 \date  July 06, 2002
 
 \remarks 
 
 multiplies two arbitrary matrices a and b. This method is only provided for
 naming consistency and simply calls mat_mult() internally.

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_normal_normal(Matrix a, Matrix b, Matrix c){
  return mat_mult(a,b,c);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_dmult_normal_normal
 \date  July 08, 2002
 
 \remarks 
 
 Multiplies two matrices a and b elementwise. Matrices must be of the same
 size. Implements the equivalent of the .* operator in Matlab

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : destination matrix c

 ******************************************************************************/
int
mat_dmult_normal_normal(Matrix a, Matrix b, Matrix c){
  int i,j;
  int ar,ac,br,bc;
  int type;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[0][NR];
  bc = b[0][NC];

  /* Check for equal input matrix sizes */
  if(ar!=br || ac!=bc){
    fprintf(stderr, "Incompatible input matrices in >mat_dot_mult<\n");
    return FALSE;
  }
  /* Check for equal output matrix size */
  if(c[0][NR]!=ar || c[0][NC]!=ac){
    fprintf(stderr, "Incompatible output matrix size in >mat_dot_mult<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a diagonal output matrix */
  if((c[0][MAT_TYPE]==IS_DIAG) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dot_mult<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a symmetric output matrix */
  if((c[0][MAT_TYPE]==IS_SYMM) &&
     (a[0][MAT_TYPE]==IS_FULL || b[0][MAT_TYPE]==IS_FULL) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dot_mult<\n");
    return FALSE;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch(type){
  case FULL_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j]*b[i][j];
    break;

  case FULL_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[i][j];
        c[j][i] = a[j][i]*b[i][j];
      }
    break;

  case SYMM_FULL:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[i][j];
        c[j][i] = a[i][j]*b[j][i];
      }
    break;

  case SYMM_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j]*b[i][j];
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = c[i][j];
    break;

  case FULL_DIAG:
  case SYMM_DIAG:
  case DIAG_FULL:
  case DIAG_SYMM:
  case DIAG_DIAG:
    for(i=1; i<=ar; i++)
      c[i][i] = a[i][i]*b[i][i];
    break;
  default:
    fprintf(stderr,"Unrecognized matrix type in >mat_dot_mult<");
    exit(-12);
  }

  /* 
   * If one of the input matrices was diagonal then zero the remainder
   * of the output matrix
   */
  if(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG){
    if(c[0][MAT_TYPE]<=IS_SYMM)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[i][j] = 0.0;
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = 0.0;
  }
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_dmult_normal_transpose
 \date  July 08, 2002
 
 \remarks 
 
 Multiplies (elementwise) matrix a with the transpose of matrix
 b. Implements the equivalent of the .* operator in Matlab

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : destination matrix c

 ******************************************************************************/
int
mat_dmult_normal_transpose(Matrix a, Matrix b, Matrix c){
  int i,j;
  int ar,ac,br,bc;
  int type;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[0][NR];
  bc = b[0][NC];

  /* Check for equal input matrix sizes */
  if(ar!=bc || ac!=br){
    fprintf(stderr, "Incompatible input matrices in >mat_dmult_normal_transpose<\n");
    return FALSE;
  }
  /* Check for equal output matrix size */
  if(c[0][NR]!=ar || c[0][NC]!=ac){
    fprintf(stderr, "Incompatible output matrix size in >mat_dmult_normal_transpose<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a diagonal output matrix */
  if((c[0][MAT_TYPE]==IS_DIAG) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_normal_transpose<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a symmetric output matrix */
  if((c[0][MAT_TYPE]==IS_SYMM) &&
     (a[0][MAT_TYPE]==IS_FULL || b[0][MAT_TYPE]==IS_FULL) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_normal_transpose<\n");
    return FALSE;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch(type){
  case FULL_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j]*b[j][i];
    break;

  case FULL_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[i][j];
        c[j][i] = a[j][i]*b[i][j];
      }
    break;

  case SYMM_FULL:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[j][i];
        c[j][i] = a[i][j]*b[i][j];
      }
    break;

  case SYMM_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j]*b[i][j];
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = c[i][j];
    break;

  case FULL_DIAG:
  case SYMM_DIAG:
  case DIAG_FULL:
  case DIAG_SYMM:
  case DIAG_DIAG:
    for(i=1; i<=ar; i++)
      c[i][i] = a[i][i]*b[i][i];
    break;
  default:
    fprintf(stderr,"Unrecognized matrix type in >mat_dmult_normal_transpose<");
    exit(-12);
  }

  /* 
   * If one of the input matrices was diagonal then zero the remainder
   * of the output matrix
   */
  if(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG){
    if(c[0][MAT_TYPE]<=IS_SYMM)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[i][j] = 0.0;
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = 0.0;
  }
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_dmult_transpose_normal
 \date  July 08, 2002
 
 \remarks 
 
 Multiplies (elementwise) transpose of matrix a with matrix
 b. Implements the equivalent of the .* operator in Matlab
 
 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : destination matrix c

 ******************************************************************************/
int
mat_dmult_transpose_normal(Matrix a, Matrix b, Matrix c){
  int i,j;
  int ar,ac,br,bc;
  int type;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[0][NR];
  bc = b[0][NC];

  /* Check for equal input matrix sizes */
  if(ac!=br || ar!=bc){
    fprintf(stderr, "Incompatible input matrices in >mat_dmult_transpose_normal<\n");
    return FALSE;
  }
  /* Check for equal output matrix size */
  if(c[0][NR]!=ac || c[0][NC]!=ar){
    fprintf(stderr, "Incompatible output matrix size in >mat_dmult_transpose_normal<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a diagonal output matrix */
  if((c[0][MAT_TYPE]==IS_DIAG) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_transpose_normal<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a symmetric output matrix */
  if((c[0][MAT_TYPE]==IS_SYMM) &&
     (a[0][MAT_TYPE]==IS_FULL || b[0][MAT_TYPE]==IS_FULL) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_transpose_normal<\n");
    return FALSE;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch(type){
  case FULL_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[j][i]*b[i][j];
    break;

  case FULL_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[j][i]*b[i][j];
        c[j][i] = a[i][j]*b[i][j];
      }
    break;

  case SYMM_FULL:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[i][j];
        c[j][i] = a[i][j]*b[j][i];
      }
    break;

  case SYMM_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j]*b[i][j];
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = c[i][j];
    break;

  case FULL_DIAG:
  case SYMM_DIAG:
  case DIAG_FULL:
  case DIAG_SYMM:
  case DIAG_DIAG:
    for(i=1; i<=ar; i++)
      c[i][i] = a[i][i]*b[i][i];
    break;
  default:
    fprintf(stderr,"Unrecognized matrix type in >mat_dmult_transpose_normal<");
    exit(-12);
  }

  /* 
   * If one of the input matrices was diagonal then zero the remainder
   * of the output matrix
   */
  if(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG){
    if(c[0][MAT_TYPE]<=IS_SYMM)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[i][j] = 0.0;
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = 0.0;
  }
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_dmult_transpose_transpose
 \date  July 08, 2002
 
 \remarks 
 
 Multiplies transposes of matrices a and b elementwise. Matrices must
 be of the same size. Implements the equivalent of the .* operator in
 Matlab

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : destination matrix c

 ******************************************************************************/
int
mat_dmult_transpose_transpose(Matrix a, Matrix b, Matrix c){
  int i,j;
  int ar,ac,br,bc;
  int type;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[0][NR];
  bc = b[0][NC];

  /* Check for equal input matrix sizes */
  if(ar!=br || ac!=bc){
    fprintf(stderr, "Incompatible input matrices in >mat_dmult_transpose_transpose<\n");
    return FALSE;
  }
  /* Check for equal output matrix size */
  if(c[0][NR]!=ac || c[0][NC]!=ar){
    fprintf(stderr, "Incompatible output matrix size in >mat_dmult_transpose_transpose<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a diagonal output matrix */
  if((c[0][MAT_TYPE]==IS_DIAG) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_transpose_transpose<\n");
    return FALSE;
  }
  /* Check if we're allowed to have a symmetric output matrix */
  if((c[0][MAT_TYPE]==IS_SYMM) &&
     (a[0][MAT_TYPE]==IS_FULL || b[0][MAT_TYPE]==IS_FULL) &&
     !(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG)){
    fprintf(stderr, "Incompatible output matrix in >mat_dmult_transpose_transpose<\n");
    return FALSE;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch(type){
  case FULL_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[j][i]*b[j][i];
    break;

  case FULL_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[j][i]*b[i][j];
        c[j][i] = a[i][j]*b[i][j];
      }
    break;

  case SYMM_FULL:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++){
        c[i][j] = a[i][j]*b[j][i];
        c[j][i] = a[i][j]*b[i][j];
      }
    break;

  case SYMM_SYMM:
    for(i=1; i<=ar; i++)
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j]*b[i][j];
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = c[i][j];
    break;

  case FULL_DIAG:
  case SYMM_DIAG:
  case DIAG_FULL:
  case DIAG_SYMM:
  case DIAG_DIAG:
    for(i=1; i<=ar; i++)
      c[i][i] = a[i][i]*b[i][i];
    break;
  default:
    fprintf(stderr,"Unrecognized matrix type in >mat_dmult_transpose_transpose<");
    exit(-12);
  }

  /* 
   * If one of the input matrices was diagonal then zero the remainder
   * of the output matrix
   */
  if(a[0][MAT_TYPE]==IS_DIAG || b[0][MAT_TYPE]==IS_DIAG){
    if(c[0][MAT_TYPE]<=IS_SYMM)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[i][j] = 0.0;
    if(c[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=ar; i++)
        for(j=i+1; j<=ac; j++)
          c[j][i] = 0.0;
  }
  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_dmult
 \date  July 08, 2002
 
 \remarks 
 
 Wrapper method for mat_dmult_normal_normal()

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : destination matrix c

 ******************************************************************************/
int
mat_dmult(Matrix a, Matrix b, Matrix c){
  return mat_dmult_normal_normal(a,b,c);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_size
 \date  August 17, 92
 
 \remarks 
 
 multiplies two arbitrary (compatible) matrices a * b

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     ar              : number of rows of a
 \param[in]     ac              : number of columns of a
 \param[in]     b		 : matrix b
 \param[in]     br              : number of rows of b
 \param[in]     bc              : number of columns of b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
mat_mult_size(Matrix a, int ar, int ac, Matrix b, int br, int bc, Matrix c)
{
  int      i,j,m;
  Matrix   temp;

  if (ac != br) {
    printf("Incompatile matrices in >mat_mult<\n");
    return FALSE;
  }

  /* check whether input and output matrices are different */
  
  if (a == c || b == c) {
    temp = my_matrix(1,ar,1,bc);
  } else {
    temp = c;
  }

  for (i=1; i <= ar; ++i) {
    for (j=1; j <= bc; ++j){
      temp[i][j]=0;
      for (m=1; m <= br; ++m){
	temp[i][j] += a[i][m] * b[m][j];
      }
    }
  }
  
  if (a == c || b == c) {
    mat_equal_size(temp,ar,bc,c);
    my_free_matrix(temp,1,ar,1,bc);
  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_vec_mult
 \date  August 17, 92
 
 \remarks 
 
 multiplies a matrix with a vector: a * b, assuming indices
 start at "1".
 Note: The program can also cope with passing the same vector as
 factor and result.
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b
 \param[out]    c		 : result of multipliciation
 
 ******************************************************************************/
int
mat_vec_mult(Matrix a, Vector b, Vector c)
{
  int     i,j,m;
  Vector  temp;
  int     ac,ar,br,cr;

  ac = a[0][NC];
  ar = a[0][NR];
  br = b[NR];
  cr = c[NR];

  if (ac != br) {
    printf("Matrix and vector are incompatible.\n");
    return FALSE;
  }

  /* bugfix (adsouza July 10, 2002)
  if (cr != br) {
    printf("Input and output vector are incompatible.\n");
    return FALSE;
  }
  end of old version */

  if (cr != ar) {
    printf("Input and output vector are incompatible.\n");
    return FALSE;
  }


  if (b == c) {
    temp = my_vector(1,ar);
  } else {
    temp = c;
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i <= ar; ++i) {
      temp[i]=0;
      for (j=1; j <= br; ++j){
	temp[i] += a[i][j] * b[j];
      }
    }

    break;


  case IS_DIAG:

    for (j=1; j <= br; ++j){
      temp[j] = a[j][j] * b[j];
    }

    break;


  case IS_SYMM:

    for (i=1; i <= ar; ++i)
      temp[i]=0;

    for (i=1; i <= ar; ++i) {
      for (j=i; j <= br; ++j){
	temp[i] += a[i][j] * b[j];
	if (i!=j)
	  temp[j] += a[i][j] * b[i];
      }
    }

    break;


  default:
    
    exit(-12);

  }

  if (b == c) {
    vec_equal(temp,c);
    my_free_vector(temp,1,ar);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_vec_mult_size
 \date  August 17, 92
 
 \remarks 
 
 multiplies a matrix with a vector: a * b, assuming indices
 start at "1".
 Note: The program can also cope with passing the same vector as
 factor and result.
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     ar              : number of rows of a
 \param[in]     ac              : number of columns of a
 \param[in]     b		 : vector b
 \param[in]     br              : number of rows of b
 \param[out]    c		 : result of multipliciation
 
 ******************************************************************************/
int
mat_vec_mult_size(Matrix a, int ar, int ac, Vector b, int br, Vector c)
{
  int     i,j,m;
  Vector  temp;
  int     cr;

  cr = br;

  if (ac != br) {
    printf("Matrix and vector are incompatible in mat_vec_mult_size.\n");
    return FALSE;
  }

  if (cr != br) {
    printf("Input and output vector are incompatible in mat_vec_mult_size.\n");
    return FALSE;
  }

  if (b == c) {
    temp = my_vector(1,ar);
  } else {
    temp = c;
  }


  for (i=1; i <= ar; ++i) {
    temp[i]=0;
    for (j=1; j <= br; ++j){
      temp[i] += a[i][j] * b[j];
    }
  }

  if (b == c) {
    vec_equal_size(temp,ar,c);
    my_free_vector(temp,1,ar);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_zero_size
 \date  August 17, 92
 
 \remarks 
 
 zeros a vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     nr              : number of rows of a
 
 ******************************************************************************/
void     
vec_zero_size(Vector a, int nr)
     
{
  int i;

  for (i=1; i<=nr; ++i) a[i] = 0.0;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  ivec_zero
 \date  August 17, 92
 
 \remarks 
 
 zeros a vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
ivec_zero(iVector a)
     
{
  int i;

  for (i=1; i<=a[NR]; ++i) a[i] = 0.0;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_zero
 \date  August 17, 92
 
 \remarks 
 
 zeros a vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
vec_zero(Vector a)
     
{
  int i;

  for (i=1; i<=a[NR]; ++i) a[i] = 0.0;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_zero
 \date  August 17, 92
 
 \remarks 
 
 zeros a matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
mat_zero(Matrix a)
     
{
  int i,j;

  switch ((int)a[0][MAT_TYPE]) {
  case IS_FULL:
    for (i=1; i<=a[0][NR]; ++i)
      for (j=1; j<=a[0][NC]; ++j)
	a[i][j] = 0.0;

    break;

  case IS_DIAG:
    for (i=1; i<=a[0][NR]; ++i)
      a[i][i] = 0.0;

    break;

  case IS_SYMM:
    for (i=1; i<=a[0][NR]; ++i)
      for (j=i; j<=a[0][NC]; ++j)
	a[i][j] = 0.0;

    break;

  default:
    exit(-14);
  }
}

/*!*****************************************************************************
 *******************************************************************************
 \note  fmat_zero
 \date  August 17, 92
 
 \remarks 
 
 zeros a matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
fmat_zero(fMatrix a)
     
{
  int i,j;

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=1; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=a[0][NR]; ++i) {
      a[i][i] = 0.0;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=i; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  default:
    
    exit(-14);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_fzero
 \date  August 17, 92
 
 \remarks 
 
 zeros a matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
mat_fzero(fMatrix a)
     
{
  int i,j;

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=1; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=a[0][NR]; ++i) {
      a[i][i] = 0.0;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=i; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  default:
    
    exit(-14);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_izero
 \date  August 17, 92
 
 \remarks 
 
 zeros a matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
mat_izero(iMatrix a)
     
{
  int i,j;

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=1; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=a[0][NR]; ++i) {
      a[i][i] = 0.0;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=i; j<=a[0][NC]; ++j) {
	a[i][j] = 0.0;
      }
    }

    break;


  default:
    
    exit(-14);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_eye
 \date  August 17, 92
 
 \remarks 
 
 makes a matrix the identity matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 
 ******************************************************************************/
void     
mat_eye(Matrix a)
     
{
  int i,j;

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=1; j<=a[0][NC]; ++j) {
	if (i==j) {
	  a[i][j] = 1.0;
	} else {
	  a[i][j] = 0.0;
	}
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=a[0][NR]; ++i) {
      a[i][i] = 1.0;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=a[0][NR]; ++i) {
      for (j=i; j<=a[0][NC]; ++j) {
	if (i==j) {
	  a[i][j] = 1.0;
	} else {
	  a[i][j] = 0.0;
	}
      }
    }

    break;


  default:
    
    exit(-14);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_zero_size
 \date  August 17, 92
 
 \remarks 
 
 zeros a matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     nr              : number of rows of a
 \param[in]     nc              : number of columns of a
 
 ******************************************************************************/
void     
mat_zero_size(Matrix a, int nr, int nc)
     
{
  int i,j;

  for (i=1; i<=nr; ++i) {
    for (j=1; j<=nc; ++j) {
      a[i][j] = 0.0;
    }
  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_equal
 \date  August 17, 92
 
 \remarks 
 
 set vector c = a
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[out]    c		 : result of assignment
 
 ******************************************************************************/
int
vec_equal(Vector a, Vector c)
     
{
  int i;

  if (a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_equal\n");
    return FALSE;
  }

  for (i=1; i<=a[NR]; ++i) c[i] = a[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_equal_apply_math
 \date  August 17, 92
 
 \remarks 
 
 set vector c = func(a)
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     func            : function to be applied
 \param[out]    c		 : result of assignment
 
 ******************************************************************************/
int
vec_equal_apply_math(Vector a, double (*func)(double), Vector c)
     
{
  int i;

  if (a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_equal\n");
    return FALSE;
  }

  for (i=1; i<=a[NR]; ++i) c[i] = (*func)(a[i]);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  ivec_equal_apply_math
 \date  August 17, 92
 
 \remarks 
 
 set vector c = func(a)
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     func            : function to be applied
 \param[out]    c		 : result of assignment
 
 ******************************************************************************/
int
ivec_equal_apply_math(iVector a, int (*func)(int), iVector c)
     
{
  int i;

  if (a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_equal\n");
    return FALSE;
  }

  for (i=1; i<=a[NR]; ++i) c[i] = (*func)(a[i]);

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_equal_scalar
 \date  August 17, 92
 
 \remarks 
 
 set vector c = scalar
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     scalar		 : scalar value
 \param[out]    c		 : result of assignment
 
 ******************************************************************************/
int
vec_equal_scalar(double scalar, Vector c)
     
{
  int i;

  for (i=1; i<=c[NR]; ++i) c[i] = scalar;

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_equal_size
 \date  August 17, 92
 
 \remarks 
 
 set vector c = a
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     nr              : number of rows in a
 \param[out]    c		 : result of assignment
 
 ******************************************************************************/
int
vec_equal_size(Vector a, int nr, Vector c)
     
{
  int i;

  for (i=1; i<=nr; ++i) c[i] = a[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_sum
 \date  July 10, 2002
 
 \remarks 
 
 Sums up the elements of a vector

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[out]    s		 : sum of the elements of a

 ******************************************************************************/
int
vec_sum(Vector a, double* s){
  int i;

  (*s) = 0.0;
  for(i=1; i<=a[NR]; i++)
    (*s) += a[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_sum_weighted
 \date  July 18, 2002
 
 \remarks 
 
 Computes a weighted sum of the elements of a vector

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     w		 : weight vector w
 \param[out]    s		 : sum of the elements of a

 ******************************************************************************/
int
vec_sum_weighted(Vector a, Vector w, double* s){
  int i;

  if(a[NR]!=w[NR]){
    fprintf(stderr, "ERROR: Incorrect weight vector length in >vec_sum_weighted<\n");
    return FALSE;
  }

  (*s) = 0.0;
  for(i=1; i<=a[NR]; i++)
    (*s) += w[i]*a[i];

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sum_rows
 \date  July 17, 2002
 
 \remarks 
 
 Sums up the row vectors of a matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[out]    b		 : vector containing the summed rows of a

 ******************************************************************************/
int
mat_sum_rows(Matrix a, Vector b){
  int i,j;
  int ar,ac,br;
  int type;

  ar   = a[0][NR];
  ac   = a[0][NC];
  type = a[0][MAT_TYPE];
  br   = b[NR];

  if(ac != br)
    fprintf(stderr,"Incorrect length of destination vector in >mat_sum_rows<");

  vec_zero(b);

  switch(type){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        b[j] += a[i][j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        b[j] += a[j][i];
      for(j=i; j<=ac; j++)
        b[j] += a[i][j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++)
      b[i] = a[i][i];
    break;

  default:
    fprintf(stderr, "ERROR: Unrecognized matrix type in >mat_sum_rows<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sum_rows_weighted
 \date  July 18, 2002
 
 \remarks 
 
 Computes a weighted sum of the row vectors of a matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     w		 : weight vector a
 \param[out]    b		 : vector containing the summed rows of a

 ******************************************************************************/
int
mat_sum_rows_weighted(Matrix a, Vector w, Vector b){
  int i,j;
  int ar,ac,wr,br;
  int type;

  ar   = a[0][NR];
  ac   = a[0][NC];
  type = a[0][MAT_TYPE];
  wr   = w[NR];
  br   = b[NR];

  if(ar != wr){
    fprintf(stderr,"ERROR: Incorrect length of weight vector in >mat_sum_rows_weighted<");
    return FALSE;
  }
  if(ac != br){
    fprintf(stderr,"ERROR: Incorrect length of destination vector in >mat_sum_rows_weighted<");
    return FALSE;
  }

  vec_zero(b);

  switch(type){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        b[j] += w[i]*a[i][j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        b[j] += w[i]*a[j][i];
      for(j=i; j<=ac; j++)
        b[j] += w[i]*a[i][j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++)
      b[i] = w[i]*a[i][i];
    break;

  default:
    fprintf(stderr, "ERROR: Unrecognized matrix type in >mat_sum_rows<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sum_columns
 \date  July 17, 2002
 
 \remarks 
 
 Sums up the column vectors of a matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[out]    b		 : vector containing the summed columns of a

 ******************************************************************************/
int
mat_sum_columns(Matrix a, Vector b){
  int i,j;
  int ar,ac,br;
  int type;

  ar   = a[0][NR];
  ac   = a[0][NC];
  type = a[0][MAT_TYPE];
  br   = b[NR];

  if(ar != br){
    fprintf(stderr,"ERROR: Incorrect length of destination vector in >mat_sum_columns<");
    return FALSE;
  }

  vec_zero(b);

  switch(type){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        b[i] += a[i][j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        b[i] += a[j][i];
      for(j=i; j<=ac; j++)
        b[i] += a[i][j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++)
      b[i] = a[i][i];
    break;

  default:
    fprintf(stderr, "ERROR: Unrecognized matrix type in >mat_sum_columns<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sum_columns_weighted
 \date  July 18, 2002
 
 \remarks 
 
 Compute the weighted sum of the column vectors of a matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     w		 : weight vector a
 \param[out]    b		 : vector containing the summed columns of a

 ******************************************************************************/
int
mat_sum_columns_weighted(Matrix a, Vector w, Vector b){
  int i,j;
  int ar,ac,wr,br;
  int type;

  ar   = a[0][NR];
  ac   = a[0][NC];
  type = a[0][MAT_TYPE];
  wr   = w[NR];
  br   = b[NR];

  if(ac != wr){
    fprintf(stderr,"ERROR: Incorrect length of weight vector in >mat_sum_columns_weighted<");
    return FALSE;
  }
  if(ar != br){
    fprintf(stderr,"ERROR: Incorrect length of destination vector in >mat_sum_columns_weighted<");
    return FALSE;
  }

  vec_zero(b);

  switch(type){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        b[i] += w[j]*a[i][j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        b[i] += w[j]*a[j][i];
      for(j=i; j<=ac; j++)
        b[i] += w[j]*a[i][j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++)
      b[i] = w[i]*a[i][i];
    break;

  default:
    fprintf(stderr, "ERROR: Unrecognized matrix type in >mat_sum_columns_weighted<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mahal_matrix
 \date  August 22, 2002
 
 \remarks 
 
 calculates b a b' where a and b are both matrices

 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c               : matrix c
 ******************************************************************************/
int
mat_mahal_matrix(Matrix a, Matrix b, Matrix c){
  int i,j,k;
  int ar,br,cr,ac,bc,cc;
  int type;
  double temp;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[0][NR];
  bc = b[0][NC];
  cr = c[0][NR];
  cc = c[0][NC];

  if(bc!=ar || ar!=ac){
    fprintf(stderr, "Incompatible input matrix sizes in >mat_mahal_matrix<");
    return FALSE;
  }
  if(cr!=cc || cr!=br){
    fprintf(stderr, "Incompatible output matrix size in >mat_mahal_matrix<");
    return FALSE;
  }
  if(c[0][MAT_TYPE]>a[0][MAT_TYPE] || c[0][MAT_TYPE]==IS_DIAG){
    fprintf(stderr, "Incompatible output matrix type in >mat_mahal_matrix<");
    return FALSE;
  }

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  mat_zero(c);

  switch(type){
  case FULL_FULL:
    for(i=1; i<=br; i++)
      for(j=1; j<=ac; j++){
        temp = 0.0;
        for(k=1; k<=bc; k++)
          temp += b[i][k]*a[k][j];
        for(k=1; k<=br; k++)
          c[i][k] += temp*b[k][j];
      }
    break;

  case FULL_SYMM:
    break;

  case FULL_DIAG:
    break;

  case SYMM_FULL:
    for(i=1; i<=br; i++)
      for(j=1; j<=ac; j++){
        temp = 0.0;
        for(k=1; k<=j; k++)
          temp += b[i][k]*a[k][j];
        for(k=j+1; k<=bc; k++)
          temp += b[i][k]*a[j][k];

        for(k=i; k<=br; k++)
          c[i][k] += temp*b[k][j];
      }

    if(c[0][MAT_TYPE]==IS_FULL){
      for(i=1; i<=cr; i++)
        for(j=i+1; j<=cc; j++)
          c[j][i] = c[i][j];
    }
    break;

  case SYMM_SYMM:
    break;

  case SYMM_DIAG:
    break;

  case DIAG_FULL:
    break;

  case DIAG_SYMM:
    break;

  case DIAG_DIAG:
    break;

  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mahal_size
 \date  August 17, 92
 
 \remarks 
 
 calculates b'a b, i.e., the mahalanobis distance

 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     nr		 : number of rows in a and b 
 \param[in]     b		 : vector b
 
 ******************************************************************************/
double   
mat_mahal_size(Matrix a, int nr, Vector b)
{
  int    i,j;
  double mahal=0.0;

  if (a[0][NR]!=b[NR] || a[0][NC]!=b[NR]) {
    printf("Incompatible matrix vector combination in mat_mahal\n");
    return 0.0;
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      for (j=i; j<=nr; ++j) {
	if (i == j) {
	  mahal += a[i][j]*b[i]*b[j];
	} else {
	  mahal += 2.*a[i][j]*b[i]*b[j];
	}
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      mahal += a[i][i]*b[i]*b[i];
    }

    break;


  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      for (j=1; j<=nr; ++j) {
	mahal += a[i][j]*b[i]*b[j];
      }
    }

    break;


  default:

    exit(-15);

  }

  return (mahal);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mahal
 \date  August 17, 92
 
 \remarks 
 
 calculates b'a b, i.e., the mahalanobis distance

 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b
 
 ******************************************************************************/
double   
mat_mahal(Matrix a, Vector b)

{
  int    i,j;
  double mahal=0.0;
  int    nr;

  nr = b[NR];

  if (a[0][NR]!=b[NR] || a[0][NC]!=b[NR]) {
    printf("Incompatible matrix vector combination in mat_mahal\n");
    return 0.0;
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      for (j=i; j<=nr; ++j) {
	if (i == j) {
	  mahal += a[i][j]*b[i]*b[j];
	} else {
	  mahal += 2.*a[i][j]*b[i]*b[j];
	}
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      mahal += a[i][i]*b[i]*b[i];
    }

    break;


  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      for (j=1; j<=nr; ++j) {
	mahal += a[i][j]*b[i]*b[j];
      }
    }

    break;


  default:

    exit(-15);

  }

  return (mahal);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mahal_irr
 \date  August 17, 92
 
 \remarks 
 
 calculates c' a  b, i.e., the "irregular mahalanobis distance"

 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b
 \param[in]     c		 : vector c
 
 ******************************************************************************/
double   
mat_mahal_irr(Vector c, Matrix a, Vector b)

{
  int i,j;
  double mahal=0.0;
  int    nr,nc;

  nr = a[0][NR];
  nc = a[0][NC];

  if (nr!=c[NR] || nc!=b[NR]) {
    printf("Incompatible matrix vector combination in mat_mahal_irr\n");
    return 0.0;
  }



  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      for (j=1; j<=nc; ++j) {
	mahal += a[i][j]*c[i]*b[j];
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      mahal += a[i][i]*c[i]*b[i];
    }

    break;


  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      for (j=i; j<=nc; ++j) {
	if (i==j) {
	  mahal += a[i][j]*c[i]*b[j];
	} else {
	  mahal += a[i][j]*c[i]*(b[j]+b[i]);
	}
      }
    }

    break;


  default:

    exit(-16);

  }


  return (mahal);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_mult_inner
 \date  August 17, 92
 
 \remarks 
 
 inner product a' * b, return result
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     b		 : vector b

 ******************************************************************************/
double  
vec_mult_inner(Vector a, Vector b)

{

  double aux = 0;
  int i;

  if (a[NR] != b[NR]) {
    printf("Incompatible vectors in vec_mult_inner\n");
    return 0.0;
  }

  for (i=1; i<=a[NR]; ++i) aux += a[i] * b[i];

  return aux;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_mult_inner_size
 \date  August 17, 92
 
 \remarks 
 
 inner product a' * b, return result
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[in]     nr               : number for rows

 ******************************************************************************/
double  
vec_mult_inner_size(Vector a, Vector b, int nr)

{

  double aux = 0;
  int i;

  for (i=1; i<=nr; ++i) aux += a[i] * b[i];

  return aux;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_mult_outer
 \date  Sept. 2005
 
 \remarks 
 
 outer product c = a x b
 vector indices start at "1". c can be the same as one of the calling vectors

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[out]    c		 : vector c

 ******************************************************************************/
int  
vec_mult_outer(Vector a, Vector b, Vector c)

{

  double aux[3+1];
  int i;

  if (a[NR] != b[NR]) {
    printf("Incompatible vectors in vec_mult_inner\n");
    return FALSE;
  }

  if (a[NR] != 3) {
    printf("Outer product only defined for 3 dimensional vectors\n");
    return FALSE;
  }

  aux[1] = a[2]*b[3]-a[3]*b[2];
  aux[2] = a[3]*b[1]-a[1]*b[3];
  aux[3] = a[1]*b[2]-a[2]*b[1];

  for (i=1; i<=3; ++i)
    c[i] = aux[i];

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_mult_outer_size
 \date  Sept. 2005
 
 \remarks 
 
 outer product c = a x b
 vector indices start at "1". c can be the same as one of the calling vectors

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[in]     nr               : number of rows in vectors
 \param[out]    c		 : vector c

 ******************************************************************************/
int
vec_mult_outer_size(Vector a, Vector b, int nr, Vector c)

{

  double aux[3+1];
  int i;

  if (nr != 3) {
    printf("Outer product only defined for 3 dimensional vectors\n");
    return FALSE;
  }

  aux[1] = a[2]*b[3]-a[3]*b[2];
  aux[2] = a[3]*b[1]-a[1]*b[3];
  aux[3] = a[1]*b[2]-a[2]*b[1];

  for (i=1; i<=3; ++i)
    c[i] = aux[i];

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  vec_mult_scalar
\date  August 17, 92

\remarks 

product of vector a  * scalar = c
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     scalar           : value by which a is to be multiplied with
 \param[out]    c		 : vector c (result)

 ******************************************************************************/
void  
vec_mult_scalar(Vector a, double scalar, Vector c)

{

  double aux = 0;
  int i;

  for (i=1; i<=a[NR]; ++i) c[i] = a[i] * scalar;

}

/*!*****************************************************************************
 *******************************************************************************
\note  vec_mult_scalar_size
\date  August 17, 92

\remarks 

product of vector a  * scalar = c
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     nr               : number of rows in a
 \param[in]     scalar           : value by which a is to be multiplied with
 \param[out]    c		 : vector c (result)

 ******************************************************************************/
void  
vec_mult_scalar_size(Vector a, int nr, double scalar, Vector c)

{

  double aux = 0;
  int i;

  for (i=1; i<=nr; ++i) c[i] = a[i] * scalar;

}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_mult_scalar
\date  August 17, 92

\remarks 

product of matrix a  * scalar = c
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     scalar           : value by which a is to be multiplied with
 \param[out]    c		 : matrix c (result)

 ******************************************************************************/
int
mat_mult_scalar(Matrix a, double scalar, Matrix c)

{

  double aux = 0;
  int i,j;

  if (a[0][NR]!=c[0][NR] || a[0][NC]!=c[0][NC] || 
      c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrices in mat_mult_scalar\n");
    return FALSE;
  }

  if (a!=c)
    mat_equal(a,c);
  
  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=a[0][NC]; ++j) {
	c[i][j] *= scalar;
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] *= scalar;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] *= scalar;
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_equal_scalar
\date  August 17, 92

\remarks 

 fills all elements of the matrix with the given scalar
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     scalar           : value which is filled into c
 \param[out]    c		 : matrix c (result)

 ******************************************************************************/
int
mat_equal_scalar(double scalar, Matrix c)

{

  double aux = 0;
  int i,j;

  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=c[0][NC]; ++j) {
	c[i][j] = scalar;
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] = scalar;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] = scalar;
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_add_scalar
\date  August 17, 92

\remarks 

add the scalar to a and assigns c
vector indices start at "1".

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a                : input matrix
 \param[in]     scalar           : value which is filled into c
 \param[out]    c		 : matrix c (result)

 ******************************************************************************/
int
mat_add_scalar(Matrix a, double scalar, Matrix c)

{

  double aux = 0;
  int i,j;

  if (a[0][NR]!=c[0][NR] || a[0][NC]!=c[0][NC] || 
      c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrices in mat_mult_scalar\n");
    return FALSE;
  }

  if (a!=c)
    mat_equal(a,c);
  
  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=c[0][NC]; ++j) {
	c[i][j] += scalar;
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] += scalar;
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] += scalar;
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_equal_apply_math
\date  August 17, 92

\remarks 

 applies a given (double) function to each element of the matrix

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		   : matrix a
 \param[in]     func               : (double) function pointer to be applied
 \param[out]    c                  : output matrix

 ******************************************************************************/
int
mat_equal_apply_math(Matrix a, double (*func)(double), Matrix c)

{

  double aux = 0;
  int i,j;

  if (a[0][NR]!=c[0][NR] || a[0][NC]!=c[0][NC] || 
      c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrices in mat_mult_scalar\n");
    return FALSE;
  }

  if (a!=c)
    mat_equal(a,c);

  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] = (*func)(c[i][i]);
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  fmat_equal_apply_math
\date  August 17, 92

\remarks 

 applies a given (double) function to each element of the matrix

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		   : matrix a
 \param[in]     func               : (float) function pointer to be applied
 \param[out]    c                  : output matrix

 ******************************************************************************/
int
fmat_equal_apply_math(fMatrix a, float (*func)(float), fMatrix c)

{

  float aux = 0;
  int i,j;

  if (a[0][NR]!=c[0][NR] || a[0][NC]!=c[0][NC] || 
      c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrices in mat_mult_scalar\n");
    return FALSE;
  }

  if (a!=c)
    fmat_equal(a,c);

  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] = (*func)(c[i][i]);
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  imat_equal_apply_math
\date  August 17, 92

\remarks 

 applies a given (double) function to each element of the matrix

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		   : matrix a
 \param[in]     func               : (int) function pointer to be applied
 \param[out]    c                  : output matrix

 ******************************************************************************/
int
imat_equal_apply_math(iMatrix a, int (*func)(int), iMatrix c)

{

  int aux = 0;
  int i,j;

  if (a[0][NR]!=c[0][NR] || a[0][NC]!=c[0][NC] || 
      c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrices in mat_mult_scalar\n");
    return FALSE;
  }

  if (a!=c)
    imat_equal(a,c);

  switch ((int)c[0][MAT_TYPE]) {
    
  case IS_FULL:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=1; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  case IS_DIAG:

    for (i=1; i<=c[0][NR]; ++i) {
      c[i][i] = (*func)(c[i][i]);
    }

    break;


  case IS_SYMM:

    for (i=1; i<=c[0][NR]; ++i) {
      for (j=i; j<=c[0][NC]; ++j) {
	c[i][j] = (*func)(c[i][j]);
      }
    }

    break;

  }

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
\note  vec_mat_mult
\date  August 17, 92

\remarks 

multiplies a vector with a matrix: a * b, assuming indices
start at "1".
Note: The program can also cope with passing the same vector as
factor and result.

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
vec_mat_mult(Vector a, Matrix b, Vector c)

{

  int     i,j,m;
  Vector  temp;
  int     ar,bc,br;

  ar = a[NR];
  br = b[0][NR];
  bc = b[0][NC];
  
  if (ar != br) {
    printf("Matrix and vector are incompatible in vec_mat_mult.\n");
    return FALSE;
  }

  if (c[NR] != bc) {
    printf("Output vector is incompatible in vec_mat_mult.\n");
    return FALSE;
  }

  if (a == c) {
    temp = my_vector(1,bc);
  } else {
    temp = c;
  }

  switch ((int)b[0][MAT_TYPE]) {

  case IS_FULL:
  
    for (i=1; i <= bc; ++i) {
      temp[i]=0;
      for (j=1; j <= br; ++j){
	temp[i] += a[j] * b[j][i];
      }
    }
    
    break;


  case IS_DIAG:

    for (i=1; i <= bc; ++i) {
      temp[i] = a[i] * b[i][i];
    }
    
    break;


  case IS_SYMM:

    for (i=1; i <= bc; ++i) {
      temp[i]=0;
      for (j=i; j <= br; ++j){
	temp[i] += a[j] * b[i][j];
	if (i!=j)
	  temp[j] += a[i] * b[i][j];
      }
    }

    break;


  default:

    exit(-17);

  }


  if (a == c) {
    vec_equal(temp,c);
    my_free_vector(temp,1,bc);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
\note  vec_mat_mult_size
\date  August 17, 92

\remarks 

multiplies a vector with a matrix: a * b, assuming indices
start at "1".
Note: The program can also cope with passing the same vector as
factor and result.

 *******************************************************************************
Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     ar               : number of cofficients in A
 \param[in]     b		 : matrix b
 \param[in]     br               : number of rows in b
 \param[in]     bc               : number of columns in b
 \param[out]    c		 : result of multipliciation

 ******************************************************************************/
int
vec_mat_mult_size(double * a, int ar,  double ** b, int br, int bc, double *c)

{

  int     i,j,m;
  Vector  temp;

  if (ar != br) {
    printf("Matrix and vector are incompatible in vec_mat_mult.\n");
    return FALSE;
  }

  if (a == c) {
    temp = my_vector(1,bc);
  } else {
    temp = c;
  }

  switch ((int)b[0][MAT_TYPE]) {

  case IS_FULL:
  
    for (i=1; i <= bc; ++i) {
      temp[i]=0;
      for (j=1; j <= br; ++j){
	temp[i] += a[j] * b[j][i];
      }
    }
    
    break;


  case IS_DIAG:

    for (i=1; i <= bc; ++i) {
      temp[i] = a[i] * b[i][i];
    }
    
    break;


  case IS_SYMM:

    for (i=1; i <= bc; ++i) {
      temp[i]=0;
      for (j=i; j <= br; ++j){
	temp[i] += a[j] * b[i][j];
	if (i!=j)
	  temp[j] += a[i] * b[i][j];
      }
    }

    break;


  default:

    exit(-17);

  }


  if (a == c) {
    vec_equal(temp,c);
    my_free_vector(temp,1,bc);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_add
 \date  August 17, 92
 
 \remarks 
 
 adds two arbitrary (compatible) matrices a + b, assuming the
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 programs can use overlapping input and output matrices
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
mat_add(Matrix a, Matrix b, Matrix c)

{
  int      i,j;
  Matrix   temp;
  int      nr,nc;
  int      type;

  temp = c;

  if (a[0][NR] != b[0][NR] || a[0][NC] != b[0][NC] ||
      a[0][NR] != c[0][NR] || a[0][NC] != c[0][NC]) {
    printf("Incompatible matrices in mat_add\n");
    return FALSE;
  }

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE] || c[0][MAT_TYPE] > b[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_add\n");
    return FALSE;
  }

  nr = a[0][NR];
  nc = a[0][NC];

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = a[i][j] + b[i][j];
      }
    }

    break;


  case FULL_DIAG:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = a[i][j];
	if (i==j)
	  temp[i][j] += b[i][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] + b[i][j];
	if (i!=j)
	  temp[j][i] = a[j][i] + b[i][j];
      }
    }

    break;
    

  case DIAG_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = b[i][j];
	if (i==j)
	  temp[i][j] += a[i][j];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE] != IS_DIAG)
      mat_zero(temp);

    for (i=1; i <= nr; ++i) {
      temp[i][i] = a[i][i] + b[i][i];
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = b[i][j];
	if (i==j)
	  temp[i][j] += a[i][j];
	else if (temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] + b[i][j];
	if (i!=j)
	  temp[j][i] = a[i][j] + b[j][i];
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j];
	if (i==j)
	  temp[i][j] += b[i][j];
	else if (temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = a[i][j];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] + b[i][j];
	if (i!=j && temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = temp[i][j];
      }
    }

    break;


  default:

    exit(-18);

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_add_size
 \date  August 17, 92
 
 \remarks 
 
 adds two arbitrary (compatible) matrices a + b, assuming the
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 programs can use overlapping input and output matrices
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[in]     nr              : number of rows
 \param[in]     nc              : number of columns
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
mat_add_size(Matrix a, Matrix b, int nr, int nc, Matrix c)

{
  int      i,j;

  for (i=1; i <= nr; ++i) {
    for (j=1; j <= nc; ++j){
      c[i][j] = a[i][j] + b[i][j];
    }
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_add_vector_rows
 \date  July 18, 2002
 
 \remarks 
 
 Add a vector to each row of the matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b to be added
 \param[out]    c		 : result matrix c

 ******************************************************************************/
int
mat_add_vec_rows(Matrix a, Vector b, Matrix c){
  int i,j;
  int ar,ac,br,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[NR];
  cr = c[0][NR];
  cc = c[0][NC];

  if(cr!=ar || cc!=ac){
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_add_vec_rows<");
    return FALSE;
  }
  if(br!=ac){
    fprintf(stderr, "ERROR: Incompatible vector size in >mat_add_vec_rows<");
    return FALSE;
  }
  if(c[0][MAT_TYPE]!=IS_FULL){
    fprintf(stderr, "ERROR: Incompatible matrix type in >mat_add_vec_rows<");
    return FALSE;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j] + b[j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = a[j][i] + b[j];
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j] + b[j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = b[j];
      c[i][i] = a[i][i] + b[i];
      for(j=i+1; j<=ac; j++)
        c[i][j] = b[j];
    }
    break;

  default:
    fprintf(stderr, "ABORT: Unknown matrix type in >mat_add_vec_rows<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_add_vector_columns
 \date  July 18, 2002
 
 \remarks 
 
 Adds a vector to each column of the matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b to be added
 \param[out]    c		 : result matrix c

 ******************************************************************************/
int
mat_add_vec_columns(Matrix a, Vector b, Matrix c){
  int i,j;
  int ar,ac,br,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[NR];
  cr = c[0][NR];
  cc = c[0][NC];

  if(cr!=ar || cc!=ac){
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_add_vec_columns<");
    return FALSE;
  }
  if(br!=ar){
    fprintf(stderr, "ERROR: Incompatible vector size in >mat_add_vec_columns<");
    return FALSE;
  }
  if(c[0][MAT_TYPE]!=IS_FULL){
    fprintf(stderr, "ERROR: Require FULL destination matrix in >mat_add_vec_columns<");
    return FALSE;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j] + b[i];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = a[j][i] + b[i];
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j] + b[i];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = b[i];
      c[i][i] = a[i][i] + b[i];
      for(j=i+1; j<=ac; j++)
        c[i][j] = b[i];
    }
    break;

  default:
    fprintf(stderr, "ABORT: Unknown matrix type in >mat_add_vec_columns<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sub
 \date  August 17, 92
 
 \remarks 
 
 subtracts two arbitrary (compatible) matrices a - b, assuming the
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     b		 : matrix b
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
mat_sub(Matrix a, Matrix b, Matrix c)

{
  int      i,j;
  Matrix   temp;
  int      nr,nc;
  int      type;

  temp = c;

  if (a[0][NR] != b[0][NR] || a[0][NC] != b[0][NC] ||
      a[0][NR] != c[0][NR] || a[0][NC] != c[0][NC]) {
    printf("Incompatible matrices in mat_sub\n");
    return FALSE;
  }

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE] || c[0][MAT_TYPE] > b[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_sub\n");
    return FALSE;
  }

  nr = a[0][NR];
  nc = a[0][NC];

  type = a[0][MAT_TYPE]*10+b[0][MAT_TYPE];

  switch (type) {

  case FULL_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = a[i][j] - b[i][j];
      }
    }

    break;


  case FULL_DIAG:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = a[i][j];
	if (i==j)
	  temp[i][j] -= b[i][j];
      }
    }

    break;


  case FULL_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] - b[i][j];
	if (i!=j)
	  temp[j][i] = a[j][i] - b[i][j];
      }
    }

    break;
    

  case DIAG_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[i][j] = b[i][j];
	if (i==j)
	  temp[i][j] -= a[i][j];
      }
    }

    break;


  case DIAG_DIAG:

    if (temp[0][MAT_TYPE] != IS_DIAG)
      mat_zero(temp);

    for (i=1; i <= nr; ++i) {
      temp[i][i] = a[i][i] - b[i][i];
    }

    break;


  case DIAG_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = b[i][j];
	if (i==j)
	  temp[i][j] -= a[i][j];
	else if (temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = b[i][j];
      }
    }

    break;


  case SYMM_FULL:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] - b[i][j];
	if (i!=j)
	  temp[j][i] = a[i][j] - b[j][i];
      }
    }

    break;


  case SYMM_DIAG:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j];
	if (i==j)
	  temp[i][j] -= b[i][j];
	else if (temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = a[i][j];
      }
    }

    break;


  case SYMM_SYMM:

    for (i=1; i <= nr; ++i) {
      for (j=i; j <= nc; ++j){
	temp[i][j] = a[i][j] - b[i][j];
	if (i!=j && temp[0][MAT_TYPE] == IS_FULL)
	  temp[j][i] = temp[i][j];
      }
    }

    break;


  default:

    exit(-19);

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sub_vector_rows
 \date  July 18, 2002
 
 \remarks 
 
 Subtracts a vector from each row of the matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b to be subtracted
 \param[out]    c		 : result matrix c

 ******************************************************************************/
int
mat_sub_vec_rows(Matrix a, Vector b, Matrix c){
  int i,j;
  int ar,ac,br,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[NR];
  cr = c[0][NR];
  cc = c[0][NC];

  if(cr!=ar || cc!=ac){
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_sub_vec_rows<");
    return FALSE;
  }
  if(br!=ac){
    fprintf(stderr, "ERROR: Incompatible vector size in >mat_sub_vec_rows<");
    return FALSE;
  }
  if(c[0][MAT_TYPE]!=IS_FULL){
    fprintf(stderr, "ERROR: Incompatible matrix type in >mat_sub_vec_rows<");
    return FALSE;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j] - b[j];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = a[j][i] - b[j];
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j] - b[j];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = -b[j];
      c[i][i] = a[i][i] - b[i];
      for(j=i+1; j<=ac; j++)
        c[i][j] = -b[j];
    }
    break;

  default:
    fprintf(stderr, "ABORT: Unknown matrix type in >mat_sub_vec_rows<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_sub_vector_columns
 \date  July 18, 2002
 
 \remarks 
 
 Subtracts a vector from each column of the matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     b		 : vector b to be subtracted
 \param[out]    c		 : result matrix c

 ******************************************************************************/
int
mat_sub_vec_columns(Matrix a, Vector b, Matrix c){
  int i,j;
  int ar,ac,br,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  br = b[NR];
  cr = c[0][NR];
  cc = c[0][NC];

  if(cr!=ar || cc!=ac){
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_sub_vec_columns<");
    return FALSE;
  }
  if(br!=ar){
    fprintf(stderr, "ERROR: Incompatible vector size in >mat_sub_vec_columns<");
    return FALSE;
  }
  if(c[0][MAT_TYPE]!=IS_FULL){
    fprintf(stderr, "ERROR: Incompatible matrix type in >mat_sub_vec_columns<");
    return FALSE;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=ar; i++)
      for(j=1; j<=ac; j++)
        c[i][j] = a[i][j] - b[i];
    break;

  case IS_SYMM:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = a[j][i] - b[i];
      for(j=i; j<=ac; j++)
        c[i][j] = a[i][j] - b[i];
    }
    break;

  case IS_DIAG:
    for(i=1; i<=ar; i++){
      for(j=1; j<i; j++)
        c[i][j] = -b[i];
      c[i][i] = a[i][i] - b[i];
      for(j=i+1; j<=ac; j++)
        c[i][j] = -b[i];
    }
    break;

  default:
    fprintf(stderr, "ABORT: Unknown matrix type in >mat_sub_vec_columns<");
    exit(-12);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_equal
 \date  August 17, 92
 
 \remarks 
 
 set matrix c = a
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
mat_equal(Matrix a, Matrix c)

{
  int i,j;

  if (a[0][NR] != c[0][NR] || a[0][NC] !=c[0][NC]) {
    printf("Incompatible matrices in mat_equal\n");
    return FALSE;
  }

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_equal\n");
    return FALSE;
  }


  if (c[0][MAT_TYPE] < a[0][MAT_TYPE]) {
    mat_zero(c);
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=1; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
      }
    }

    break;


  case IS_SYMM:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=i; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
	if (c[0][MAT_TYPE]==IS_FULL && i!=j)
	  c[j][i] = a[i][j];
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i <= a[0][NR]; ++i) {
      c[i][i] = a[i][i];
    }

    break;


  default:

    exit(-19);

  }

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  fmat_equal
 \date  August 17, 92
 
 \remarks 
 
 set matrix c = a
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
fmat_equal(fMatrix a, fMatrix c)

{
  int i,j;

  if (a[0][NR] != c[0][NR] || a[0][NC] !=c[0][NC]) {
    printf("Incompatible matrices in mat_equal\n");
    return FALSE;
  }

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_equal\n");
    return FALSE;
  }


  if (c[0][MAT_TYPE] < a[0][MAT_TYPE]) {
    fmat_zero(c);
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=1; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
      }
    }

    break;


  case IS_SYMM:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=i; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
	if (c[0][MAT_TYPE]==IS_FULL && i!=j)
	  c[j][i] = a[i][j];
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i <= a[0][NR]; ++i) {
      c[i][i] = a[i][i];
    }

    break;


  default:

    exit(-19);

  }

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  imat_equal
 \date  August 17, 92
 
 \remarks 
 
 set matrix c = a
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
imat_equal(iMatrix a, iMatrix c)

{
  int i,j;

  if (a[0][NR] != c[0][NR] || a[0][NC] !=c[0][NC]) {
    printf("Incompatible matrices in mat_equal\n");
    return FALSE;
  }

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_equal\n");
    return FALSE;
  }


  if (c[0][MAT_TYPE] < a[0][MAT_TYPE]) {
    mat_izero(c);
  }

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=1; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
      }
    }

    break;


  case IS_SYMM:

    for (i=1; i <= a[0][NR]; ++i) {
      for (j=i; j <= a[0][NC]; ++j){
	c[i][j] = a[i][j];
	if (c[0][MAT_TYPE]==IS_FULL && i!=j)
	  c[j][i] = a[i][j];
      }
    }

    break;


  case IS_DIAG:

    for (i=1; i <= a[0][NR]; ++i) {
      c[i][i] = a[i][i];
    }

    break;


  default:

    exit(-19);

  }

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  mat_equal_size
 \date  August 17, 92
 
 \remarks 
 
 set matrix c = a
 matrix indices start at "1".
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     nr              : number of rows of a
 \param[in]     nc              : number of columns of a
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
mat_equal_size(Matrix a, int nr, int nc, Matrix c)

{
  int i,j;

  for (i=1; i <= nr; ++i) {
    for (j=1; j <= nc; ++j){
      c[i][j] = a[i][j];
    }
  }

  return TRUE;

}


/*!*****************************************************************************
 *******************************************************************************
 \note  vec_add
 \date  August 17, 92
 
 \remarks 
 
 adds two arbitrary (compatible) vectors a + b, assuming the
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
vec_add(Vector a, Vector b, Vector c)
{

  int     i;
  
  if (a[NR] != b[NR] || a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_add\n");
    return FALSE;
  }

  for (i=1; i <= a[NR]; ++i) {
    c[i] = a[i] + b[i];
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_add_size
 \date  August 17, 92
 
 \remarks 
 
 adds two arbitrary (compatible) vectors a + b, assuming the
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[in]     nr              : number of elements in vector
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
vec_add_size(Vector a, Vector b, int nr, Vector c)
{

  int     i;
  
  for (i=1; i <= nr; ++i) {
    c[i] = a[i] + b[i];
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_add_scalar
 \date  August 17, 92
 
 \remarks 
 
 adds scalar to vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     scalar		 : scalar value
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
vec_add_scalar(Vector a, double scalar, Vector c)
{

  int     i;
  
  if (a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_add_scalar\n");
    return FALSE;
  }

  for (i=1; i <= a[NR]; ++i) {
    c[i] = a[i] + scalar;
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_sub
 \date  August 17, 92
 
 \remarks 
 
 subtracts two arbitrary (compatible) vectors a - b, assuming the
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
vec_sub(Vector a, Vector b, Vector c)
{

  int     i;
  
  if (a[NR] != b[NR] || a[NR] != c[NR]) {
    printf("Incompatible vectors in vec_sub\n");
    return FALSE;
  }

  for (i=1; i <= a[NR]; ++i) {
    c[i] = a[i] - b[i];
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_euc2
 \date  May 1998
 
 \remarks 
 
 returns the squared euclidan norm of two vectors
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 
 ******************************************************************************/
double 
vec_euc2(Vector a, Vector b)
{

  int     i;
  double  euc2=0;
  
  if (a[NR] != b[NR]) {
    printf("Incompatible vectors in vec_euc2\n");
    return FALSE;
  }

  for (i=1; i <= a[NR]; ++i) {
    euc2 += sqr(a[i] - b[i]);
  }

  return euc2;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_euc2_size
 \date  May 1998
 
 \remarks 
 
 returns builds the squared euclidian norm between two equal size vectors
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[in]     nr              : length of vector
 
 ******************************************************************************/
double
vec_euc2_size(Vector a, Vector b, int nr)
{

  int     i;
  double euc2=0;
  
  for (i=1; i <= nr; ++i) {
    euc2 += sqr(a[i] - b[i]);
  }

  return euc2;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_sub_size
 \date  August 17, 92
 
 \remarks 
 
 subtracts two arbitrary (compatible) vectors a + b, assuming the
 vector indices start at "1".
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : vector a
 \param[in]     b		 : vector b
 \param[in]     nr              : length of vector
 \param[out]    c		 : result of addition
 
 ******************************************************************************/
int
vec_sub_size(Vector a, Vector b, int nr, Vector c)
{

  int     i;
  
  for (i=1; i <= nr; ++i) {
    c[i] = a[i] - b[i];
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_trans
 \date  August 17, 92
 
 \remarks 
 
 builds the transpose of a matrix
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[out]    c		 : result of transpose
 
 ******************************************************************************/
int
mat_trans(Matrix a, Matrix c)

{
  int    i,j;
  Matrix temp;
  int    nr,nc;

  if (c[0][MAT_TYPE] > a[0][MAT_TYPE]) {
    printf("Incompatible matrix types in mat_trans\n");
    return FALSE;
  }

  if (a[0][NR] != c[0][NC] || a[0][NC] != c[0][NR]) {
    printf("Incompatible matrices in mat_trans\n");
    return FALSE;
  }

  nc = a[0][NC];
  nr = a[0][NR];

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    if (a == c) {
      temp = my_matrix(1,nc,1,nr);
    } else {
      temp = c;
    } 

    for (i=1; i <= nr; ++i) {
      for (j=1; j <= nc; ++j){
	temp[j][i] = a[i][j];
      }
    }

    if (a == c) {
      mat_equal(temp,c);
      my_free_matrix(temp,1,nc,1,nr);
    }

    break;


  case IS_DIAG:

    mat_equal(a,c);

    break;


  case IS_SYMM:

    mat_equal(a,c);

    break;


  default:

    exit(-20);

  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_trans_size
 \date  August 17, 92
 
 \remarks 
 
 builds the transpose of a matrix
 Note: if a vector is passed, it must be the pointer to the vector;
 everything is handled as a matrix!
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a		 : matrix a
 \param[in]     nr              : number of rows of a
 \param[in]     nc              : number of columns of a
 \param[out]    c		 : result of transpose
 
 ******************************************************************************/
int
mat_trans_size(Matrix a, int nr, int nc, Matrix c)

{
  int    i,j;
  Matrix temp;

  if (a == c) {
    temp = my_matrix(1,nc,1,nr);
  } else {
    temp = c;
  }
  
  for (i=1; i <= nr; ++i) {
    for (j=1; j <= nc; ++j){
      temp[j][i] = a[i][j];
    }
  }
  
  if (a == c) {
    mat_equal(temp,c);
    my_free_matrix(temp,1,nc,1,nr);
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_mult_inner
 \date  January 94
 Rewritten      : July 18, 2002 (adsouza)
 
 \remarks 
 
 multiplies the transpose of the matrix with the matrix: a' * a

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[out]    c		 : result of operation

 ******************************************************************************/
int
mat_mult_inner(Matrix a, Matrix c){
  int    i,j,k;
  Matrix temp;
  int    ar,ac,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  cr = c[0][NR];
  cc = c[0][NC];
  
  if (cr!=ac || cc!=ac) {
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_mult_inner<\n");
    return FALSE;
  }
  if(c[0][MAT_TYPE]==IS_DIAG && a[0][MAT_TYPE]!=IS_DIAG){
    fprintf(stderr, "ERROR: Incompatible output matrix type in >mat_mult_inner<\n");
    return FALSE;
  }
  
  /* check whether input and output matrices are different */
  if (a == c) {
    if (c[0][MAT_TYPE] == IS_SYMM)
      temp = my_matrix_symm(1,cr,1,cc);
    else if (c[0][MAT_TYPE] == IS_DIAG)
      temp = my_matrix_diag(1,cr,1,cc);
    else 
      temp = my_matrix(1,cr,1,cc);
  } else {
    temp = c;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=cr; i++)
      for(j=i; j<=cc; j++){
        temp[i][j] = 0.0;
        for(k=1; k<=ar; k++)
          temp[i][j] += a[k][i]*a[k][j];
      }

    if(temp[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=cr; i++)
        for(j=i+1; j<=cc; j++)
          temp[j][i] = temp[i][j];
    break;

  case IS_SYMM:
    for (i=1; i<=cr; i++)
      for (j=i; j<=cc; j++){
        temp[i][j]=0.0;
        for (k=1; k<=ar; k++){
          if(i<=k && k<=j)
            temp[i][j] += a[i][k]*a[k][j];
          else if(i>k && k<=j)
            temp[i][j] += a[k][i]*a[k][j];
          else if(i<=k && k>j)
            temp[i][j] += a[i][k]*a[j][k];
          else
            temp[i][j] += a[k][i]*a[j][k];
        }
      }

    if(temp[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=cr; i++)
        for(j=i+1; j<=cc; j++)
          temp[j][i] = temp[i][j];
    break;

  case IS_DIAG:
    mat_zero(temp);
    for(i=1; i<=cr; i++)
      temp[i][i] = a[i][i]*a[i][i];
    break;

  default:
    fprintf(stderr, "ABORT: Unrecognized matrix type in >mat_mult_inner<\n");
    exit(-12);
  }

  if(a==c){
    mat_equal(temp,c);
    my_free_matrix(temp,1,cr,1,cc);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_mult_outer
\date  January 94
Rewritten       : July 18, 2002 (adsouza)

\remarks 

	multiplies the matrix with its transpose: a * a'
	
 *******************************************************************************
	Parameters:  (i/o = input/output)
	
 \param[in]     a		 : matrix a
 \param[out]    c		 : result of operation
	
 ******************************************************************************/
int
mat_mult_outer(Matrix a, Matrix c){
  int    i,j,k;
  Matrix temp;
  int    ar,ac,cr,cc;

  ar = a[0][NR];
  ac = a[0][NC];
  cr = c[0][NR];
  cc = c[0][NC];
  
  if (cr!=ar || cc!=ar) {
    fprintf(stderr, "ERROR: Incompatible matrix sizes in >mat_mult_inner<\n");
    return FALSE;
  }
  if(c[0][MAT_TYPE]==IS_DIAG && a[0][MAT_TYPE]!=IS_DIAG){
    fprintf(stderr, "ERROR: Incompatible output matrix type in >mat_mult_inner<\n");
    return FALSE;
  }
  
  /* check whether input and output matrices are different */
  if (a == c) {
    if (c[0][MAT_TYPE] == IS_SYMM)
      temp = my_matrix_symm(1,cr,1,cc);
    else if (c[0][MAT_TYPE] == IS_DIAG)
      temp = my_matrix_diag(1,cr,1,cc);
    else 
      temp = my_matrix(1,cr,1,cc);
  } else {
    temp = c;
  }

  switch((int)a[0][MAT_TYPE]){
  case IS_FULL:
    for(i=1; i<=cr; i++)
      for(j=i; j<=cc; j++){
        temp[i][j] = 0.0;
        for(k=1; k<=ac; k++)
          temp[i][j] += a[i][k]*a[j][k];
      }

    if(temp[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=cr; i++)
        for(j=i+1; j<=cc; j++)
          temp[j][i] = temp[i][j];
    break;

  case IS_SYMM:
    for (i=1; i<=cr; i++)
      for (j=i; j<=cc; j++){
        temp[i][j]=0.0;
        for (k=1; k<=ac; k++){
          if(i<=k && k<=j)
            temp[i][j] += a[i][k]*a[k][j];
          else if(i>k && k<=j)
            temp[i][j] += a[k][i]*a[k][j];
          else if(i<=k && k>j)
            temp[i][j] += a[i][k]*a[j][k];
          else
            temp[i][j] += a[k][i]*a[j][k];
        }
      }

    if(temp[0][MAT_TYPE]==IS_FULL)
      for(i=1; i<=cr; i++)
        for(j=i+1; j<=cc; j++)
          temp[j][i] = temp[i][j];
    break;

  case IS_DIAG:
    mat_zero(temp);
    for(i=1; i<=cr; i++)
      temp[i][i] = a[i][i]*a[i][i];
    break;

  default:
    fprintf(stderr, "ABORT: Unrecognized matrix type in >mat_mult_inner<\n");
    exit(-12);
  }

  if(a==c){
    mat_equal(temp,c);
    my_free_matrix(temp,1,cr,1,cc);
  }

  return TRUE;
}

/*!*****************************************************************************
 *******************************************************************************
\note  mat_mult_outer_size
\date  January 94

\remarks 

	multiplies the matrix with its transpose: a * a'
	
 *******************************************************************************
	Parameters:  (i/o = input/output)
	
 \param[in]     a		 : matrix a
 \param[in]     nr               : number of rows of a
 \param[in]     nc               : number of colums of a
 \param[out]    c		 : result of operation
	
 ******************************************************************************/
int
mat_mult_outer_size(Matrix a, int nr, int nc, Matrix c)

{
  int    i,j,m;
  Matrix temp;

  /* check whether input and output matrices are different */
  
  if (a == c) {
    temp = my_matrix(1,nr,1,nr);
  } else {
    temp = c;
  }


  for (i=1; i <= nr; ++i) {
    for (j=1; j <= nr; ++j){
      temp[i][j] = 0.0;
      for (m=1; m <= nc; ++m) {
	temp[i][j] += a[i][m] * a[j][m];
      }
    }
  }

  if ( a == c) {
    mat_equal_size(temp,nr,nr,c);
    my_free_matrix(temp,1,nr,1,nr);
  }

  return TRUE;

}


/*****************************************************************************
  utility program vector: allocates a double vector with range nl...nh
  ****************************************************************************/
Vector
my_vector(int nl, int nh)

{
  double *v;

  if (nl == 1) {
    
    v = (double *) my_calloc_track((size_t) (nh-nl+1+1),sizeof(double));
    if (v==NULL)
      printf("allocation failure in vector()");
    v[0] = nh-nl+1;
    return v;

  } else {

    v = (double *) my_calloc_track((size_t) (nh-nl+1),sizeof(double));
    if (v==NULL)
      printf("allocation failure in vector()");
    return v-nl;

  }

}

void 
my_free_vector(Vector vec, int nl, int nh)
{
  if (nl == 1) {
    free((char*) (vec));
  } else {
    free((char*) (vec+nl));
  }
}

/***************************************************************************
  utility program vector: allocates a float vector with range nl...nh
  **************************************************************************/
fVector
my_fvector(int nl, int nh)
{

  float *v;

  if (nl == 1) {
    
    v = (float *) my_calloc_track((size_t) (nh-nl+1+1),sizeof(float));
    if (v==NULL)
      printf("allocation failure in vector()");
    v[0] = nh-nl+1;
    return v;

  } else {

    v = (float *) my_calloc_track((size_t) (nh-nl+1),sizeof(float));
    if (v==NULL)
      printf("allocation failure in vector()");
    return v-nl;

  }

}

void 
my_free_fvector(fVector vec, int nl, int nh)
{
  if (nl == 1) {
    free((char*) (vec));
  } else {
    free((char*) (vec+nl));
  }
}

/****************************************************************************
  utility program ivector: allocates a int vector with range nl...nh
  ***************************************************************************/
iVector
my_ivector(int nl, int nh)
{
  int *v;

  if (nl == 1) {
    
    v = (int *) my_calloc_track((size_t) (nh-nl+1+1),sizeof(int));
    if (v==NULL)
      printf("allocation failure in vector()");
    v[0] = nh-nl+1;
    return v;

  } else {

    v = (int *) my_calloc_track((size_t) (nh-nl+1),sizeof(int));
    if (v==NULL)
      printf("allocation failure in vector()");
    return v-nl;

  }

}

void 
my_free_ivector(iVector vec, int nl, int nh)
{
  if (nl == 1) {
    free((char*) (vec));
  } else {
    free((char*) (vec+nl));
  }
}

/*****************************************************************************
  utility program livector: allocates a long vector with range nl...nh
  ***************************************************************************/
liVector
my_livector(int nl, int nh)
{
  long *v;

  if (nl == 1) {
    
    v = (long *) my_calloc_track((size_t) (nh-nl+1+1),sizeof(long));
    if (v==NULL)
      printf("allocation failure in vector()");
    v[0] = nh-nl+1;
    return v;

  } else {

    v = (long *) my_calloc_track((size_t) (nh-nl+1),sizeof(long));
    if (v==NULL)
      printf("allocation failure in vector()");
    return v-nl;

  }

}

void 
my_free_livector(liVector vec, int nl, int nh)
{
  if (nl == 1) {
    free((char*) (vec));
  } else {
    free((char*) (vec+nl));
  }
}

/*****************************************************************************
  utility program my_nrerror
  ****************************************************************************/
void 
my_nrerror(char *error_text)
{
  fprintf(stderr,"Numerical Recipes run-time error...\n");
  fprintf(stderr,"%s\n",error_text);
  fprintf(stderr,"...now exiting to system...\n");
  fprintf(stderr,"\n        hit return to continue...\n");
  getchar();
  exit(-1);
}

/******************************************************************************
  utility program my_matrix: note: this is a modified version of
  the normal matrix() numerical recipe version. It allocates
  the memory for the matrix in one chunk, such that it can
  be treated as consecutive memory in write() or DSP programs
  
 *****************************************************************************/
Matrix
my_matrix(int nrl, int nrh, int ncl, int nch)
{

  int      i;
  double **m;
  double  *chunk;
  int      info = FALSE;

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (double **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(double*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (double *) my_calloc_track((size_t) N_MAT_INFO,sizeof(double));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_FULL;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }
    
  chunk = (double *) my_calloc_track( (size_t) (nrh-nrl+1) * (nch-ncl+1),sizeof(double));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(double *) &(chunk[(i-nrl)*(nch-ncl+1)]);
    m[i] -= ncl;
  }
  
  return m;
  
}


/*****************************************************************************
  my_matrix_sym; allocates the memory for a symmetric matrix, i.e., the
  upper diagonal matrix including the diagonal.Can be freed with my_free_matrix
  ***************************************************************************/
Matrix
my_matrix_symm(int nrl, int nrh, int ncl, int nch)
{

  int      i,j;
  double **m;
  double  *chunk;
  int      info = FALSE;
  
  if (nrh-nrl != nch-ncl)
    my_nrerror("diagonal matrices must be square!\n");

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (double **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(double*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (double *) my_calloc_track((size_t) N_MAT_INFO,sizeof(double));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_SYMM;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }

  chunk = (double *)my_calloc_track((size_t)(sqr(nrh-nrl+1) + (nrh-nrl+1))/2.,sizeof(double));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  j = 0;
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(double *) &(chunk[j]);
    m[i] -= ncl + (i-nrl);
    j = j + (nch-ncl+1) - (i-nrl);
  }
  
  return m;

}
/*****************************************************************************
  my_imatrix_sym; allocates the memory for a symmetric matrix, i.e., the
  upper diagonal matrix including the diagonal.Can be freed with my_free_imatrix
  ***************************************************************************/
iMatrix
my_imatrix_symm(int nrl, int nrh, int ncl, int nch)
{

  int      i,j;
  int    **m;
  int     *chunk;
  int      info = FALSE;
  
  if (nrh-nrl != nch-ncl)
    my_nrerror("diagonal matrices must be square!\n");

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (int **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(int*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in imatrix()");

  if (info) {

    m[0] = (int *) my_calloc_track((size_t) N_MAT_INFO,sizeof(int));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in imatrix()");
    m[0][MAT_TYPE] = IS_SYMM;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }

  chunk = (int *)my_calloc_track((size_t)(sqr(nrh-nrl+1) + (nrh-nrl+1))/2.,sizeof(int));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in imatrix()");
  
  j = 0;
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(int *) &(chunk[j]);
    m[i] -= ncl + (i-nrl);
    j = j + (nch-ncl+1) - (i-nrl);
  }
  
  return m;

}
/******************************************************************************
  my_matrix_diag; allocates the memory for a diagonal matrix, i.e., 
  only the diagonal. Can be freed with my_free_matrix
  ****************************************************************************/
Matrix
my_matrix_diag(int nrl, int nrh, int ncl, int nch)

{
  int      i,j;
  double **m;
  double  *chunk;
  int      info = FALSE;
  
  if (nrh-nrl != nch-ncl)
    my_nrerror("diagonal matrices must be square!\n");

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (double **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(double*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (double *) my_calloc_track((size_t) N_MAT_INFO,sizeof(double));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_DIAG;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }
  
  chunk = (double *) my_calloc_track((size_t) (nrh-nrl+1),sizeof(double));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  j = 0;
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(double *) &(chunk[j]);
    m[i] -= ncl + (i-nrl);
    j = j + 1;
  }
  
  return m;

}


/******************************************************************************
  my_imatrix_diag; allocates the memory for a diagonal matrix, i.e., 
  only the diagonal. Can be freed with my_free_imatrix
  ****************************************************************************/
iMatrix
my_imatrix_diag(int nrl, int nrh, int ncl, int nch)

{
  int      i,j;
  int **m;
  int  *chunk;
  int      info = FALSE;
  
  if (nrh-nrl != nch-ncl)
    my_nrerror("diagonal matrices must be square!\n");

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (int **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(int*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (int *) my_calloc_track((size_t) N_MAT_INFO,sizeof(int));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_DIAG;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }
  
  chunk = (int *) my_calloc_track((size_t) (nrh-nrl+1),sizeof(int));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  j = 0;
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(int *) &(chunk[j]);
    m[i] -= ncl + (i-nrl);
    j = j + 1;
  }
  
  return m;

}


/*****************************************************************************
  utility program my_free_matrix; adjusted to my special matrix() program
  ***************************************************************************/
void 
my_free_matrix(Matrix mat, int nrl, int nrh, int ncl, int nch)

{
  int i;

  free((char*) &(mat[nrl][ncl]));
  if (nrl==1 && ncl==1) {
    free((char*) mat[0]);
    free((char*) mat);
  } else {
    free((char*) (mat+nrl));
  }

}

/******************************************************************************
  utility program my_matrix: note: this is a modified version of
  the normal matrix() numerical recipe version. It allocates
  the memory for the matrix in one chunk, such that it can
  be treated as consecutive memory in write() or DSP programs
  ****************************************************************************/
fMatrix
my_fmatrix(int nrl, int nrh, int ncl, int nch)
{

  int      i;
  float **m;
  float  *chunk;
  int      info = FALSE;

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (float **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(float*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (float *) my_calloc_track((size_t) N_MAT_INFO,sizeof(float));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_FULL;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }
    
  chunk = (float *) my_calloc_track((size_t) (nrh-nrl+1) * (nch-ncl+1),sizeof(float));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(float *) &(chunk[(i-nrl)*(nch-ncl+1)]);
    m[i] -= ncl;
  }
  
  return m;

}

/******************************************************************************
  utility program my_free_fmatrix; adjusted to my special matrix() program
  ****************************************************************************/
void 
my_free_fmatrix(fMatrix mat, int nrl, int nrh, int ncl, int nch)

{
  int i;

  free((char*) &(mat[nrl][ncl]));
  if (nrl==1 && ncl==1) {
    free((char*) mat[0]);
    free((char*) mat);
  } else {
    free((char*) (mat+nrl));
  }

}
/******************************************************************************
  utility program my_imatrix: note: this is a modified version of
  the normal matrix() numerical recipe version. It allocates
  the memory for the matrix in one chunk, such that it can
  be treated as consecutive memory in write() or DSP programs
  ****************************************************************************/
iMatrix
my_imatrix(int nrl, int nrh, int ncl, int nch)

{

  int      i;
  int    **m;
  int     *chunk;
  int      info = FALSE;

  if (nrl==1 && ncl == 1) {
    info = TRUE;
  }

  m    = (int **) my_calloc_track((size_t) (nrh-nrl+1+info),sizeof(int*));
  if (m==NULL)
    my_nrerror("allocation failure 1 in matrix()");

  if (info) {

    m[0] = (int *) my_calloc_track((size_t) N_MAT_INFO,sizeof(int));
    if (m[0]==NULL)
      my_nrerror("allocation failure 0 in matrix()");
    m[0][MAT_TYPE] = IS_FULL;
    m[0][NR]       = nrh-nrl+1;
    m[0][NC]       = nch-ncl+1;

  } else {
  
    m -= nrl;

  }
    
  chunk = (int *) my_calloc_track( (size_t) (nrh-nrl+1) * (nch-ncl+1),sizeof(int));
  if (chunk==NULL)
    my_nrerror("allocation failure 2 in matrix()");
  
  for(i=nrl;i<=nrh;i++) {
    m[i]=(int *) &(chunk[(i-nrl)*(nch-ncl+1)]);
    m[i] -= ncl;
  }
  
  return m;

}

/******************************************************************************
  utility program my_free_fmatrix; adjusted to my special matrix() program
  ****************************************************************************/
void 
my_free_imatrix(iMatrix mat, int nrl, int nrh, int ncl, int nch)

{
  int i;

  free((char*) &(mat[nrl][ncl]));
  if (nrl==1 && ncl==1) {
    free((char*) mat[0]);
    free((char*) mat);
  } else {
    free((char*) (mat+nrl));
  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_ran0
 \date  11/10/91
 
 \remarks 
 
 this is the numerical recipes function to smoothen the problems with
 the C-function rand()
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     seed		: used for seeding rand() when function is called the first
 time, or when seed < 0!
 
 note: the returned number is between 0.0 and 1.0
 
 ******************************************************************************/
double
my_ran0(int *idum)
     
{

  static double y, maxran, v[98];
  double dum;
  static int iff=0;
  int j;
  unsigned i,k;
  void my_nrerror();


  if (*idum < 0 || iff == 0) {

    iff = 1;
    maxran = MY_RAND_MAX+1.0;
    srand(*idum);
    *idum = 1;
    for (j=1; j<=97; j++)
      dum = rand()%MY_RAND_MAX;
    for (j=1; j<=97; j++)
      v[j] = rand()%MY_RAND_MAX; /* ensure sun-compatib. */
    y = rand()%MY_RAND_MAX;

  }

  j = 1. + 97.0 * y/maxran;

  if (j > 97 || j < 1)
    my_nrerror("my_ran0: This is nonsense\n");
  y = v[j];
  v[j] = rand()%MY_RAND_MAX;

  return (y/maxran);
}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_gasdev
 \date  11/10/91
 
 \remarks 
 
 this is the numerical recipes function which returns a normally
 distributed deviate with zero mean and unit variance, using my_ran0
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     seed		: used for seeding rand() when function is called the first
 time, or when seed < 0!
 
 
 ******************************************************************************/
double
my_gasdev(int *idum)
     
{

  static int iset=0;
  static double gset;
  double fac, r, v1, v2;

  if (iset == 0) {

    do {
      v1 = 2.0 * my_ran0(idum) - 1.0;
      v2 = 2.0 * my_ran0(idum) - 1.0;
      r = sqr(v1) + sqr(v2);
    }
    while (r >= 1.0);

    fac = sqrt(-2.0 * log(r)/r);
    gset = v1 * fac;
    iset = 1;
    return v2*fac;

  }
  else {

    iset = 0;
    return gset;

  }
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_a_float
 \date  11/10/93
 
 \remarks 
 
 read a float type variabe with a comment in front; this important
 for checking parameter readings in long parameter lists
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     silent   : don't print message
 \param[in]     fd       : pointer to file where line should be read
 \param[in]     string   : string to be printed with read variable
 \param[out]    pvalue   : pointer to variable to be returned

 ******************************************************************************/
void
read_a_float(int silent, FILE *fd, char *string, float *pvalue )

{
  float value;
  char scanf_string1[ 1000 ];
  char scanf_string2[ 1000 ];
  int rc;

  rc = fscanf( fd, "%s %s %f", scanf_string1, scanf_string2, &value );
  *pvalue = (float) value;
  if (!silent)  
    printf( "reading %s: %s %s %g\n", string, scanf_string1, scanf_string2,
	   *pvalue );
}

/*!*****************************************************************************
 *******************************************************************************
\note  read_a_string
\date  11/10/93

\remarks 

      reads a string type variabe with a comment in front; this important
      for checking parameter readings in long parameter lists

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     silent   : don't print message
 \param[in]     fd       : pointer to file where line should be read
 \param[in]     string   : string to be printed with read variable
 \param[out]    pvalue   : pointer to variable to be returned
      
 ******************************************************************************/
void
read_a_string(int silent, FILE *fd, char *string, char *svalue )
     
{
  float value;
  char scanf_string1[ 1000 ];
  char scanf_string2[ 1000 ];
  int rc;

  rc = fscanf( fd, "%s %s %s", scanf_string1, scanf_string2, svalue );
  if (!silent)  
    printf( "reading %s: %s %s %s\n", string, scanf_string1, scanf_string2,
	   svalue );
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_a_double
 \date  11/10/93
 
 \remarks 
 
 read a double type variabe with a comment in front; this important
 for checking parameter readings in long parameter lists
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     silent   : don't print message
 \param[in]     fd       : pointer to file where line should be read
 \param[in]     string   : string to be printed with read variable
 \param[out]    pvalue   : pointer to variable to be returned
 
 ******************************************************************************/
void
read_a_double(int silent,  FILE *fd, char *string, double *pvalue )

{
  double value;
  char scanf_string1[ 1000 ];
  char scanf_string2[ 1000 ];
  int rc;

  rc = fscanf( fd, "%s %s %lf", scanf_string1, scanf_string2, &value );
  *pvalue = (double) value;
  if (!silent)
    printf( "reading %s: %s %s %f\n", string, scanf_string1, scanf_string2,
	 *pvalue );
}


/*!*****************************************************************************
 *******************************************************************************
 \note  read_an_int
 \date  11/10/93
 
 \remarks 
 
 read an int type variabe with a comment in front; this important
 for checking parameter readings in long parameter lists
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     silent   : don't print message
 \param[in]     fd       : pointer to file where line should be read
 \param[in]     string   : string to be printed with read variable
 \param[out]    pint     : pointer to variable to be returned
      
 ******************************************************************************/
void
read_an_int(int silent, FILE *fd, char *string, int  *pint )
     
{
  int value;
  char scanf_string1[ 1000 ];
  char scanf_string2[ 1000 ];
  int rc;

  rc = fscanf( fd, "%s %s %d", scanf_string1, scanf_string2, &value );
  *pint = (int) value;
  if (!silent) 
    printf( "reading %s: %s %s %d\n", string, scanf_string1, scanf_string2,
	   *pint );
}

/*!*****************************************************************************
 *******************************************************************************
 \note  read_an_int
 \date  11/10/93
 
 \remarks 
 
 read an int type variabe with a comment in front; this important
 for checking parameter readings in long parameter lists
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     silent   : don't print message
 \param[in]     fd       : pointer to file where line should be read
 \param[in]     string   : string to be printed with read variable
 \param[out]    pint     : pointer to variable to be returned
 
 ******************************************************************************/
void
read_a_long(int silent, FILE *fd, char *string, long  *pint )
     
{
  long value;
  char scanf_string1[ 1000 ];
  char scanf_string2[ 1000 ];
  int rc;

  rc = fscanf( fd, "%s %s %ld", scanf_string1, scanf_string2, &value );
  *pint = (long) value;
  if (!silent) 
    printf( "reading %s: %s %s %ld\n", string, scanf_string1, scanf_string2,
	   *pint );
}

/*!*****************************************************************************
 *******************************************************************************
 \note  fopen_strip
 \date  March 94
 
 \remarks 
 
 opens a file with a given file name for read/write, but strips away
 all C/C++ convention comments in this file. The pointer returned will
 point to a temp file afterwards which will be automatically removed
 afterwards. The orginal file remains unchanged.

 Note: - we also strip ';' '=' ':' for easier parsing
       - #include "filename" is permitted to include other files
       - the # sign is a reserved character
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]   filename : name of file to be read
 
 ******************************************************************************/
FILE *
fopen_strip(char *filename)
     
{
  FILE *temp;

  // open a temp file 
  if ((temp = tmpfile())==NULL)
    return NULL;

  // read the files and included files recursively
  fopen_strip_recursive(filename,temp);
 
  // reset the tempfile to the beginning
  rewind(temp);

  return temp;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fopen_strip_recursive
 \date  March 94
 
 \remarks 
 
 recursive program to read all files and included files for fopen_strip
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]   filename : name of file to be read
 \param[in]   temp     : file pointer for output file
 
 ******************************************************************************/
static void
fopen_strip_recursive(char *filename, FILE *temp)
{

  enum Comments {
    C_NONE,
    C_PLAIN,
    C_PLUS
  };

  FILE *infile;
  int	i,k,rc,last_rc,j;
  int   skip = C_NONE;
  int   wait = 2;
  int   dummy;
  char  keyword[100];
  char  fname[100+1];

  infile = fopen(filename,"r");
  if (infile == NULL) {
    printf("Error: fopen_strip couldn't read file >%s<",filename);
    return;
  }
  

  // clean away comments and add new include files
  last_rc = EOF;
  while ((rc=fgetc(infile)) != EOF) {
    
    --wait;
    
    if (skip == C_NONE && rc == '*' && last_rc == '/') {
      skip = C_PLAIN;
    } else if (skip == C_NONE && rc == '/' && last_rc == '/') {
      skip = C_PLUS;
    } else if (skip == C_PLAIN && rc == '/' && last_rc == '*') {
      skip = C_NONE;
      wait = 2;
    } else if (skip == C_PLUS && rc == '\n') {
      skip = C_NONE;
      wait = 2;
    }

    // check for # signs
    if ( rc == '#' && skip == C_NONE && wait <= 0) {
      k=fscanf(infile,"%s",keyword);
      if (strcmp(keyword,"include")==0) {
	// find the string between double quotes
	while ((rc=fgetc(infile)) != '"') { // find the first double quote
	  if (rc == EOF) {
	    printf("Error: fopen_strip: could not parse include file name (first \")\n");
	    return;
	  }
	}
	j=0;
	while ((rc=fgetc(infile)) != '"') { // read until the next double quote
	  if (rc == EOF) {
	    printf("Error: fopen_strip: could not parse include file name (second \")\n");
	    return;
	  }
	  if (j<100)
	    fname[j++] = rc;
	}
	fname[j]='\0';

	fopen_strip_recursive(fname,temp);
      }
      continue;
    }
    
    if (skip == C_NONE && last_rc != EOF && wait <= 0) {
      if (last_rc != ';' && last_rc != ',' && last_rc != '=') {
	fputc(last_rc,temp);
      }
    }
    
    last_rc = rc;
    
  }
  
  fputc(last_rc,temp);
  fclose(infile);

}

/*!*****************************************************************************
 *******************************************************************************
 \note  remove_temp_file
 \date  March 94
 
 \remarks 
 
 removes the current tempfile
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
void
remove_temp_file(void)
     
{
  if (strcmp(tempfile,"")!=0) {
    remove(tempfile);
    strcpy(tempfile,"");
  }
}

/*!*****************************************************************************
 *******************************************************************************
 \note  fopen_strip_comments
 \date  March 94
 
 \remarks 
 
 opens a file with a given file name for read/write, but strips
 away all C-convention comments in this file. The pointer 
 returned will point to a temp.temp file afterwards which
 can be inspected or removed afterwards. The orginal file
 remains unchanged
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 none
 
 ******************************************************************************/
FILE *
fopen_strip_comments(char *filename)
     
{

  FILE *infile, *temp;
  int	i,k,rc,last_rc;
  int   skip = FALSE;
  int   wait = 2;
  int   dummy;
  int   fd;


  infile = fopen(filename,"r");

  if (infile == NULL) {

    return NULL;

  }

  /* clean away any comment lines as well as commas and semicolons */

  /*sprintf(tempfile,"%s%ld",TEMPFILE,random_number(0,100000));*/
  sprintf(tempfile,"%sXXXXXXXX",TEMPFILE);
  if ((fd = mkstemp(tempfile))==-1 ||
      (temp = fdopen(fd,"w+"))==NULL) {
    strcpy(tempfile,"");
    return NULL;
  }
  last_rc = EOF;
  
  while ((rc=fgetc(infile)) != EOF) {
    
    --wait;
    
    if (rc == '*' && last_rc == '/') {
      skip = TRUE;
    } else if (rc == '/' && last_rc == '*') {
      skip = FALSE;
      wait = 2;
    }
    
    if (!skip && last_rc != EOF && wait <= 0) {
      if (last_rc != ';' && last_rc != ',')
	fputc(last_rc,temp);
    }
    
    last_rc = rc;
    
  }
  
  fputc(last_rc,temp);
  fclose(infile);
  rewind(temp);

  return temp;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_mat
 \date  August 17, 92
 
 \remarks 
 
 just prints the given matrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 
 ******************************************************************************/
void  
print_mat(char *comm, Matrix a)
     
{

  int i,j;
  int nr,nc;

  nr = a[0][NR];
  nc = a[0][NC];

  printf("Matrix >%s< :  [%dx%d] (type=%d)\n",comm,nr,nc,(int)a[0][MAT_TYPE]);

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	printf("% 8.4f ",a[i][j]);
      }
      printf("\n");
    }
    printf("\n");

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	if (i==j) {
	  printf("% 8.4f ",a[i][j]);
	} else {
	  printf("% 8.4f ",0.0);
	}
      }
      printf("\n");
    }
    printf("\n");

    break;


  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	if (i<=j) {
	  printf("% 8.4f ",a[i][j]);
	} else {
	  printf("% 8.4f ",a[j][i]);
	}
      }
      printf("\n");
    }
    printf("\n");

    break;


  default:

    exit(-22);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_imat
 \date  May 2006
 
 \remarks 
 
 just prints the given imatrix
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 
 ******************************************************************************/
void  
print_imat(char *comm, iMatrix a)
     
{

  int i,j;
  int nr,nc;

  nr = a[0][NR];
  nc = a[0][NC];

  printf("Matrix >%s< :\n",comm);

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	printf("% 6d ",a[i][j]);
      }
      printf("\n");
    }
    printf("\n");

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	if (i==j) {
	  printf("% 6d ",a[i][j]);
	} else {
	  printf("% 6d ",0);
	}
      }
      printf("\n");
    }
    printf("\n");

    break;


  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      printf("          ");
      for (j=1; j<=nc; ++j) {
	if (i<=j) {
	  printf("% 6d ",a[i][j]);
	} else {
	  printf("% 6d ",a[j][i]);
	}
      }
      printf("\n");
    }
    printf("\n");

    break;


  default:

    exit(-22);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_mat
 \date  August 17, 92
 
 \remarks 
 
 just prints the given matrix to a file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 
 ******************************************************************************/
void  
fprint_mat(FILE *fp, char *comm, Matrix a)
     
{

  int i,j;
  int nr,nc;

  nr = a[0][NR];
  nc = a[0][NC];

  if (strcmp(comm,"") != 0)
    fprintf(fp,"Matrix >%s< :\n",comm);

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	fprintf(fp,"% 8.4f ",a[i][j]);
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	if (i==j) {
	  fprintf(fp,"% 8.4f ",a[i][j]);
	} else {
	  fprintf(fp,"% 8.4f ",0.0);
	}
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	if (i<=j) {
	  fprintf(fp,"% 8.4f ",a[i][j]);
	} else {
	  fprintf(fp,"% 8.4f ",a[j][i]);
	}
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  default:

    exit(-22);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_imat
 \date  August 17, 92
 
 \remarks 
 
 just prints the given matrix to a file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 
 ******************************************************************************/
void  
fprint_imat(FILE *fp, char *comm, iMatrix a)
     
{

  int i,j;
  int nr,nc;

  nr = a[0][NR];
  nc = a[0][NC];

  if (strcmp(comm,"") != 0)
    fprintf(fp,"Matrix >%s< :\n",comm);

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	fprintf(fp,"% d ",a[i][j]);
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  case IS_DIAG:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	if (i==j) {
	  fprintf(fp,"% d ",a[i][j]);
	} else {
	  fprintf(fp,"% d ",0);
	}
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  case IS_SYMM:

    for (i=1; i<=nr; ++i) {
      fprintf(fp,"          ");
      for (j=1; j<=nc; ++j) {
	if (i<=j) {
	  fprintf(fp,"% d ",a[i][j]);
	} else {
	  fprintf(fp,"% d ",a[j][i]);
	}
      }
      fprintf(fp,"\n");
    }
    fprintf(fp,"\n");

    break;


  default:

    exit(-22);

  }

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_mat_size
 \date  August 17, 92
 
 \remarks 
 
 just prints the given matrix to a file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 \param[in]     nr    : number of rows
 \param[in]     nc    : number of columns
 
 ******************************************************************************/
void  
fprint_mat_size(FILE *fp, char *comm, Matrix a, int nr, int nc)
     
{

  int i,j;

  for (i=1; i<=nr; ++i) {
    fprintf(fp,"          ");
    for (j=1; j<=nc; ++j) {
      fprintf(fp,"% 8.4f ",a[i][j]);
    }
    fprintf(fp,"\n");
  }

  fprintf(fp,"\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fwrite_mat
 \date  August 17, 92
 
 \remarks 
 
 writes the given matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : an (open) file pointer
 \param[in]     a     : the matrix to be written
 
 ******************************************************************************/
int
fwrite_mat(FILE *fp, Matrix a)
     
{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_double(a[0][i]);
#endif

  num = N_MAT_INFO;  
  if (fwrite(&(a[0][0]),sizeof(double),num,fp)!= num) {
    printf( "cannot fwrite matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_double(a[0][i]);
  mat_equal_apply_math(a,byteswap_double,a);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fwrite(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fwrite(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fwrite(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  default:

    exit(-22);

  }

#ifdef BYTESWAP
  mat_equal_apply_math(a,byteswap_double,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fwrite_fmat
 \date  April 1999
 
 \remarks 
 
 writes the given matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : an (open) file pointer
 \param[in]     a     : the matrix to be written
 
 ******************************************************************************/
int
fwrite_fmat(FILE *fp, fMatrix a)
     
{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_float(a[0][i]);
#endif

  num = N_MAT_INFO;
  if (fwrite(&(a[0][0]),sizeof(float),num,fp)!= num) {
    printf( "cannot fwrite matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_float(a[0][i]);
  fmat_equal_apply_math(a,byteswap_float,a);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fwrite(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fwrite(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fwrite(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  default:

    exit(-22);

  }

#ifdef BYTESWAP
  fmat_equal_apply_math(a,byteswap_float,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fwrite_imat
 \date  April 2011
 
 \remarks 
 
 writes the given integer matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : an (open) file pointer
 \param[in]     a     : the matrix to be written
 
 ******************************************************************************/
int
fwrite_imat(FILE *fp, iMatrix a)

{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_int(a[0][i]);
#endif

  num = N_MAT_INFO;
  if (fwrite(&(a[0][0]),sizeof(int),num,fp)!= num) {
    printf( "cannot fwrite matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_int(a[0][i]);
  imat_equal_apply_math(a,byteswap_int,a);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fwrite(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fwrite(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fwrite(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fwrite matrix.\n" );
      return FALSE;
    }

    break;


  default:

    exit(-22);

  }

#ifdef BYTESWAP
  imat_equal_apply_math(a,byteswap_int,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fread_mat
 \date  August 17, 92
 
 \remarks 
 
 reads the given matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a     : the matrix to be read
 \param[in]     fp    : an (open) file pointer
 
 ******************************************************************************/
int
fread_mat(FILE *fp, Matrix a)
     
{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

  num = N_MAT_INFO;
  if (fread(&(a[0][0]),sizeof(double),num,fp)!= num) {
    printf( "cannot fread matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_double(a[0][i]);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fread(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fread(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fread(&(a[1][1]),sizeof(double),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  default:

    return FALSE;

  }

#ifdef BYTESWAP
  mat_equal_apply_math(a,byteswap_double,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fread_fmat
 \date  August 17, 92
 
 \remarks 
 
 reads the given matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a     : the matrix to be read
 \param[in]     fp    : an (open) file pointer
 
 ******************************************************************************/
int
fread_fmat(FILE *fp, fMatrix a)
     
{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

  num = N_MAT_INFO;
  if (fread(&(a[0][0]),sizeof(float),num,fp)!= num) {
    printf( "cannot fread matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_float(a[0][i]);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fread(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fread(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fread(&(a[1][1]),sizeof(float),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  default:

    return FALSE;

  }

#ifdef BYTESWAP
  fmat_equal_apply_math(a,byteswap_float,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fread_imat
 \date  August 17, 92
 
 \remarks 
 
 reads the given matrix as binary to given file pointer
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a     : the matrix to be read
 \param[in]     fp    : an (open) file pointer
 
 ******************************************************************************/
int
fread_imat(FILE *fp, iMatrix a)
     
{

  int i,j;
  int nr,nc;
  int num;

  nr = a[0][NR];
  nc = a[0][NC];

  num = N_MAT_INFO;

  if (fread(&(a[0][0]),sizeof(int),num,fp)!= num) {
    printf( "cannot fread matrix.\n" );
    return FALSE;
  }
  
#ifdef BYTESWAP
  for (i=0;i<N_MAT_INFO;++i)
    a[0][i]=byteswap_int(a[0][i]);
#endif

  switch ((int)a[0][MAT_TYPE]) {

  case IS_FULL:

    num = a[0][NR]*a[0][NC];
    if (fread(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_DIAG:

    num = a[0][NR];
    if (fread(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  case IS_SYMM:

    num = a[0][NR]*(a[0][NR] + 1)/2;
    if (fread(&(a[1][1]),sizeof(int),num,fp)!= num) {
      printf( "cannot fread matrix.\n" );
      return FALSE;
    }

    break;


  default:

    return FALSE;

  }

#ifdef BYTESWAP
  imat_equal_apply_math(a,byteswap_int,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_mat_size
 \date  August 17, 92
 
 \remarks 
 
 just prints the given matrix with limits
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the matrix to be printed
 \param[in]     nr    : number of rows
 \param[in]     nc    : number of columns
 
 ******************************************************************************/
void  
print_mat_size(char *comm, Matrix a, int nr, int nc)
     
{

  int i,j;

  printf("Matrix >%s< :\n",comm);

  for (i=1; i<=nr; ++i) {
    printf("          ");
    for (j=1; j<=nc; ++j) {
      printf("% 8.4f ",a[i][j]);
    }
    printf("\n");
  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_vec
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 
 ******************************************************************************/
void  
print_vec(char *comm, Vector a)
     
{

  int i,j;

  printf("Vector >%s< : [%d]\n",comm,(int)a[NR]);

  for (i=1; i<=a[NR]; ++i) {
    printf("          % 8.4f\n",a[i]);
  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_ivec
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector to a  file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 
 ******************************************************************************/
void  
fprint_ivec(FILE *fp, char *comm, iVector a)
     
{

  int i,j;

  if (strcmp(comm,"")!=0)
    fprintf(fp,"Vector >%s< :\n",comm);

  for (i=1; i<=a[NR]; ++i) {
    fprintf(fp,"          %5d\n",a[i]);
  }
  fprintf(fp,"\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_ivec_size
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector to a  file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 \param[in]     nr    : number of elements in vector
 
 ******************************************************************************/
void  
fprint_ivec_size(FILE *fp, char *comm, iVector a, int nr)
     
{

  int i,j;

  if (strcmp(comm,"")!=0)
    fprintf(fp,"Vector >%s< :\n",comm);

  for (i=1; i<=nr; ++i) {
    fprintf(fp,"          %5d\n",a[i]);
  }
  fprintf(fp,"\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_vec
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector to a  file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 
 ******************************************************************************/
void  
fprint_vec(FILE *fp, char *comm, Vector a)
     
{

  int i,j;

  if (strcmp(comm,"")!=0)
    fprintf(fp,"Vector >%s< :\n",comm);

  for (i=1; i<=a[NR]; ++i) {
    fprintf(fp,"          % 8.4f\n",a[i]);
  }
  fprintf(fp,"\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fprint_vec_size
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector to a  file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : open file pointer
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 \param[in]     nr    : number of elements in vector
 
 ******************************************************************************/
void  
fprint_vec_size(FILE *fp, char *comm, Vector a, int nr)
     
{

  int i,j;

  if (strcmp(comm,"")!=0)
    fprintf(fp,"Vector >%s< :\n",comm);

  for (i=1; i<=nr; ++i) {
    fprintf(fp,"          % 8.4f\n",a[i]);
  }
  fprintf(fp,"\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fwrite_vec
 \date  August 17, 92
 
 \remarks 
 
 writes vector in binary out to file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : an (open) file pointer
 \param[in]     a     : the vector to be written
 
 ******************************************************************************/
int
fwrite_vec(FILE *fp, Vector a)
     
{
  int num;
  
  num = a[NR]+1;

#ifdef BYTESWAP
  vec_equal_apply_math(a,byteswap_double,a);
  a[NR]=byteswap_double(a[NR]);
#endif

  if (fwrite(&(a[0]),sizeof(double),num,fp)!= num) {
    printf( "cannot fwrite vector\n" );
    return FALSE;
  }

#ifdef BYTESWAP
  a[NR]=byteswap_double(a[NR]);
  vec_equal_apply_math(a,byteswap_double,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fwrite_ivec
 \date  August 17, 92
 
 \remarks 
 
 writes vector in binary out to file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     fp    : an (open) file pointer
 \param[in]     a     : the vector to be written
 
 ******************************************************************************/
int
fwrite_ivec(FILE *fp, iVector a)
     
{
  int num;
  
  num = a[NR]+1;

#ifdef BYTESWAP
  ivec_equal_apply_math(a,byteswap_int,a);
  a[NR]=byteswap_int(a[NR]);
#endif

  if (fwrite(&(a[0]),sizeof(int),num,fp)!= num) {
    printf( "cannot fwrite vector\n" );
    return FALSE;
  }

#ifdef BYTESWAP
  a[NR]=byteswap_int(a[NR]);
  ivec_equal_apply_math(a,byteswap_int,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fread_vec
 \date  August 17, 92
 
 \remarks 
 
 reads vector in binary out to file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a     : the vector to be read
 \param[in]     fp    : an (open) file pointer
 
 ******************************************************************************/
int
fread_vec(FILE *fp, Vector a)
     
{
  int num;
  
  num = a[NR]+1;
  if (fread(&(a[0]),sizeof(double),num,fp)!= num) {
    printf( "cannot fread vector\n" );
    return FALSE;
  }

#ifdef BYTESWAP
  a[0]=byteswap_double(a[0]);
  vec_equal_apply_math(a,byteswap_double,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  fread_ivec
 \date  August 17, 92
 
 \remarks 
 
 reads vector in binary out to file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     a     : the vector to be read
 \param[in]     fp    : an (open) file pointer
 
 ******************************************************************************/
int
fread_ivec(FILE *fp, iVector a)
     
{
  int num;
  
  num = a[NR]+1;
  if (fread(&(a[0]),sizeof(int),num,fp)!= num) {
    printf( "cannot fread vector\n" );
    return FALSE;
  }

#ifdef BYTESWAP
  a[0]=byteswap_int(a[0]);
  ivec_equal_apply_math(a,byteswap_int,a);
#endif

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_vec_size
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 \param[in]     nr    : number of rows of a
 
 ******************************************************************************/
void  
print_vec_size(char *comm, Vector a, int nr)
     
{

  int i,j;
  
  printf("Vector >%s< :\n",comm);

  for (i=1; i<=nr; ++i) {
    printf("          % 8.4f\n",a[i]);
  }
  printf("\n");

}

/*!*****************************************************************************
 *******************************************************************************
 \note  print_ivec
 \date  August 17, 92
 
 \remarks 
 
 just prints the given vector
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     com   : comment
 \param[in]     a     : the vector to be printed
 
 ******************************************************************************/
void  
print_ivec(char *comm, iVector a)
     
{

  int i,j;

  printf("Vector >%s< :\n",comm);

  for (i=1; i<=a[NR]; ++i) {
    printf("          % 5d\n",a[i]);
  }
  printf("\n");
  

}

/*!*****************************************************************************
 *******************************************************************************
 \note  interpolate
 \date  August 17, 92
 
 \remarks 
 
 multidimensional interpolation on a grid: given n_dim_in
 input dimensions and the vectors of the grid points
 as well as a lookup point, the function generates
 the multidimensional output (n_dim_out) by means of multi-
 linear interpolation
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     v_in    : vector with grid points
 \param[in]     n_in    : how many input dimensions
 \param[in]     v_out   : vector with corresponding output points
 \param[in]     n_out   : how many output dimensions
 \param[in]     v_q     : the input query point
 \param[out]    out     : the interpolated point
 
 NOTE: the function returns FALSE if query point lies outside
 of grid rectangle
 
 ******************************************************************************/
#define MAX_IN 50
#define MAX_PTS 100
int
interpolate(double **v_in, int n_in, double **v_out, int n_out, 
	    double *v_q, double *out)
     
{

  int i,j,k;
  static double *v_max;
  static double *v_min;
  static double **w;
  static int    **bin;
  int    n_pts;
  static int    *num;
  static double *fac;
  double aux;
  static int max_in = 0, max_pts = 0;

  n_pts = 1 << n_in;  /* n_in and n_pts are coupled */

  /* do we need more working memory */

  if (n_in > max_in) {

    if (max_in != 0) {
      my_free_vector(v_max,1,n_in);
      my_free_vector(v_min,1,n_in);
      my_free_matrix(w,1,n_in,0,1);
      my_free_imatrix(bin,1,n_pts,1,n_in);
      my_free_ivector(num,1,n_pts);
      my_free_vector(fac,1,n_pts);
    }

    v_max = my_vector(1,n_in);
    v_min = my_vector(1,n_in);
    w     = my_matrix(1,n_in,0,1);
    bin   = my_imatrix(1,n_pts,1,n_in);
    num   = my_ivector(1,n_pts);
    fac   = my_vector(1,n_pts);

    max_in = n_in;
    max_pts = n_pts;

  }
  

  /* determine max an min values of input vector */
   
  for (j=1; j<=n_in; ++j) {
    v_max[j] = -1.e30;
    v_min[j] = +1.e30;
    for (i=1; i<=n_pts; ++i) {
      if (v_in[i][j] > v_max[j]) v_max[j] = v_in[i][j];
      if (v_in[i][j] < v_min[j]) v_min[j] = v_in[i][j];
    }
  }

  /* determine the "weight" of the query point */

  for (i=1; i<=n_in; ++i) {
    aux = v_max[i]-v_min[i];
    if (fabs(aux) < 1.e-10) aux = macro_sign(aux)*1.e-10;
    w[i][1] = (v_q[i] - v_min[i])/aux;
    /* this value must be between 0 and 1 */
    if (w[i][1] > 1 || w[i][1] < 0) return FALSE;
    w[i][0] = 1.-w[i][1];
  }

  /* determine the "binary" vector for each grid point */

  for (i=1; i<=n_pts; ++i) {
    for (j=1; j<=n_in; ++j) {
      if (v_in[i][j] == v_max[j]) {
	bin[i][j] = 1;
      } else {
	bin[i][j] = 0;
      }
    }
  }

  /* some computation can be summerized before the final interpolation */

  for (j=1; j<=n_pts; ++j) {
    fac[j] = 1.;
    for (k=1; k<=n_in; ++k) {
      fac[j] *= w[k][ bin[j][k] ];
    }
  }
  
 
  /* do the interpolation for every output dimension */

  for (i=1; i<=n_out; ++i) {
    out[i] = 0.0;
    for (j=1; j<=n_pts; ++j) {
      out[i] += v_out[j][i] * fac[j];
    }
  }

  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  beep
 \date  August 17, 92
 
 \remarks 
 
 make the terminal beep
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 none
 
 ******************************************************************************/
int
beep(int how_many)
     
{

  int i;

  for (i=1; i<=how_many; ++i) printf("%c",0x7);
  fflush(stdout);
  return TRUE;

}

/*!*****************************************************************************
 *******************************************************************************
 \note  my_isnan
 \date  August 17, 92
 
 \remarks 
 
 returns TRUE if given number is an nan
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 none
 
 ******************************************************************************/
int
my_isnan(double x)
     
{

  int non_neg = x >= 0.0;
  int non_pos = x <= 0.0;
  return( !(non_neg || non_pos) );
}

/*!*****************************************************************************
 *******************************************************************************
 \note  find_keyword
 \date  October, 1995
 
 \remarks 
 
 find a key word in the file given by fp and positions the read cursor
 right after the keyword
 
 *******************************************************************************
 Function Parameters: [in]=input,[out]=output
 
 \param[in]     fp       : file pointer
 \param[in]     word     : keyword to be found
 
 ******************************************************************************/
int
find_keyword(FILE *fp, char *name)

{

  int  i,j,c;
  int  rc = TRUE;
  char string[strlen(name)*2];
  int  l;
  char sep[]={' ','\n',':',',',';','=','\t','\0'};

  rewind(fp);
  l  = strlen(name);
  i  = 0;

  while (rc != EOF) {

    rc=fgetc(fp);
    if ( rc != EOF ) {

      string[i++] = rc;
      string[i]   = '\0';

      if ( strstr(string,name) != NULL) {
	// wait for one more character to judge whether this string
	// has the correct end delimiters
	if (strchr(sep,string[i-1]) != NULL) {
	  // now check for preceeding delimiter

	  if (i-l-2 < 0) // this means "name" was the first string in file
	    return TRUE;
	  else if (strchr(sep,string[i-l-2]) != NULL) //otherwise check delim
	    return TRUE;
	}
      }

      if (i >= 2*l-1) {
	strcpy(string,&(string[i-l]));
	i = strlen(string);
      }

    }

  }

  return FALSE;

}

  
/*!*****************************************************************************
 *******************************************************************************
 \note  maxMin
 \date  August, 96
 
 \remarks 
 
 updates max and min variables from a given value
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[in]     x        : the input
 \param[in,out] max_x    : the current max, overwritten if needed
 \param[in,out] min_x    : the current min, overwritten if needed


 ******************************************************************************/
void
maxMin(double x, double *max_x, double *min_x)

{
  
  if (x > *max_x) *max_x = x;
  if (x < *min_x) *min_x = x;

}

#ifndef c40  

#endif
#ifdef UNIX
/*!*****************************************************************************
 *******************************************************************************
 \note  getFile
 \date  August 17, 92
 
 \remarks 
 
 prompts user for a filename and checks the existence of the file
 
 *******************************************************************************
 Parameters:  (i/o = input/output)
 
 \param[out]    fname : the name of the file
 
 ******************************************************************************/
int 
getFile(char *fname)

{
  
  FILE *temp;
  char temp_name[100];
  char response[100];
  int rc;
  
 AGAIN:
  printf("\nInput File Name:\n");
  if (!get_string("filename\0",fname,temp_name)) return FALSE;
  
  /* check whether file already exists */

  if (temp_name[0] == '?') {

    rc = system("ls");
    goto AGAIN;

  }

  if ((temp = fopen(temp_name,"r")) == NULL) {
    
    printf("Error: File with file name >%s< does not exist\n",temp_name);
    goto AGAIN;

  }

  fclose(temp);
  strcpy(fname,temp_name);

  return TRUE;
}
#endif

/*!*****************************************************************************
 *******************************************************************************
 \note  mat_add_shape
 \date  May 1999
 
 \remarks 
 
 adds a row to the given matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     nr              : number of rows to be added
 \param[in]     nc              : number of columns to be added 

 ******************************************************************************/
int
mat_add_shape(Matrix *a, int nr, int nc)
{
  int      i,j,m;
  int      ar,ac;
  int      type;
  Matrix   temp=NULL,temp2;

  ar   = (*a)[0][NR];
  ac   = (*a)[0][NC];
  type = (*a)[0][MAT_TYPE];

  /* allocate a new matrix */

  switch (type) {

  case IS_FULL:
    temp = my_matrix(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      for (j=1; j<=ac; ++j)
	temp[i][j] = (*a)[i][j];
    break;

  case IS_SYMM:
    temp = my_matrix_symm(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      for (j=i; j<=ac; ++j)
	temp[i][j] = (*a)[i][j];
    break;

  case IS_DIAG:
    temp = my_matrix_diag(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      temp[i][i] = (*a)[i][i];
    break;

  default:
    exit(-12);

  }

  temp2 = *a;
  *a = temp;

  my_free_matrix(temp2,1,ar,1,ac);

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  imat_add_shape
 \date  May 1999
 
 \remarks 
 
 adds a row to the given integer matrix

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : matrix a
 \param[in]     nr              : number of rows to be added
 \param[in]     nc              : number of columns to be added 

 ******************************************************************************/
int
imat_add_shape(iMatrix *a, int nr, int nc)
{
  int      i,j,m;
  int      ar,ac;
  int      type;
  iMatrix  temp=NULL,temp2;

  ar   = (*a)[0][NR];
  ac   = (*a)[0][NC];
  type = (*a)[0][MAT_TYPE];

  /* allocate a new matrix */

  switch (type) {

  case IS_FULL:
    temp = my_imatrix(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      for (j=1; j<=ac; ++j)
	temp[i][j] = (*a)[i][j];
    break;

  case IS_SYMM:
    temp = my_imatrix_symm(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      for (j=i; j<=ac; ++j)
	temp[i][j] = (*a)[i][j];
    break;

  case IS_DIAG:
    temp = my_imatrix_diag(1,ar+nr,1,ac+nc);
    for (i=1; i<=ar; ++i)
      temp[i][i] = (*a)[i][i];
    break;

  default:
    exit(-12);

  }

  temp2 = *a;
  *a = temp;

  my_free_imatrix(temp2,1,ar,1,ac);

  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  vec_add_shape
 \date  May 1999
 
 \remarks 
 
 adds elements to a vector

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     nr              : number of elements to be added

 ******************************************************************************/
int
vec_add_shape(Vector *a, int nr)
{
  int      i;
  int      ar;
  Vector   temp=NULL,temp2;

  ar   = (*a)[0];

  /* allocate a new vector */

  temp = my_vector(1,ar+nr);

  for (i=1; i<=ar; ++i)
    temp[i] = (*a)[i];

  temp2 = *a;
  *a = temp;

  my_free_vector(temp2,1,ar);


  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  ivec_add_shape
 \date  May 1999
 
 \remarks 
 
 adds elements to an ivector

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     a		 : vector a
 \param[in]     nr              : number of elements to be added

 ******************************************************************************/
int
ivec_add_shape(iVector *a, int nr)
{
  int      i;
  int      ar;
  iVector   temp=NULL,temp2;

  ar   = (*a)[0];

  /* allocate a new vector */

  temp = my_ivector(1,ar+nr);

  for (i=1; i<=ar; ++i)
    temp[i] = (*a)[i];

  temp2 = *a;
  *a = temp;

  my_free_ivector(temp2,1,ar);



  return TRUE;
  
}

/*!*****************************************************************************
 *******************************************************************************
 \note  byteswap
 \date  May 2000
 
 \remarks 
 
 swaps the bytes of given variable

 *******************************************************************************
 Parameters:  (i/o = input/output)

 \param[in]     ptr		 : pointer to the variable to swap
 \param[in]     nbytes          : number of bytes to be swapped

 ******************************************************************************/
void
byteswap(char *ptr, size_t n_bytes)
{
  int      i,j;
  short    c[100];;

  for (i=0; i<n_bytes; ++i)
    c[i] = ptr[i];

  j = n_bytes;
  for (i=0; i<n_bytes; ++i)
    ptr[--j] = c[i];

}
double
byteswap_double(double var)
{
  byteswap((char *)&var,sizeof(double));
  return var;
}
float
byteswap_float(float var)
{
  byteswap((char *)&var,sizeof(float));
  return var;
}
int
byteswap_int(int var)
{
  byteswap((char *)&var, sizeof(int));
  return var;
}

/*!*****************************************************************************
 *******************************************************************************
  \note  clmcplot_convert
  \date  June 1999

  \remarks 

  reads an CLMCPLOT file and returns the relevant variables. All relevant
  memory is allocated if D, vnames, units are initialized with the NULL
  pointer. Otherwise we assume enough memory is available.

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname     : filename of file to be read
 \param[out]    D         : data matrix
 \param[out]    vnames    : names of data columns
 \param[out]    units     : units of data columns
 \param[out]    freq      : sampling frequency of data
 \param[out]    n_cols    : number of columns
 \param[out]    n_rows    : number of rows

  returns TRUE if successful, otherwise FALSE

 ******************************************************************************/
int
mrdplot_convert(char *fname, Matrix *D, char ***vnames, char ***units, 
		double *freq, int *n_cols, int *n_rows)
{
  return clmcplot_convert(fname, D, vnames, units, freq, n_cols, n_rows);
}

int
clmcplot_convert(char *fname, Matrix *D, char ***vnames, char ***units, 
		double *freq, int *n_cols, int *n_rows)
{
  int     j,i,r;
  FILE   *fp;
  int     buffer_size;
  fMatrix buff;
  int     aux;
  int     rc;

  /* open the file, and parse the parameters */
  fp = fopen(fname,"r");
  if (fp == NULL)
    return FALSE;
  
  /* get the number of rows, columns, sampling frequency
     and calc the buffer_size */
  rc = fscanf(fp,"%d %d %d %lf",&buffer_size,n_cols,n_rows,freq);

  /* alocate memory for the array variables if needed */
  if (*vnames == NULL) 
    *vnames = (char **)my_calloc(*n_cols+1, sizeof(char *), MY_STOP);
  if (*units == NULL)
    *units = (char **)my_calloc(*n_cols+1, sizeof(char *), MY_STOP);

  for (i=1;i<= *n_cols;++i){
    (*vnames)[i] = (char *) my_calloc(40, sizeof(char), MY_STOP);
    (*units)[i]  = (char *) my_calloc(40, sizeof(char), MY_STOP);
    rc= fscanf(fp, "%s %s", (*vnames)[i], (*units)[i]);
  }

  /* there are two extra blank chrs at the end of the block
     and a line return which we must account for */
  fgetc(fp);
  fgetc(fp);
  fgetc(fp);
  
  /* read file into a buffer and check if the matrix size is correct */  
  buff = my_fmatrix(1,*n_rows,1,*n_cols);
  if (*D == NULL)
    *D = my_matrix(1,*n_rows,1,*n_cols);
  
  if (fread(&buff[1][1],sizeof(float),*n_rows * *n_cols,fp)!= *n_rows * *n_cols){
    printf("cannot fread matrix. \n");
    return MY_ERROR;
  }

#ifdef BYTESWAP
  /* convert little-endian to big-endian */ 
  for (j=1; j<=*n_cols; ++j) {
    for (i=1; i<=*n_rows; ++i) {
      aux = LONGSWAP(*((int *)&(buff[i][j])));
      buff[i][j] = *((float *)&aux);
    }
  }
#endif
  
  fclose(fp);

  /* copy data into D matrix */
  for (j=1; j<=*n_cols; ++j) {
    for (i=1; i<=*n_rows; ++i) {
      (*D)[i][j] = (double) buff[i][j];
    }
  }

  /* free up memory by deallocating resources */
  my_free_fmatrix (buff,1,*n_rows,1,*n_cols);
  
  return TRUE;
  
}


/*!*****************************************************************************
 *******************************************************************************
\note  clmcplot_gen
\date  March 2006
   
\remarks 

        writes clmcplot data file 

 *******************************************************************************
 Function Parameters: [in]=input,[out]=output

 \param[in]     fname     : filename of file to be read
 \param[in]     D         : data matrix
 \param[in]     vnames    : names of data columns
 \param[in]     units     : units of data columns
 \param[in]     freq      : sampling frequency of data
 \param[in]     n_cols    : number of columns
 \param[in]     n_rows    : number of rows


 ******************************************************************************/
int
mrdplot_gen(char *fname, Matrix D, char **vnames, char **units, 
	    double freq, int n_cols, int n_rows) 
{
  return clmcplot_gen(fname, D, vnames, units,freq,n_cols,n_rows);
  
}
int
clmcplot_gen(char *fname, Matrix D, char **vnames, char **units, 
	    double freq, int n_cols, int n_rows)
{
  int     j,i,r;
  FILE   *fp;
  int     buffer_size;
  fMatrix buffer;
  int     aux;

  // open the file, and write all data
  if ( (fp = fopen( fname, "w" )) == NULL )  {
    printf( "cannot fopen file %s for write.\n", fname );
    return FALSE;
  }

  // create a floating point buffer and copy all data over
  buffer = my_fmatrix(1,n_rows,1,n_cols);
  for (i=1; i<=n_rows; ++i)
    for (j=1; j<=n_cols; ++j)
      buffer[i][j] = D[i][j];
  
  // write the the buffer size, the number of columns, the sampling time, 
  // the column names and units
  buffer_size = n_rows*n_cols;
  fprintf(fp,"%d  %d  %d  %f\n",buffer_size, n_cols, n_rows, freq);

  for (i=1; i<=n_cols; ++i) {

    if (strcmp(vnames[i],"")==0)
      sprintf(vnames[i],"C%d",i);
    fprintf(fp,"%s  ",vnames[i]);

    if (strcmp(units[i],"")==0)
      sprintf(vnames[i],"-");
    fprintf(fp,"%s  ",units[i]);

  }
  fprintf(fp,"\n");

#ifdef BYTESWAP
  /* convert little-endian to big-endian */ 
  for (j=1; j<=n_cols; ++j) {
    for (i=1; i<=n_rows; ++i) {
      aux = LONGSWAP(*((int *)&(buffer[i][j])));
      buffer[i][j] = *((float *)&aux);
    }
  }
#endif

  if (fwrite(&(buffer[1][1]),sizeof(float),n_rows*n_cols,fp)!= n_rows*n_cols) {
    printf( "cannot fwrite matrix.\n" );
    return FALSE;
  }

  fclose( fp );

  return TRUE;

}

