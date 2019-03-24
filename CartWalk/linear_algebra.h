/*****************************************************************************/
/*! \file    linear_algebra.h
 *  \author  Rhoban Project
 *  \date    2010-09
 *  \brief   Personal Linear Algebra Library
 *****************************************************************************/
#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#include <vector>
#include <math.h>
#include <map>
#include <list>
#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <stdexcept>
#include <complex>
#include "CartWalk/math_log.h"
#include "CartWalk/math_basics.h"
using namespace std;

/*! \brief The base numeric type, maybe float or double */
#ifndef ARM
typedef double scalar;
#else
typedef float scalar;
#endif

/* TODO:
 * Encapsuler completement le type */

/*****************************************************************************/
#define LA_ERROR(str) MathLog::fatal_error(str)
/*****************************************************************************/

void set_ll_epsilon(scalar e);
double LA_precision();
bool is_zero(scalar x);

/*****************************************************************************/

class Matrix : public vector<scalar>
{
public:
  int lig;
  int col;

public:
  /***************************************************************************/
  /* construction                                                            */
  /***************************************************************************/

  /*! \brief base constructor */
  Matrix(int l, int c);

  /*! \brief base constructor */
  Matrix(int l, int c, scalar v);

  /*! \brief constructor of vector */
  Matrix(int dim);

  /*! \brief void matrix */
  Matrix();

  Matrix& operator=(const vector<float>& c);
  Matrix& operator=(const vector<double>& c);

  /*! \brief constructor of vector */
  Matrix(const Matrix& other);

  /* \brief construct directly a vector */
  Matrix(const vector<scalar>& v);

  /*! \brief constructor from brut data */
  Matrix(int l, int c, const scalar* data);

  ~Matrix();

  /*! \brief construct a vector from a scalar array */
  static Matrix mk_vector(const scalar* t, int n);

  /*! \brief construct a vector object directly from the
    coefficients, usefull in some test case. */
  static Matrix mk_vector(int dim, scalar x1, ...);

  /*! \brief clear the matrix and resize it (put 0.0 in each cell) */
  void matrix_resize(int lig, int col);

  /*! base cast from real number to matrix */
  static Matrix real2Matrix(scalar x);

  /*! base cast from matrix to real */
  static scalar Matrix2real(Matrix x);

  /***************************************************************************/
  /* Special Cases                                                           */
  /***************************************************************************/

  static Matrix null_matrix(int l, int c);
  static Matrix null_vector(int n);

  /* \brief contruct the i-th vector of the canonical base of dim dim */
  static Matrix canonic_base(int dim, int i);

  /*! \brief construct the identity matrix */
  static Matrix identity(int n);

  /*! \brief construct the identity matrix (non square)*/
  static Matrix identity(int l, int c);

  /*! Rotation Matrix in dimension 2. */
  static Matrix rotation_2D(scalar theta);

  /*! Rotation Matrix in any dimension in the plane (X1,X2). */
  static Matrix rotation(int dim, int X1, int X2, scalar theta);

  /*! Rotation around the (3D) vector axis, of angle theta in the direction
   *  of axis */
  Matrix static rotation_3D(Matrix axis, double theta);

  /*! vertical vector (a, a+dt, a+2dt, ...., a+n*dt) with n = (b-a) / dt */
  static Matrix arithmetic_vector(scalar a, scalar b, scalar dt);

  /***************************************************************************/
  /* Coefficient access                                                      */
  /***************************************************************************/

  /*! \brief set coefficient */
  inline void set(int l, int c, scalar v)
  {
    (*this)[l * col + c] = v;
  };

  /*! \brief get coefficient */
  inline scalar get(int l, int c) const
  {
    return (*this)[l * col + c];
  };

  /*! \brief get coefficient */
  inline scalar& get_ref(int l, int c)
  {
    return (*this)[l * col + c];
  };

  /*! \brief special vector function: add a new component to the vector,
   * at the end (push_back) */
  void extend(scalar x);
  /*! \brief idem but extends with a vector */
  void extend(Matrix v);

  /*! \brief checks whether this and a have same dimensions or not. */
  inline bool same_dim(Matrix a)
  {
    return a.lig == lig && a.col == col;
  };

  /***************************************************************************/
  /* Construction Operations                                                 */
  /***************************************************************************/

  /*!\brief concat a list of matrix in column */
  static Matrix horizontal_concat(const vector<Matrix>& matrices);

  /*!\brief concat a list of matrix in column */
  static Matrix horizontal_concat(Matrix m1, Matrix m2);

  /*!\brief concat a list of matrix in column with an other list, pairwise */
  static vector<Matrix> horizontal_concat(const vector<Matrix>& m1s, const vector<Matrix>& m2s);

  /*!\brief adds a line at the bottom of the matrix, in place for efficiency */
  void add_line(Matrix l);
  void add_line(vector<double>::const_iterator b, vector<double>::const_iterator e);

  /*!\brief concat a list of matrix in lines */
  static Matrix vertical_concat(const vector<Matrix>& matrices);

  /*!\brief concat a list of matrix in lines with an other list, pairwise */
  static vector<Matrix> vertical_concat(const vector<Matrix>& m1s, const vector<Matrix>& m2s);

  /*!\brief concat a list of matrix in lines */
  static Matrix vertical_concat(Matrix m1, Matrix m2);

  /*! \brief select lines between line_min and line_max (included) */
  Matrix select_lines(int line_min, int line_max) const;

  /*! \brief select columns between line_min and line_max (included) */
  Matrix select_columns(int col_min, int col_max) const;

  /*! \brief select lines between line_min and line_max (included) */
  static vector<Matrix> select_lines(const vector<Matrix>& l, int line_min, int line_max);

  /*! \brief select columns between line_min and line_max (included) */
  static vector<Matrix> select_columns(const vector<Matrix>& l, int col_min, int col_max);

  /*! \brief extract the column n of the matrix */
  Matrix extract_column(int j) const;

  /* \brief extract the line i of the matrix */
  Matrix extract_line(int i) const;

  /*! \brief select lines between line_min and line_max (included) */
  Matrix extract_lines(int line_min, int line_max) const;

  /*! \brief select columns between line_min and line_max (included) */
  Matrix extract_columns(int col_min, int col_max) const;

  /* \brief set the line i to be equal to l */
  void set_line(int i, Matrix l);

  /* \brief set the column j to be equal to c */
  void set_column(int j, Matrix c);

  /* \brief extract the minor matrix of position (i,j) */
  Matrix extract_minor(int i, int j) const;

  /* \brief insert matrix at line i */
  Matrix insert_at_line(const Matrix& m, int i0) const;

  /* \brief insert matrix at column j */
  Matrix insert_at_column(const Matrix& m, int j0) const;

  /*! \brief exchange the columns j1 and j2 */
  void exchange_column(int j1, int j2);

  /*! \brief exchange the lines i1 and i2 */
  void exchange_line(int i1, int i2);

  /*! \brief extract the matrix list from id_min to id_max (included)
   *  if id_max == -1 then this means the end of the list */
  static vector<Matrix> extract_matrix_list(const vector<Matrix>& l, int id_min, int id_max);

  /*! \brief extract matrixes from l from the index list indexes. */
  static vector<Matrix> select_from_indexes(const vector<Matrix>& l, vector<int> indexes);

  /***************************************************************************/
  /* operations on matrix and matrix lists                                   */
  /***************************************************************************/

  /*! \brief product scalar * vector */
  friend Matrix operator/(const Matrix& v, double x);

  /*! \brief multiply each matrix of l by x */
  friend vector<Matrix> operator/(const vector<Matrix>& l, double x);

  /*! \brief product scalar * vector */
  friend Matrix operator/(const Matrix& v, int x);

  /*! \brief multiply each matrix of l by x */
  friend vector<Matrix> operator/(const vector<Matrix>& l, int x);

  /*! \brief product scalar * vector */
  friend Matrix operator*(int x, const Matrix& v);

  /*! \brief multiply each matrix of l by x */
  friend vector<Matrix> operator*(int x, const vector<Matrix>& l);

  /*! \brief product scalar * vector */
  friend Matrix operator*(float x, const Matrix& v);

  /*! \brief multiply each matrix of l by x */
  friend vector<Matrix> operator*(float x, const vector<Matrix>& l);

  /*! \brief product scalar * vector */
  friend Matrix operator*(double x, const Matrix& v);

  /*! \brief multiply each matrix of l by x */
  friend vector<Matrix> operator*(double x, const vector<Matrix>& l);

  /*! \brief sum of vector */
  friend Matrix operator+(const Matrix& v1, const Matrix& v2);

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend vector<Matrix> operator+(const vector<Matrix>& l, const Matrix& m);

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend vector<Matrix> operator+(const Matrix& m, const vector<Matrix>& l);

  /*! \brief sum of vector */
  friend void operator+=(Matrix& v1, const Matrix& v2);

  /*! \brief vertical concatenation with another Matrix with same number of columns */
  friend void operator|=(Matrix& matrix1, const Matrix& matrix2);

  /*! \brief vertical concatenation with another Matrix with same number of lines */
  friend void operator&=(Matrix& matrix1, const Matrix& matrix2);

  /*! \brief diff of vector */
  friend void operator-=(Matrix& v1, const Matrix& v2);

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend void operator+=(vector<Matrix>& l, const Matrix& m);

  /*! \brief sum of vector */
  friend Matrix operator-(const Matrix& v1, const Matrix& v2);

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend vector<Matrix> operator-(const vector<Matrix>& l, const Matrix& m);

  /*! \brief sum of element by element of a list of matrix with another list of matrix */
  friend vector<Matrix> operator-(const vector<Matrix>& lm1, const vector<Matrix>& lm2);
  // TODO the same for the other operators

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend vector<Matrix> operator-(const Matrix& m, const vector<Matrix>& l);

  /*! \brief sum of vector */
  friend Matrix operator-(const Matrix& m);

  /*! \brief sum of element by element of a list of matrix with a matrix */
  friend vector<Matrix> operator-(const vector<Matrix>& l);

  /*! \brief product of matrices */
  friend Matrix operator*(const Matrix& m1, const Matrix& m2);

  /*! \brief multiply each matrix of l by m to the left  */
  friend vector<Matrix> operator*(const vector<Matrix>& l, const Matrix& m);

  /*! \brief multiply each matrix of m by the scalar l */
  friend void operator*=(Matrix& m, scalar l);

  /*! \brief transpose */
  Matrix transpose() const;

  /*! \brief cross product (produit vectoriel :-) */
  static Matrix cross_product(Matrix x1, Matrix x2);

  /*! \brief compute the linear interpolation at x of the function { x1->y1, x2->y2 } */
  static Matrix linear_interpol(double x1, Matrix y1, double x2, Matrix y2, double x);

  /***************************************************************************/
  /* Predicate testing matrix format                                         */
  /***************************************************************************/

  /*! \brief test the lower triangular matrix format
   *  if false is returned, error contains the max error */
  static bool lower_triangular_matrix(const Matrix& A, scalar* error);
  static bool lower_triangular_matrix(const Matrix& A);

  /*! \brief test the strict lower triangular matrix format with only one on the diagonal
   *  if false is returned, error contains the max error */
  static bool unit_lower_triangular_matrix(const Matrix& A, scalar* error);
  static bool unit_lower_triangular_matrix(const Matrix& A);

  /*! \brief test the upper triangular matrix format
   *  if false is returned, error contains the max error */
  static bool upper_triangular_matrix(const Matrix& A, scalar* error);
  static bool upper_triangular_matrix(const Matrix& A);

  /*! \brief test the Hessenberg matrix format
   *  if false is returned, error contains the max error */
  static bool hessenberg_matrix(const Matrix& A, scalar* error);
  static bool hessenberg_matrix(const Matrix& A);

  /*! \brief test the orthogonal matrix format
   *  if false is returned, error contains the max error */
  static bool orthogonal_matrix(const Matrix& A, scalar* error);
  static bool orthogonal_matrix(const Matrix& A);

  /*! \brief test the permutation matrix format
   *  if false is returned, error contains the max error */
  static bool permutation_matrix(const Matrix& A, scalar* error);
  static bool permutation_matrix(const Matrix& A);

  /*! \brief test the bidiagonal matrix format
   *  if false is returned, error contains the max error */
  static bool bidiagonal_matrix(const Matrix& A, scalar* error);
  static bool bidiagonal_matrix(const Matrix& A);

  /*! \brief test the diagonal matrix format
   *  if false is returned, error contains the max error */
  static bool diagonal_matrix(const Matrix& A, scalar* error);
  static bool diagonal_matrix(const Matrix& A);

  /***************************************************************************/
  /* Auxiliary functions for complex operations                              */
  /***************************************************************************/

  /*! \brief compute the vector for the left Householder transformation associated with the col-th column and the lig
   * row of the given matrix */
  static Matrix Householder_vector(Matrix& m, int lig, int col);
  /*! \brief compute the vector for the right Householder transformation associated with the col-th column and the lig
   * row of the given matrix */
  static Matrix Householder_vector_right(Matrix& m, int col, int lig);

  /*! brief Left multiply the given matrix m with the Householder matrix defined by the vector U and the i-th column */
  static void Householder_prod(Matrix& m, Matrix& U, int i);

  /*! brief Right multiply the given matrix m with the Householder matrix defined by the vector U and the i-th column */
  static void Householder_prod_right(Matrix& m, Matrix& U, int i);

  /*! \brief Householder_prod optimized version for the R matrix in QR decomposition (upper triangular) */
  static void Householder_prod_optimized(Matrix& m, Matrix& U, int i);

  /*! \brief return the product of diadonal elements */
  scalar diagonal_product_matrix() const;

  /*! \brief Compute the matrix U in the PLU decomposition
   *  the boolean represent the sign of the determinant of P */
  pair<Matrix, bool> PLU_Decomposition_optimized_for_determinant() const;

  /*! \brief compute sqrt(a^2 + b^2) without numerical issues */
  static scalar hypo(scalar a, scalar b);

  /*! \brief annihilate the (i,j) value in the matrix B and report the operation in U or V
   * use Givens Rotation
   * linear complexity
   * if (i,j) is under the diagonal, a left tranformation is performed
   * a right tranformation if above
   */
  static void Givens_Transformation(int i, int j, triple<Matrix>& UBV);

  /***************************************************************************/
  /* Complex operations on matrix                                            */
  /***************************************************************************/

  /*! \brief gauss algorithm matrix inversion,
   *  the boolean indicates if the matrix is invertible
   *  and if the returned matrix is valid */
  pair<bool, Matrix> gauss_inverse() const;

  /*! \brief The pseudo-inverse of Penrose-Moore.
      Cf. http://fr.wikipedia.org/wiki/Pseudo-inverse */
  Matrix pseudo_inverse() const;

  /*! \brief Compute the Cholesky decomposition of a positive
   *  definite symetric matrix. (The returned matrix is lower triangular)
   *  http://fr.wikipedia.org/wiki/Factorisation_de_Cholesky */
  Matrix Cholesky() const;

  /*! \brief Compute the LU decomposition of a square matrix.
   *  http://en.wikipedia.org/wiki/LU_decomposition */
  triple<Matrix> PLU_Decomposition() const;

  /*! \brief Compute the Hessenberg decomposition of the matrix
   *  return P,H
   *  H : the Hessenberg form of the matrix
   *  P : a orthogonal matrix such as PH is the inital matrix
   *  Decomposition : P * H * P.transpose() */
  pair<Matrix, Matrix> Hessenberg_Decomposition() const;

  /*! \brief Compute the bidiagonal decomposition of the matrix
    http://en.wikipedia.org/wiki/Bidiagonal_matrix */
  triple<Matrix> Bidiagonal_Decomposition() const;

  /*! \brief compute the QR decomposition, i.e. decompose the matrix
   *  into a product QR where Q is orthogonal and R triangular.
   * see http://fr.wikipedia.org/wiki/D%C3%A9composition_QR */
  pair<Matrix, Matrix> QR_Decomposition() const;

  /*! \brief Compute the SVD decomposition of the matrix
     http://en.wikipedia.org/wiki/Singular_value_decomposition */
  triple<Matrix> SVD_Decomposition() const;

  /*! \brief EN CHANTIER compute the eigenvalues of the matrix, by QR iterations.
   *  see http://en.wikipedia.org/wiki/QR_algorithm */
  // Matrix Eigenvalues() const;
  vector<complex<scalar> > Eigenvalue();

  /*! \brief Compute the determinant of the square matrix */
  scalar determinant() const;

  /*! \brief Solve the equation Lx = b by forward substitution
   *  L must be lower triangular (no check done)
   *  x is returned */
  static Matrix forwardSubstitution(const Matrix& Lx, const Matrix& b);
  static Matrix backwardSubstitution(const Matrix& Ux, const Matrix& b);

  /*! \brief Compute the mean of a set of matrix, term by term*/
  static Matrix mean(const vector<Matrix>& matrix_set);

  /*! \brief statistic of lines of the matrix */
  Matrix mean_of_lines();
  Matrix std_dev_of_lines();
  Matrix max_of_lines();
  Matrix min_of_lines();

  /*! \brief Compute the statistics of columns of the matrix */
  Matrix mean_of_columns();
  Matrix std_dev_of_columns();
  Matrix max_of_columns();
  Matrix min_of_columns();

  /*! \brief Compute the mean of p-power of components, term by term (kind of norm L^p) */
  static Matrix mean_pow(const vector<Matrix>& matrix_set, double p);

  /*! \brief Compute the standard deviation of the matrix term by term */
  static Matrix std_dev(const vector<Matrix>& matrix_set);

  /*! \brief Normalize the samples to mu = 0, std_dev = 1 */
  static vector<Matrix> normalize_samples(const vector<Matrix>& samples);

  /*! \brief The term to term maximum */
  static Matrix matrix_max(const vector<Matrix>& matrix_set);

  /*! \brief The term to term minimum */
  static Matrix matrix_min(const vector<Matrix>& matrix_set);

  /*! \brief The index of the maximum of the l x c coefficient of the list of matrix */
  static int idx_of_max(const vector<Matrix>& matrix_set, int min_idx, int max_idx, int l, int c);

  /*! \brief The index of the minimum of the l x c coefficient of the list of matrix */
  static int idx_of_min(const vector<Matrix>& matrix_set, int min_idx, int max_idx, int l, int c);

  /*! \brief Compute the smoothed sequence, by discounting backward */
  static vector<Matrix> smooth_sequence(const vector<Matrix>& seq, scalar discount_factor, int window_width);

  /*! \brief Compute the smoothed line sequence, by discounting backward */
  static Matrix smooth_lines(const Matrix& seq, scalar discount_factor, int window_width);

  /*! \brief Compute and extract the smoothed line of index idx of seq */
  static Matrix extract_smoothed_line(const Matrix& seq, scalar discount_factor, int window_width, int idx);

  /*! \brief Compute and extract the smoothed value of index idx of seq (smoothing is done vertically, i.e.
   *  with previous lines */
  scalar vertical_smoothed_value(scalar discount_factor, int window_width, int col_idx, int idx);

  /*! \brief Print a statistic report about the matrix set */
  static void statistic_report(const vector<Matrix>& matrix_set);

  /*! \brief Compute the mean of a subset of a set of matrix. The
      computation takes matrix between indexes a and b included */
  static Matrix local_mean(const vector<Matrix>& matrix_set, int a, int b);

  /*! \brief Construct the list of indexes of element of the serie at
   *  distance less than radius from x
   *  \param basis_change is the mapping (linear) applied to each vector
   *  to compute the distance, NULL means identity mapping */
  static vector<int> get_neighboors(const vector<Matrix>& list, Matrix x, scalar radius, Matrix* basis_change = NULL);

  /*! \brief Construct the list of index of the (maximum) n closest
   *  element of the serie at distance less than radius from x.
   *  \param basis_change is the mapping (linear) applied to each vector
   *  to compute the distance, NULL means identity mapping. */
  static vector<int> get_closest_neighboors(vector<Matrix>& list, Matrix x, int n, scalar radius, Matrix* basis_change);

  /*! \brief Jacobian computation by least squares. X and Y are lists
      of vectors, sampling a function. Compute an approximation of the
      Jacobian matrix at the mean of X set. */
  static Matrix jacobian(const vector<Matrix>& X, const vector<Matrix>& Y);

  /*! \brief Compute the image of x by the affine mapping
   *  approximating the sample */
  static Matrix affine_approximation(const vector<Matrix>& X, const vector<Matrix>& Y, const Matrix& x);

  /*! \brief scalar product */
  static scalar scalar_prod(const Matrix& x, const Matrix& y);

  /*! \brief normalize the vector to get norm2 equal to 1 */
  Matrix normalize(Matrix* basis_change) const;

  /*! \brief normalize the vector to get norm2 equal to 1 */
  Matrix normalize() const;

  /*! \brief Compute the norm 2 of the vector,
   *  \param basis_change is the mapping (linear) applied to each vector
   *  to compute the distance, NULL means identity mapping */
  scalar norm2(Matrix* basis_change) const;

  /* norm2(NULL) */
  scalar norm2() const;

  /*! \brief Computes the distance with another matrix
   */
  scalar distance2(const Matrix& other) const
  {
    scalar s = 0.0;
    unsigned int n = size();
    if (n != other.size())
      throw std::runtime_error("norm2_distance failed: Matrix size mismatch");

    for (unsigned int k = 0; k < n; k++)
    {
      scalar t = other[k] - (*this)[k];
      s += t * t;
    }

    return sqrt(s);
  }

  /*! \brief Computes the 3D distance with another matrix
   */
  scalar distance3(const Matrix& other) const
  {
    return sqrt((other[0] - (*this)[0]) * (other[0] - (*this)[0]) + (other[1] - (*this)[1]) * (other[1] - (*this)[1]) +
                (other[2] - (*this)[2]) * (other[2] - (*this)[2]));
  }

  static scalar norm2_distance(const Matrix& m1, const Matrix& m2)
  {
    return m1.distance2(m2);
  };

  /*! \brief Compute the norm 2 of the vector
   *  \param basis_change is the mapping (linear) applied to each vector
   *  to compute the distance, NULL means identity mapping */
  scalar norm_max(Matrix* basis_change) const;

  /* norm_max(NULL) */
  scalar norm_max() const;

  /*! \brief Give the absolute values of each coefficient of the matrix */
  Matrix abs() const;

  /*! \brief Covariance Matrix of two lists of vectors */
  static Matrix covariance(vector<Matrix> X, vector<Matrix> Y);

  /*! \brief Compute the image of x by the least square affine mapping
      approximating the sample. Here we use the least square
      approximation formula, given by the covariance matrix:
      TODO: explication covariance et least square. */
  static Matrix LS_affine_approximation(vector<Matrix> X, vector<Matrix> Y, Matrix x);

  /* Compute an affine approximation of form Y = A.X + B, return (A,B) */
  static pair<Matrix, Matrix> LS_affine_approximation_equation(vector<Matrix> X, vector<Matrix> Y);

  /* Idem as previous function, but vector are given by lines of the matrixes */
  static pair<Matrix, Matrix> LS_affine_approximation_equation(Matrix X, Matrix Y);

  /* compute an approximation of the function X -> Y under the form of a polynom
   * of degree n. Return the coefficients of the polynom [a_0, a_1, ..., a_{n-1}]
   * the algorithm is based on the computation of the pseudo inverse. */
  static vector<double> polynomial_approximation(vector<double> X, vector<double> Y, int n);

  /***************************************************************************/
  /* misc                                                                    */
  /***************************************************************************/

  /*! \brief set the minimum ration between coefficient to be epsilon
      by setting low values to 0.0 */
  void epsilon_filter();

  /*! \brief all the coefficients are-they null ? */
  bool null_matrix();

  /*! \brief equality term by term */
  bool operator==(const Matrix& other);

  /*! \brief test the equality of matrix aimed for testing
   *  if false is returned, the norm_max of m1-m2 is compute in error */
  static bool equal_matrix(const Matrix& m1, const Matrix& m2, scalar* error);

  /*! \brief pretty printing */
  void pp(ostream& os);

  /*! direct pretty print */
  friend ostream& operator<<(ostream& stream, const Matrix& m);

  /*! \brief the number of decimal to be printed */
  static void set_pp_precision(int n);

  /*! \brief export the list of matrix into a text file (compatible with gnuplot) */
  static void export_text(string filename, vector<Matrix> l);

  /*! \brief export a matrix into a text file (compatible with gnuplot) */
  static void export_text(string filename, Matrix m);

  /*! \brief read a list of vector from a file, one vector per line,
   *  if record_nb = -1, one takes all the available records of the file */
  static vector<Matrix> vector_list_from_file(string filename, int start_idx, int record_nb);

  /*! \brief read a list of vector from a file, one vector per line */
  static vector<Matrix> vector_list_from_file(string filename);

  /*! \brief read a matrix from a file */
  static Matrix matrix_from_file(string filename);

  /*! \brief write a list of vector to a file, one vector per line */
  static void vector_list_to_file(const vector<Matrix>& list, string filename);

  /*! \brief read a vector from a string. The string is supposed to contains a sequence of numbers */
  static Matrix vector_from_string(string str);

  /*! \brief print just the dimensions of the matrix */
  void size_pp(ostream& os);

  /*! \brief compute the product M * M.transpose() (for optimizations)
   *  result is supposed to have the good dimension M.lig x M.lig */
  static void square_transpose(Matrix& M, Matrix& result);
};

/*****************************************************************************/

#define MATRIX_ITERATE(v, x) for (vector<scalar>::iterator x = (v)->begin(); x != (v)->end(); x++)

#define CONST_MATRIX_ITERATE(v, x) for (vector<scalar>::const_iterator x = (v)->begin(); x != (v)->end(); x++)

/*****************************************************************************/
// Tools (to be put anywhere else)
/*****************************************************************************/

double argument(Matrix v);

/** conversion in dimension 2 */
Matrix cartesian2polar(Matrix v);

/** conversion in dimension 2 */
Matrix polar2cartesian(Matrix v);

/* x-y-z position to (radius-longitude-latitude) (rad) */
Matrix cartesian_to_spherical(Matrix M);

/* (radius-longitude-latitude) to x-y-z position (rad) */
Matrix spherical_to_cartesian(Matrix M);

/* Compute the angle of a triangle from lengths:
 * a,b,c length of opposite side of angles A, B, C
 * return the angle A in [0, pi] */
double al_kashi(double a, double b, double c);

/* return the angle of the rotation around axis to put origin on target */
double inverse_rotation(Matrix origin, Matrix target, Matrix axis);

#endif
/*****************************************************************************/
/*****************************************************************************/
