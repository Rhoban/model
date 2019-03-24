/*****************************************************************************/
/*! \file    linear_algebra.cc
 *  \author  OL
 *  \date    2010-09
 *  \brief   Personal Linear Algebra Library
 *****************************************************************************/
#include <math.h>
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <list>
#include <complex>
#include "assert.h"
#include <fstream>
#include "math_basics.h"
#include "linear_algebra.h"
using namespace std;

/*****************************************************************************/

scalar ll_epsilon = 0.000001;

void set_ll_epsilon(scalar e)
{
  ll_epsilon = e;
}

double LA_precision()
{
  return ll_epsilon;
};

bool is_zero(scalar x)
{
  if (x == 0.0)
    return true;
  if (x > 0.0)
    return x <= ll_epsilon;
  if (x < 0.0)
    return x >= -ll_epsilon;
  return false;
}

/***************************************************************************/
/* construction                                                            */
/***************************************************************************/

Matrix::Matrix(int l, int c) : vector<scalar>(l * c), lig(l), col(c){};

/*****************************************************************************/

Matrix::Matrix(int l, int c, scalar v) : vector<scalar>(l * c), lig(l), col(c)
{
  for (unsigned int i = 0; i < size(); i++)
    (*this)[i] = v;
}

/*****************************************************************************/

Matrix::Matrix(int dim) : vector<scalar>(dim), lig(dim), col(1){};

/*****************************************************************************/

Matrix::Matrix() : vector<scalar>(0), lig(0), col(0){};

Matrix& Matrix::operator=(const vector<float>& c)
{
  if (lig != (int)c.size() || col != 1)
    throw std::runtime_error("Size mismatch");
  for (int i = 0; i < lig; i++)
    at(i) = (double)c[i];
  return *this;
}

Matrix& Matrix::operator=(const vector<double>& c)
{
  if (lig != (int)c.size() || col != 1)
    throw std::runtime_error("Size mismatch");
  for (int i = 0; i < lig; i++)
    at(i) = c[i];
  return *this;
}

/*****************************************************************************/

Matrix::Matrix(const Matrix& other) : vector<scalar>(other), lig(other.lig), col(other.col)
{
}

/*****************************************************************************/

Matrix::Matrix(const vector<scalar>& v) : lig(v.size()), col(1)
{
  *((vector<scalar>*)this) = v;
}

/*****************************************************************************/

Matrix::Matrix(int l, int c, const scalar* data) : vector<scalar>(l * c), lig(l), col(c)
{
  if (l < 0 || c < 0)
    LA_ERROR("Matrix:: bad index");
  for (int i = 0; i < l; i++)
    for (int j = 0; j < c; j++)
      set(i, j, *(data + i * c + j));
}

/*****************************************************************************/

Matrix::~Matrix()
{
}

/*****************************************************************************/

Matrix Matrix::mk_vector(const scalar* t, int n)
{
  Matrix V(n, 1);
  for (int i = 0; i < n; i++)
    V.set(i, 0, t[i]);
  return V;
}

/*****************************************************************************/

Matrix Matrix::mk_vector(int dim, scalar x1, ...)
{
  Matrix v(dim, 1);

  va_list args;
  va_start(args, x1);

  for (int i = 0; i < dim; i++)
  {
    v[i] = x1;
    x1 = va_arg(args, double);
  }
  va_end(args);

  return v;
}

/*****************************************************************************/

void Matrix::matrix_resize(int new_lig, int new_col)
{
  if (new_lig < 0 || new_col < 0)
    LA_ERROR("matrix_resize:: bad index");
  clear();
  lig = new_lig;
  col = new_col;
  for (int i = 0; i < lig * col; i++)
    push_back(0.0);
}

/*****************************************************************************/

Matrix Matrix::real2Matrix(scalar x)
{
  return Matrix::mk_vector(1, x);
}

/*****************************************************************************/

scalar Matrix::Matrix2real(Matrix x)
{
  return x.get(0, 0);
}

/***************************************************************************/
/* Special Cases                                                           */
/***************************************************************************/

Matrix Matrix::null_matrix(int l, int c)
{
  Matrix m(l, c);
  for (int i = 0; i < l; i++)
    for (int j = 0; j < c; j++)
      m.set(i, j, 0.0);
  return m;
}

Matrix Matrix::null_vector(int n)
{
  return null_matrix(n, 1);
}

/*****************************************************************************/

Matrix Matrix::canonic_base(int dim, int i)
{
  Matrix e(dim, 1, 0.0);
  e.set(i, 0, 1.0);
  return e;
}

/*****************************************************************************/

Matrix Matrix::identity(int n)
{
  Matrix v(n, n, 0.0);
  for (int i = 0; i < n; i++)
    v.set(i, i, 1.0);
  return v;
}

/*****************************************************************************/

Matrix Matrix::identity(int l, int c)
{
  Matrix v(l, c, 0.0);
  int limit = (l < c) ? l : c;
  for (int i = 0; i < limit; i++)
    v.set(i, i, 1.0);
  return v;
}

/*****************************************************************************/

Matrix Matrix::rotation_2D(scalar theta)
{
  Matrix R(2, 2, cos(theta));
  scalar s = sin(theta);
  R.set(0, 1, -s);
  R.set(1, 0, s);
  return R;
}

/*****************************************************************************/

Matrix Matrix::rotation(int dim, int X1, int X2, scalar theta)
{
  Matrix R = Matrix::identity(dim);
  double c = cos(theta);
  double s = sin(theta);
  R.set(X1, X1, c);
  R.set(X2, X2, c);
  R.set(X1, X2, -s);
  R.set(X2, X1, s);

  return R;
}

/*****************************************************************************/

Matrix Matrix::rotation_3D(Matrix axis, double theta)
{
  Matrix norm_axis = axis.normalize(NULL);
  Matrix V0(3, 1, 0.0);
  bool found = false;
  for (int d = 0; !found && d < 3; d++)
    if (norm_axis[d] == 0.0)
    {
      V0 = Matrix::canonic_base(3, d);
      found = true;
    }
  if (!found)
  {
    V0[0] = 1.0 / norm_axis[0];
    V0[1] = -1.0 / norm_axis[1];
    V0[2] = 0.0;
    V0 = V0.normalize(NULL);
  }
  Matrix V1 = cross_product(norm_axis, V0);
  Matrix P = Matrix::horizontal_concat(norm_axis, Matrix::horizontal_concat(V0, V1));
  return (P * Matrix::rotation(3, 1, 2, theta)) * P.transpose();
}

/***************************************************************************/

Matrix Matrix::arithmetic_vector(scalar a, scalar b, scalar dt)
{
  if (b < a)
    return Matrix(0, 0, 0.0);
  int n = (b - a) / dt;
  Matrix result(n, 1, 0.0);
  for (int i = 0; i < n; i++)
    result.set(i, 0, a + i * dt);
  return result;
}

/***************************************************************************/
/* Coefficient access                                                      */
/***************************************************************************/

void Matrix::extend(scalar x)
{
  if (col == 1)
  {
    push_back(x);
    lig++;
    return;
  }
  if (lig == 1)
  {
    push_back(x);
    col++;
    return;
  }
  if (col == 0 && lig == 0)
  {
    push_back(x);
    lig = 1;
    col = 1;
    return;
  }
  LA_ERROR("extend:: not a vector");
};

void Matrix::extend(Matrix v)
{
  for (int i = 0; i < v.lig; i++)
    extend(v[i]);
}

/***************************************************************************/
/* Construction Operations                                                 */
/***************************************************************************/

Matrix Matrix::horizontal_concat(const vector<Matrix>& matrices)
{
  if (matrices.size() == 0)
    return Matrix(0, 0);

  int lig = matrices[0].lig;
  int col = 0;

  for (unsigned int k = 0; k < matrices.size(); k++)
  {
    if (matrices[k].lig != lig)
      return Matrix(0, 0);
    col += matrices[k].col;
  }

  Matrix the_matrix(lig, col);
  int curr_col = 0;
  for (int i = 0; i < lig; i++)
  {
    curr_col = 0;
    for (unsigned int k = 0; k < matrices.size(); k++)
    {
      for (int j = 0; j < matrices[k].col; j++)
      {
        the_matrix.set(i, curr_col, matrices[k].get(i, j));
        curr_col++;
      }
    }
  }

  return the_matrix;
}

/*****************************************************************************/

Matrix Matrix::horizontal_concat(Matrix m1, Matrix m2)
{
  if (m1.lig != m2.lig)
    return Matrix(0, 0);
  Matrix the_matrix(m1.lig, m1.col + m2.col);

  int curr_col = 0;
  for (int i = 0; i < m1.lig; i++)
  {
    curr_col = 0;
    for (int j = 0; j < m1.col; j++)
    {
      the_matrix.set(i, curr_col, m1.get(i, j));
      curr_col++;
    }
    for (int j = 0; j < m2.col; j++)
    {
      the_matrix.set(i, curr_col, m2.get(i, j));
      curr_col++;
    }
  }

  return the_matrix;
}

/*****************************************************************************/

vector<Matrix> Matrix::horizontal_concat(const vector<Matrix>& m1s, const vector<Matrix>& m2s)
{
  vector<Matrix> result;
  for (unsigned int k = 0; k < m1s.size(); k++)
    result.push_back(horizontal_concat(m1s[k], m2s[k]));
  return result;
}

/*****************************************************************************/

void Matrix::add_line(Matrix l)
{
  if (lig > 0 && col != l.col)
    LA_ERROR("add_line: bad number of columns");
  if (lig == 0 && col == 0)
    col = l.col;
  insert(end(), l.begin(), l.end());
  lig++;
}

void Matrix::add_line(vector<double>::const_iterator b, vector<double>::const_iterator e)
{
  int c = e - b;
  if (lig > 0 && col != c)
    LA_ERROR("add_line: bad number of columns");
  if (lig == 0 && col == 0)
    col = c;
  insert(end(), b, e);
  lig++;
}

/*****************************************************************************/

vector<Matrix> Matrix::vertical_concat(const vector<Matrix>& m1s, const vector<Matrix>& m2s)
{
  vector<Matrix> result;
  for (unsigned int k = 0; k < m1s.size(); k++)
    result.push_back(vertical_concat(m1s[k], m2s[k]));
  return result;
}

/*****************************************************************************/

Matrix Matrix::vertical_concat(const vector<Matrix>& matrices)
{
  if (matrices.size() == 0)
    return Matrix(0, 0);

  int lig = 0;
  int col = matrices[0].col;

  for (unsigned int k = 0; k < matrices.size(); k++)
  {
    if (matrices[k].col != col)
      return Matrix(0, 0);
    lig += matrices[k].lig;
  }

  Matrix the_matrix(lig, col);
  int curr_lig = 0;
  for (unsigned int k = 0; k < matrices.size(); k++)
  {
    for (int i = 0; i < matrices[k].lig; i++)
    {
      for (int j = 0; j < matrices[k].col; j++)
        the_matrix.set(curr_lig, j, matrices[k].get(i, j));
      curr_lig++;
    }
  }

  return the_matrix;
}

/*****************************************************************************/

Matrix Matrix::vertical_concat(Matrix m1, Matrix m2)
{
  if (m1.col != m2.col)
    return Matrix(0, 0);
  Matrix the_matrix(m1.lig + m2.lig, m1.col);

  int curr_lig = 0;
  for (int i = 0; i < m1.lig; i++)
  {
    for (int j = 0; j < m1.col; j++)
      the_matrix.set(curr_lig, j, m1.get(i, j));
    curr_lig++;
  }

  for (int i = 0; i < m2.lig; i++)
  {
    for (int j = 0; j < m2.col; j++)
      the_matrix.set(curr_lig, j, m2.get(i, j));
    curr_lig++;
  }

  return the_matrix;
}

/*****************************************************************************/

Matrix Matrix::select_lines(int line_min, int line_max) const
{
  if ((line_max < line_min) || (line_min < 0) || (line_max < 0) || (line_max >= lig))
    LA_ERROR("select_lines");

  Matrix selected(line_max - line_min + 1, col);
  for (int i = line_min; i <= line_max; i++)
    for (int j = 0; j < col; j++)
      selected.set(i - line_min, j, get(i, j));

  return selected;
}

/*****************************************************************************/

Matrix Matrix::select_columns(int col_min, int col_max) const
{
  if ((col_max < col_min) || (col_min < 0) || (col_max < 0) || (col_max >= col))
    LA_ERROR("select_columns");

  Matrix selected(lig, col_max - col_min + 1);
  for (int i = 0; i < lig; i++)
    for (int j = col_min; j <= col_max; j++)
      selected.set(i, j - col_min, get(i, j));

  return selected;
}

/*****************************************************************************/

vector<Matrix> Matrix::select_lines(const vector<Matrix>& l, int line_min, int line_max)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i].select_lines(line_min, line_max));
  return result;
}

/*****************************************************************************/

vector<Matrix> Matrix::select_columns(const vector<Matrix>& l, int col_min, int col_max)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i].select_columns(col_min, col_max));
  return result;
}

/*****************************************************************************/

Matrix Matrix::extract_column(int j) const
{
  if (j < 0 || j >= col)
    return Matrix(0, 0);

  Matrix C(lig, 1);
  for (int i = 0; i < lig; i++)
    C.set(i, 0, get(i, j));

  return C;
}

/*****************************************************************************/

Matrix Matrix::extract_line(int i) const
{
  if (i < 0 || i >= lig)
    return Matrix(0, 0);

  Matrix L(1, col);
  for (int j = 0; j < col; j++)
    L.set(0, j, get(i, j));

  return L;
}

/*****************************************************************************/

Matrix Matrix::extract_lines(int line_min, int line_max) const
{
  return select_lines(line_min, line_max);
}

Matrix Matrix::extract_columns(int col_min, int col_max) const
{
  return select_columns(col_min, col_max);
}

/*****************************************************************************/

void Matrix::set_line(int i, Matrix l)
{
  if (l.col != col)
    return;
  for (int j = 0; j < col; j++)
    set(i, j, l.get(0, j));
}

void Matrix::set_column(int j, Matrix c)
{
  if (c.lig != lig)
    return;
  for (int i = 0; i < lig; i++)
    set(i, j, c.get(i, 0));
}

/*****************************************************************************/

Matrix Matrix::extract_minor(int i0, int j0) const
{
  if ((i0 < 0) || (j0 < 0) || (i0 >= lig) || (j0 >= col) || (lig == 1) || (col == 1))
    return Matrix(0, 0);

  Matrix the_minor(lig - 1, col - 1);
  for (int i = 0; i < lig - 1; i++)
    for (int j = 0; j < col - 1; j++)
    {
      int i_get, j_get;
      if (i < i0)
        i_get = i;
      else
        i_get = i + 1;
      if (j < j0)
        j_get = j;
      else
        j_get = j + 1;
      the_minor.set(i, j, get(i_get, j_get));
    }

  return the_minor;
}

/*****************************************************************************/

Matrix Matrix::insert_at_line(const Matrix& m, int i0) const
{
  if (col != m.col)
    LA_ERROR("insert_at_line");

  Matrix result(lig + m.lig, col);
  for (int i = 0; i < i0; i++)
    for (int j = 0; j < col; j++)
      result.set(i, j, get(i, j));
  for (int i = i0; i < i0 + m.lig; i++)
    for (int j = 0; j < col; j++)
      result.set(i, j, m.get(i - i0, j));
  for (int i = i0 + m.lig; i < lig + m.lig; i++)
    for (int j = 0; j < col; j++)
      result.set(i, j, get(i - m.lig, j));

  return result;
}

/*****************************************************************************/

Matrix Matrix::insert_at_column(const Matrix& m, int j0) const
{
  if (lig != m.lig)
    LA_ERROR("insert_at_column");

  Matrix result(lig, col + m.col);
  for (int j = 0; j < j0; j++)
    for (int i = 0; i < lig; i++)
      result.set(i, j, get(i, j));
  for (int j = j0; j < j0 + m.col; j++)
    for (int i = 0; i < lig; i++)
      result.set(i, j, m.get(i, j - j0));
  for (int j = j0 + m.col; j < col + m.col; j++)
    for (int i = 0; i < lig; i++)
      result.set(i, j, get(i, j - m.col));

  return result;
}

/*****************************************************************************/

void Matrix::exchange_column(int j1, int j2)
{
  if ((j1 < 0) || (j2 < 0) || (j1 >= col) || (j2 >= col))
    LA_ERROR("exchange_column");

  for (int i = 0; i < lig; i++)
  {
    scalar tmp = get(i, j1);
    set(i, j1, get(i, j2));
    set(i, j2, tmp);
  }
}

/*****************************************************************************/

void Matrix::exchange_line(int i1, int i2)
{
  if ((i1 < 0) || (i2 < 0) || (i1 >= lig) || (i2 >= lig))
    LA_ERROR("exchange_line");

  for (int j = 0; j < col; j++)
  {
    scalar tmp = get(i1, j);
    set(i1, j, get(i2, j));
    set(i2, j, tmp);
  }
}

/*****************************************************************************/

vector<Matrix> Matrix::extract_matrix_list(const vector<Matrix>& l, int id_min, int id_max)
{
  if (id_max == -1)
    id_max = l.size() - 1;
  vector<Matrix> result;
  for (int i = id_min; i <= id_max; i++)
    result.push_back(l[i]);
  return result;
}

/*****************************************************************************/

vector<Matrix> Matrix::select_from_indexes(const vector<Matrix>& l, vector<int> indexes)
{
  vector<Matrix> selected;
  for (unsigned int i = 0; i < indexes.size(); i++)
    selected.push_back(l[indexes[i]]);
  return selected;
}

/***************************************************************************/
/* operations on matrix and matrix lists                                   */
/***************************************************************************/

/*****************************************************************************/

Matrix operator/(const Matrix& v, double x)
{
  Matrix result(v.lig, v.col);
  for (unsigned int i = 0; i < v.size(); i++)
    result[i] = v[i] / x;
  return result;
}

/*****************************************************************************/

vector<Matrix> operator/(const vector<Matrix>& l, double x)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i] / x);
  return result;
}

/*****************************************************************************/

Matrix operator/(const Matrix& v, int x)
{
  Matrix result(v.lig, v.col);
  for (unsigned int i = 0; i < v.size(); i++)
    result[i] = v[i] / x;
  return result;
}

/*****************************************************************************/

vector<Matrix> operator/(const vector<Matrix>& l, int x)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i] / x);
  return result;
}

/*****************************************************************************/

Matrix operator*(int x, const Matrix& v)
{
  Matrix result(v.lig, v.col);
  for (unsigned int i = 0; i < v.size(); i++)
    result[i] = ((scalar)x) * v[i];
  return result;
}

vector<Matrix> operator*(int x, const vector<Matrix>& l)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(((scalar)x) * l[i]);
  return result;
}

/*****************************************************************************/

Matrix operator*(float x, const Matrix& v)
{
  Matrix result(v.lig, v.col);
  for (unsigned int i = 0; i < v.size(); i++)
    result[i] = ((scalar)x) * v[i];
  return result;
}

/*****************************************************************************/

vector<Matrix> operator*(float x, const vector<Matrix>& l)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(((scalar)x) * l[i]);
  return result;
}

/*****************************************************************************/

Matrix operator*(double x, const Matrix& v)
{
  Matrix result(v.lig, v.col);
  for (unsigned int i = 0; i < v.size(); i++)
    result[i] = ((scalar)x) * v[i];
  return result;
}

/*****************************************************************************/

vector<Matrix> operator*(double x, const vector<Matrix>& l)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(((scalar)x) * l[i]);
  return result;
}

/*****************************************************************************/

Matrix operator+(const Matrix& v1, const Matrix& v2)
{
  if (v1.size() != v2.size())
    LA_ERROR("matrix addition +:: bad dimensions");

  Matrix result(v1.lig, v1.col);
  for (unsigned int i = 0; i < v1.size(); i++)
    result[i] = v1[i] + v2[i];
  return result;
}

/*****************************************************************************/

vector<Matrix> operator+(const vector<Matrix>& l, const Matrix& m)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i] + m);
  return result;
}

/*****************************************************************************/

vector<Matrix> operator+(const Matrix& m, const vector<Matrix>& l)
{
  return l + m;
}

/*****************************************************************************/

void operator|=(Matrix& matrix1, const Matrix& matrix2)
{
  if (matrix1.col != matrix2.col)
    LA_ERROR("matrix vertical concatenation |= :: bad dimensions");
  matrix1.insert(matrix1.end(), matrix2.begin(), matrix2.end());
  matrix1.lig += matrix2.lig;
}

void operator&=(Matrix& matrix1, const Matrix& matrix2)
{
  if (matrix1.lig != matrix2.lig)
    LA_ERROR("matrix horizontal concatenation |= :: bad dimensions");
  matrix1.insert(matrix1.end(), matrix2.begin(), matrix2.end());
  matrix1.col += matrix2.col;
}

void operator+=(Matrix& v1, const Matrix& v2)
{
  if (v1.size() != v2.size())
    LA_ERROR("matrix addition += :: bad dimensions");

  for (unsigned int i = 0; i < v1.size(); i++)
    v1[i] += v2[i];
}

void operator-=(Matrix& v1, const Matrix& v2)
{
  if (v1.size() != v2.size())
    LA_ERROR("matrix addition-= :: bad dimensions");

  for (unsigned int i = 0; i < v1.size(); i++)
    v1[i] -= v2[i];
}

/*****************************************************************************/

void operator+=(vector<Matrix>& l, const Matrix& m)
{
  for (unsigned int i = 0; i < l.size(); i++)
    l[i] += m;
}

/*****************************************************************************/

Matrix operator-(const Matrix& v1, const Matrix& v2)
{
  if (v1.size() != v2.size())
  {
    cout << v1 << endl;
    cout << "-" << endl;
    cout << v2 << endl;
    LA_ERROR("matrix addition - :: bad dimensions");
  }

  Matrix result(v1.lig, v1.col);
  for (unsigned int i = 0; i < v1.size(); i++)
    result[i] = v1[i] - v2[i];
  return result;
}

/*****************************************************************************/

vector<Matrix> operator-(const vector<Matrix>& l, const Matrix& m)
{
  return l + ((-1.0) * m);
}

/*****************************************************************************/

vector<Matrix> operator-(const Matrix& m, const vector<Matrix>& l)
{
  return m + ((-1.0) * l);
}

/*****************************************************************************/

vector<Matrix> operator-(const vector<Matrix>& lm1, const vector<Matrix>& lm2)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < lm1.size(); i++)
    result.push_back(lm1[i] - lm2[i]);
  return result;
}

/*****************************************************************************/

Matrix operator-(const Matrix& m)
{
  return (-1.0) * m;
}

/*****************************************************************************/

vector<Matrix> operator-(const vector<Matrix>& l)
{
  return (-1.0) * l;
}

/*****************************************************************************/

Matrix operator*(const Matrix& m1, const Matrix& m2)
{
  if (m1.col != m2.lig)
    LA_ERROR("matrix product:: bad dimensions");

  Matrix result(m1.lig, m2.col);
  for (int i = 0; i < m1.lig; i++)
    for (int j = 0; j < m2.col; j++)
    {
      scalar s = 0.0;
      for (int k = 0; k < m1.col; k++)
        s += m1.get(i, k) * m2.get(k, j);
      result.set(i, j, s);
    }

  return result;
}

/*****************************************************************************/

vector<Matrix> operator*(const vector<Matrix>& l, const Matrix& m)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(l[i] * m);
  return result;
}

/*****************************************************************************/

vector<Matrix> operator*(const Matrix& m, const vector<Matrix>& l)
{
  vector<Matrix> result;
  for (unsigned int i = 0; i < l.size(); i++)
    result.push_back(m * l[i]);
  return result;
}

/*****************************************************************************/

void operator*=(Matrix& m1, scalar l)
{
  for (unsigned int i = 0; i < m1.size(); i++)
    m1[i] *= l;
}

/*****************************************************************************/

Matrix Matrix::transpose() const
{
  Matrix result(col, lig);
  for (int i = 0; i < lig; i++)
    for (int j = 0; j < col; j++)
      result.set(j, i, get(i, j));
  return result;
}

/*****************************************************************************/

Matrix Matrix::cross_product(Matrix x1, Matrix x2)
{
  if (x1.lig != 3 || x1.col != 1 || x2.lig != 3 || x2.col != 1)
    LA_ERROR("cross product of bad dimension vector");

  Matrix P(3, 1, 0.0);
  P[0] = x1[1] * x2[2] - x1[2] * x2[1];
  P[1] = x1[2] * x2[0] - x1[0] * x2[2];
  P[2] = x1[0] * x2[1] - x1[1] * x2[0];

  return P;
}

/***************************************************************************/

Matrix Matrix::linear_interpol(double x1, Matrix y1, double x2, Matrix y2, double x)
{
  if (x1 == x2)
    return 0.5 * (y1 + y2);
  return y1 + (x - x1) / (x2 - x1) * (y2 - y1);
}

/***************************************************************************/
/* Predicate testing matrix format                                         */
/***************************************************************************/

bool Matrix::lower_triangular_matrix(const Matrix& m, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < m.lig; i++)
  {
    for (int j = 0; j < m.col; j++)
    {
      if (i < j && real_abs(m.get(i, j)) > max)
        max = real_abs(m.get(i, j));
    }
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::lower_triangular_matrix(const Matrix& m)
{
  return lower_triangular_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::unit_lower_triangular_matrix(const Matrix& m, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < m.lig; i++)
  {
    for (int j = 0; j < m.col; j++)
    {
      if (i < j && real_abs(m.get(i, j)) > max)
        max = real_abs(m.get(i, j));
      if (i == j && real_abs(m.get(i, j) - 1.0) > max)
        max = real_abs(m.get(i, j) - 1.0);
    }
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::unit_lower_triangular_matrix(const Matrix& m)
{
  return unit_lower_triangular_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::upper_triangular_matrix(const Matrix& m, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < m.lig; i++)
  {
    for (int j = 0; j < m.col; j++)
    {
      if (i > j && real_abs(m.get(i, j)) > max)
        max = real_abs(m.get(i, j));
    }
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::upper_triangular_matrix(const Matrix& m)
{
  return upper_triangular_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::hessenberg_matrix(const Matrix& m, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < m.lig; i++)
  {
    for (int j = 0; j < m.col; j++)
    {
      if (i > j + 1 && real_abs(m.get(i, j)) > max)
        max = real_abs(m.get(i, j));
    }
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::hessenberg_matrix(const Matrix& m)
{
  return hessenberg_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::orthogonal_matrix(const Matrix& m, scalar* error)
{
  Matrix prod1 = m.transpose() * m;
  Matrix prod2 = m * m.transpose();
  scalar error1 = 0.0;
  scalar error2 = 0.0;
  bool boolean = equal_matrix(prod1, Matrix::identity(prod1.lig), &error1) &&
                 equal_matrix(prod2, Matrix::identity(prod2.lig), &error2);
  if (error != NULL)
    *error = ((error1 > error2) ? error1 : error2);
  return boolean;
}

bool Matrix::orthogonal_matrix(const Matrix& m)
{
  return orthogonal_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::permutation_matrix(const Matrix& m, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < m.lig; i++)
  {
    scalar count = 0.0;
    for (int j = 0; j < m.col; j++)
    {
      if (!is_zero(m.get(i, j)) && !is_zero(m.get(i, j) - 1.0))
        return false;
      count += m.get(i, j);
    }
    if (real_abs(count - 1.0) > max)
      max = real_abs(count - 1.0);
  }
  for (int j = 0; j < m.lig; j++)
  {
    scalar count = 0.0;
    for (int i = 0; i < m.col; i++)
    {
      if (!is_zero(m.get(i, j)) && !is_zero(m.get(i, j) - 1.0))
        return false;
      count += m.get(i, j);
    }
    if (real_abs(count - 1.0) > max)
      max = real_abs(count - 1.0);
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::permutation_matrix(const Matrix& m)
{
  return permutation_matrix(m, NULL);
}

/*****************************************************************************/

bool Matrix::bidiagonal_matrix(const Matrix& A, scalar* error)
{
  scalar max = 0.0;
  int m = A.lig;
  int n = A.col;
  if (m >= n)
  {
    if (!lower_triangular_matrix(A, error))
      return false;
    for (int i = 0; i < A.lig; i++)
    {
      for (int j = 0; j < A.col; j++)
      {
        if (i > j + 1 && real_abs(A.get(i, j)) > max)
          max = real_abs(A.get(i, j));
      }
    }
    if (error != NULL)
      *error = max;
    return is_zero(max);
  }
  else
  {
    if (!upper_triangular_matrix(A, error))
      return false;
    for (int i = 0; i < A.lig; i++)
    {
      for (int j = 0; j < A.col; j++)
      {
        if (i < j - 1 && real_abs(A.get(i, j)) > max)
          max = real_abs(A.get(i, j));
      }
    }
    if (error != NULL)
      *error = max;
    return is_zero(max);
  }
}

bool Matrix::bidiagonal_matrix(const Matrix& A)
{
  return bidiagonal_matrix(A, NULL);
}

/*****************************************************************************/

bool Matrix::diagonal_matrix(const Matrix& A, scalar* error)
{
  scalar max = 0.0;
  for (int i = 0; i < A.lig; i++)
  {
    for (int j = 0; j < A.col; j++)
    {
      if (i != j && real_abs(A.get(i, j)) > max)
        max = real_abs(A.get(i, j));
    }
  }
  if (error != NULL)
    *error = max;
  return is_zero(max);
}

bool Matrix::diagonal_matrix(const Matrix& A)
{
  return diagonal_matrix(A, NULL);
}

/***************************************************************************/
/* Auxiliary functions for complex operations                              */
/***************************************************************************/

Matrix Matrix::Householder_vector(Matrix& m, int lig, int col)
{
  if (col >= m.col || lig >= m.lig)
    LA_ERROR("Householder::bad index");
  // compute the norm of the column : alpha
  scalar sum = 0.0;
  for (int k = lig; k < m.lig; k++)
  {
    sum += m.get(k, col) * m.get(k, col);
  }
  scalar alpha = sqrt(sum);
  // choise the sign of alpha : must maximize the denominator : norm
  scalar tmp1 = (m.get(lig, col) - alpha);
  scalar tmp2 = (m.get(lig, col) + alpha);
  scalar norm;
  tmp1 = tmp1 * tmp1;
  tmp2 = tmp2 * tmp2;
  if (tmp1 > tmp2)
  {
    norm = sum - m.get(lig, col) * m.get(lig, col) + tmp1;
  }
  else
  {
    norm = sum - m.get(lig, col) * m.get(lig, col) + tmp2;
    alpha = -alpha;
  }
  // null case
  if (is_zero(norm))
  {
    Matrix U(m.lig - lig, 1, 0.0);
    return U;
  }
  // construct the Householder vector
  norm = sqrt(norm);
  Matrix U(m.lig - lig, 1);
  U.set(0, 0, (m.get(lig, col) - alpha) / norm);
  for (int k = lig + 1; k < m.lig; k++)
  {
    U.set(k - lig, 0, m.get(k, col) / norm);
  }
  return U;
}

/*****************************************************************************/

Matrix Matrix::Householder_vector_right(Matrix& m, int col, int lig)
{
  if (col >= m.col || lig >= m.lig)
    LA_ERROR("Householder::bad index");
  // compute the norm of the column : alpha
  scalar sum = 0.0;
  for (int k = col; k < m.col; k++)
  {
    sum += m.get(lig, k) * m.get(lig, k);
  }
  scalar alpha = sqrt(sum);
  // choise the sign of alpha : must maximize the denominator : norm
  scalar tmp1 = (m.get(lig, col) - alpha);
  scalar tmp2 = (m.get(lig, col) + alpha);
  scalar norm;
  tmp1 = tmp1 * tmp1;
  tmp2 = tmp2 * tmp2;
  if (tmp1 > tmp2)
  {
    norm = sum - m.get(lig, col) * m.get(lig, col) + tmp1;
  }
  else
  {
    norm = sum - m.get(lig, col) * m.get(lig, col) + tmp2;
    alpha = -alpha;
  }
  // null case
  if (is_zero(norm))
  {
    Matrix U(m.col - col, 1, 0.0);
    return U;
  }
  // construct the Householder vector
  norm = sqrt(norm);
  Matrix U(m.col - col, 1);
  U.set(0, 0, (m.get(lig, col) - alpha) / norm);
  for (int k = col + 1; k < m.col; k++)
  {
    U.set(k - col, 0, m.get(lig, k) / norm);
  }
  return U;
}

/*****************************************************************************/

void Matrix::Householder_prod(Matrix& m, Matrix& U, int i)
{
  if (U.lig != m.lig - i)
    LA_ERROR("Householder prod left::bad size");
  for (int k = 0; k < m.col; k++)
  {
    scalar dot_prod = 0.0;
    for (int l = i; l < m.lig; l++)
    {
      dot_prod += m.get(l, k) * U.get(l - i, 0);
    }
    for (int l = i; l < m.lig; l++)
    {
      m.set(l, k, m.get(l, k) - 2.0 * dot_prod * U.get(l - i, 0));
    }
  }
}

/*****************************************************************************/

void Matrix::Householder_prod_right(Matrix& m, Matrix& U, int i)
{
  if (U.lig != m.col - i)
    LA_ERROR("Householder prod right::bad size");
  for (int k = 0; k < m.lig; k++)
  {
    scalar dot_prod = 0.0;
    for (int l = i; l < m.col; l++)
    {
      dot_prod += m.get(k, l) * U.get(l - i, 0);
    }
    for (int l = i; l < m.col; l++)
    {
      m.set(k, l, m.get(k, l) - 2.0 * dot_prod * U.get(l - i, 0));
    }
  }
}

/*****************************************************************************/

void Matrix::Householder_prod_optimized(Matrix& m, Matrix& U, int i)
{
  for (int k = i; k < m.col; k++)
  {
    scalar dot_prod = 0.0;
    for (int l = i; l < m.lig; l++)
    {
      dot_prod += m.get(l, k) * U.get(l - i, 0);
    }
    for (int l = i; l < m.lig; l++)
    {
      m.set(l, k, m.get(l, k) - 2.0 * dot_prod * U.get(l - i, 0));
    }
  }
}

/*****************************************************************************/

scalar Matrix::diagonal_product_matrix() const
{
  scalar prod = 1.0;
  for (int i = 0; i < lig; i++)
  {
    prod = prod * get(i, i);
  }
  return prod;
}

/*****************************************************************************/

pair<Matrix, bool> Matrix::PLU_Decomposition_optimized_for_determinant() const
{
  if (col != lig)
    LA_ERROR("PLU:: non square");
  int n = col;
  Matrix U = *this;
  bool sign = 0;
  for (int k = 0; k < n - 1; k++)
  {
    scalar pivot = U.get(k, k);
    int index = k;
    for (int i = k; i < n; i++)
    {
      if (real_abs(U.get(i, k)) > pivot)
      {
        pivot = real_abs(U.get(i, k));
        index = i;
      }
    }
    if (is_zero(pivot))
      return pair<Matrix, bool>(Matrix(0, 0), 0);
    if (index != k)
    {
      for (int j = k; j < n; j++)
      {
        scalar tmp = U.get(k, j);
        U.set(k, j, U.get(index, j));
        U.set(index, j, tmp);
      }
      sign = !sign;
    }
    for (int i = k + 1; i < n; i++)
    {
      scalar coef = U.get(i, k) / U.get(k, k);
      for (int j = k; j < n; j++)
      {
        U.set(i, j, U.get(i, j) - coef * U.get(k, j));
      }
    }
  }
  return pair<Matrix, bool>(U, sign);
}

/*****************************************************************************/

scalar Matrix::hypo(scalar a, scalar b)
{
  if (is_zero(a) && is_zero(b))
    return 0.0;
  else if (is_zero(a))
    return real_abs(b);
  else if (is_zero(b))
    return real_abs(a);
  else if (real_abs(a) > real_abs(b))
    return real_abs(a) * sqrt(1.0 + (b / a) * (b / a));
  else
    return real_abs(b) * sqrt(1.0 + (a / b) * (a / b));
}

/*****************************************************************************/

void Matrix::Givens_Transformation(int i, int j, triple<Matrix>& UBV)
{
  int m = UBV.second.lig;
  int n = UBV.second.col;
  if (i < 0 || j < 0 || i == j || i >= m || j >= n)
    LA_ERROR("Givens_Transformation:: bad index");
  scalar a, b;
  if (i > j)
  {
    a = UBV.second.get(j, j);
    b = UBV.second.get(i, j);
  }
  else
  {
    a = UBV.second.get(i, i);
    b = UBV.second.get(i, j);
  }
  scalar r = hypo(a, b);
  scalar c = a / r;
  scalar s = -b / r;

  if (i > j)
  {
    Matrix L1 = UBV.second.extract_line(j);
    Matrix L2 = UBV.second.extract_line(i);
    Matrix CC1 = UBV.first.extract_column(j);
    Matrix CC2 = UBV.first.extract_column(i);
    for (int k = 0; k < n; k++)
    {
      UBV.second.set(i, k, c * L2.get(0, k) + s * L1.get(0, k));
      UBV.second.set(j, k, c * L1.get(0, k) - s * L2.get(0, k));
    }
    for (int k = 0; k < m; k++)
    {
      UBV.first.set(k, j, -s * CC2.get(k, 0) + c * CC1.get(k, 0));
      UBV.first.set(k, i, s * CC1.get(k, 0) + c * CC2.get(k, 0));
    }
  }
  else
  {
    Matrix C1 = UBV.second.extract_column(i);
    Matrix C2 = UBV.second.extract_column(j);
    Matrix LL1 = UBV.third.extract_line(i);
    Matrix LL2 = UBV.third.extract_line(j);
    for (int k = 0; k < m; k++)
    {
      UBV.second.set(k, i, -s * C2.get(k, 0) + c * C1.get(k, 0));
      UBV.second.set(k, j, s * C1.get(k, 0) + c * C2.get(k, 0));
    }
    for (int k = 0; k < n; k++)
    {
      UBV.third.set(j, k, c * LL2.get(0, k) + s * LL1.get(0, k));
      UBV.third.set(i, k, c * LL1.get(0, k) - s * LL2.get(0, k));
    }
  }
}

/***************************************************************************/
/* Complex operations on matrix                                            */
/***************************************************************************/

pair<bool, Matrix> Matrix::gauss_inverse() const
{
  if (col != lig)
    return pair<bool, Matrix>(false, Matrix(0, 0));
  int n = lig;
  Matrix A = *this;

  // Normalize coefficients
  scalar norm_max = A.norm_max();
  if (norm_max != 1.0)
    A = (1.0 / norm_max) * A;

  Matrix inv = Matrix::identity(n);
  for (int k = 0; k < n; k++)
  {
    // select the greater pivot
    scalar pivot = 0.0;
    int index = 0;
    for (int i = k; i < n; i++)
    {
      if (real_abs(A.get(i, k)) > real_abs(pivot))
      {
        pivot = A.get(i, k);
        index = i;
      }
    }
    if (is_zero(pivot))
      return pair<bool, Matrix>(false, inv);
    // swap row
    if (index != k)
    {
      for (int j = 0; j < n; j++)
      {
        scalar tmp = A.get(k, j);
        A.set(k, j, A.get(index, j));
        A.set(index, j, tmp);
        tmp = inv.get(k, j);
        inv.set(k, j, inv.get(index, j));
        inv.set(index, j, tmp);
      }
    }
    // normalize row
    for (int j = 0; j < n; j++)
    {
      A.set(k, j, A.get(k, j) / pivot);
      inv.set(k, j, inv.get(k, j) / pivot);
    }
    // add pivot row
    for (int i = 0; i < n; i++)
    {
      if (i == k)
        continue;
      scalar coef = -A.get(i, k) / A.get(k, k);
      for (int j = k; j < n; j++)
      {
        A.set(i, j, A.get(i, j) + coef * A.get(k, j));
      }
      for (int j = 0; j < n; j++)
      {
        inv.set(i, j, inv.get(i, j) + coef * inv.get(k, j));
      }
    }
  }
  return pair<bool, Matrix>(true, (1.0 / norm_max) * inv);
}

/*****************************************************************************/

Matrix Matrix::pseudo_inverse() const
{
  Matrix A = (*this);

  // Normalize coefficients
  scalar norm_max = A.norm_max();
  if (is_zero(norm_max))
    return null_matrix(0, 0);
  if (norm_max != 1.0)
    A = (1.0 / norm_max) * A;

  triple<Matrix> SVD = A.SVD_Decomposition();

  for (int i = 0; i < SVD.second.lig && i < SVD.second.col; i++)
  {
    if (!is_zero(SVD.second.get(i, i)))
      SVD.second.set(i, i, 1.0 / SVD.second.get(i, i));
  }

  Matrix prod = SVD.third.transpose() * SVD.second.transpose() * SVD.first.transpose();
  return (1.0 / norm_max) * prod;
  return prod;
}

/*****************************************************************************/

Matrix Matrix::Cholesky() const
{
  if (lig != col)
    LA_ERROR("Cholesky: non square");
  // Caution, to speed up, one does not verify that
  // the matrix is symetric positive definite
  int n = lig;
  Matrix L(n, n, 0.0);
  Matrix A = (*this);

  // Normalize coefficients
  scalar norm_max = A.norm_max();
  if (norm_max != 1.0)
    A = (1.0 / norm_max) * A;

  for (int i = 0; i < n; i++)
  {
    // Compute L[i,i]
    scalar sum = A.get(i, i);
    for (int k = 0; k < i; k++)
      sum -= L.get(i, k) * L.get(i, k);
    L.set(i, i, sqrt(sum));

    // Compute L[j,i]
    for (int j = i + 1; j < n; j++)
    {
      sum = A.get(j, i);
      for (int k = 0; k < i; k++)
        sum -= L.get(i, k) * L.get(j, k);
      L.set(j, i, sum / L.get(i, i));
    }
  }
  return sqrt(norm_max) * L;
}

/*****************************************************************************/

triple<Matrix> Matrix::PLU_Decomposition() const
{
  if (col != lig)
    LA_ERROR("PLU:: non square");
  int n = col;
  Matrix P = Matrix::identity(n);
  Matrix L = Matrix(n, n, 0.0);
  Matrix U = *this;

  // Normalize coefficients
  scalar norm_max = U.norm_max();
  if (norm_max != 1.0)
    U = (1.0 / norm_max) * U;

  for (int k = 0; k < n - 1; k++)
  {
    scalar pivot = U.get(k, k);
    int index = k;
    for (int i = k; i < n; i++)
    {
      if (real_abs(U.get(i, k)) > pivot)
      {
        pivot = real_abs(U.get(i, k));
        index = i;
      }
    }
    if (is_zero(pivot))
      return triple<Matrix>(Matrix(0, 0), Matrix(0, 0), Matrix(0, 0));
    if (index != k)
    {
      for (int j = 0; j < n; j++)
      {
        scalar tmp = P.get(k, j);
        P.set(k, j, P.get(index, j));
        P.set(index, j, tmp);
      }
      for (int j = k; j < n; j++)
      {
        scalar tmp = U.get(k, j);
        U.set(k, j, U.get(index, j));
        U.set(index, j, tmp);
      }
      for (int j = 0; j < n; j++)
      {
        scalar tmp = L.get(k, j);
        L.set(k, j, L.get(index, j));
        L.set(index, j, tmp);
      }
    }
    L.set(k, k, 1.0);
    for (int i = k + 1; i < n; i++)
    {
      scalar coef = U.get(i, k) / U.get(k, k);
      L.set(i, k, coef);
      for (int j = k; j < n; j++)
      {
        U.set(i, j, U.get(i, j) - coef * U.get(k, j));
      }
    }
    P.epsilon_filter();
    L.epsilon_filter();
    U.epsilon_filter();
  }
  L.set(n - 1, n - 1, 1.0);
  return triple<Matrix>(P.transpose(), L, norm_max * U);
}

/*****************************************************************************/

pair<Matrix, Matrix> Matrix::Hessenberg_Decomposition() const
{
  if (lig != col)
    LA_ERROR("Heisenberg:: matrix non square");
  int n = lig;
  Matrix P = Matrix::identity(n);
  Matrix H = *this;

  // Normalize coefficients
  scalar norm_max = H.norm_max();
  if (norm_max != 1.0)
    H = (1.0 / norm_max) * H;

  for (int k = 0; k < n - 2; k++)
  {
    Matrix U = Householder_vector(H, k + 1, k);
    Householder_prod(H, U, k + 1);
    Householder_prod_right(H, U, k + 1);
    Householder_prod_right(P, U, k + 1);
    H.epsilon_filter();
    P.epsilon_filter();
  }
  return pair<Matrix, Matrix>(P, norm_max * H);
}

/*****************************************************************************/

triple<Matrix> Matrix::Bidiagonal_Decomposition() const
{
  int m = lig;
  int n = col;
  Matrix U = Matrix::identity(m);
  Matrix B = *this;
  Matrix V = Matrix::identity(n);

  // Normalize coefficients
  scalar norm_max = B.norm_max();
  if (norm_max != 1.0)
    B = (1.0 / norm_max) * B;

  for (int k = 0; k < n && k < m; k++)
  {
    Matrix u;
    if (m >= n)
    {
      u = Householder_vector_right(B, k, k);
      Householder_prod_right(B, u, k);
      Householder_prod_right(V, u, k);
      if (k < n - 1 || m > n)
      {
        u = Householder_vector(B, k + 1, k);
        Householder_prod(B, u, k + 1);
        Householder_prod(U, u, k + 1);
      }
    }
    else
    {
      u = Householder_vector(B, k, k);
      Householder_prod(B, u, k);
      Householder_prod(U, u, k);
      if (k < n - 1)
      {
        u = Householder_vector_right(B, k + 1, k);
        Householder_prod_right(B, u, k + 1);
        Householder_prod_right(V, u, k + 1);
      }
    }
    B.epsilon_filter();
    U.epsilon_filter();
    V.epsilon_filter();
  }
  return triple<Matrix>(U.transpose(), norm_max * B, V.transpose());
}

/*****************************************************************************/

pair<Matrix, Matrix> Matrix::QR_Decomposition() const
{
  Matrix Q = Matrix::identity(lig);
  Matrix R = *this;

  // Normalize coefficients
  scalar norm_max = R.norm_max();
  if (norm_max != 1.0)
    R = (1.0 / norm_max) * R;

  int rank = (lig - 1 < col) ? lig - 1 : col;
  for (int i = 0; i < rank; i++)
  {
    Matrix U = Householder_vector(R, i, i);
    Householder_prod_optimized(R, U, i);
    Householder_prod(Q, U, i);
    Q.epsilon_filter();
    R.epsilon_filter();
  }
  return pair<Matrix, Matrix>(Q.transpose(), norm_max * R);
}

/*****************************************************************************/

triple<Matrix> Matrix::SVD_Decomposition() const
{
  Matrix A = (*this);

  // Normalize coefficients
  scalar norm_max = A.norm_max();
  if (norm_max != 1.0)
    A = (1.0 / norm_max) * A;

  triple<Matrix> UBV = A.Bidiagonal_Decomposition();

  int m = UBV.second.lig;
  int n = UBV.second.col;
  int count = 1;
  if (m >= n)
  {
    while (count != 0)
    {
      count = 0;
      for (int k = 0; k + 1 < n; k++)
      {
        if (real_abs(UBV.second.get(k + 1, k)) > ll_epsilon)
        {
          Givens_Transformation(k + 1, k, UBV);
          count++;
        }
        if (real_abs(UBV.second.get(k, k + 1)) > ll_epsilon)
        {
          Givens_Transformation(k, k + 1, UBV);
          count++;
        }
      }
      if (m > n && real_abs(UBV.second.get(n, n - 1)) > ll_epsilon)
      {
        Givens_Transformation(n, n - 1, UBV);
        count++;
      }
    }
  }
  else
  {
    while (count != 0)
    {
      count = 0;
      for (int k = 0; k + 1 < m; k++)
      {
        if (real_abs(UBV.second.get(k, k + 1)) > ll_epsilon)
        {
          Givens_Transformation(k, k + 1, UBV);
          count++;
        }
        if (real_abs(UBV.second.get(k + 1, k)) > ll_epsilon)
        {
          Givens_Transformation(k + 1, k, UBV);
          count++;
        }
      }
      if (real_abs(UBV.second.get(m - 1, m)) > ll_epsilon)
      {
        Givens_Transformation(m - 1, m, UBV);
        count++;
      }
    }
  }
  for (int i = 0; i < UBV.second.lig; i++)
  {
    for (int j = 0; j < UBV.second.col; j++)
    {
      if (real_abs(UBV.second.get(i, j)) < ll_epsilon)
        UBV.second.set(i, j, 0.0);
    }
  }
  UBV.second = norm_max * UBV.second;

  return UBV;
}

/*****************************************************************************/

double EV_RATIO = 0.000001;
int EV_TURN_MAX = 10000;

// TODO: EN CHANTIER: d'abord la forme Hessenberg
/*Matrix Matrix::Eigenvalues() const {
  scalar criterion = norm_max() * EV_RATIO;
  int turn_nb = 0;

  Matrix E = *this;
  Matrix OLD_E = E;
  scalar error = criterion+1;
  while (turn_nb++ < EV_TURN_MAX && error > criterion) {
    pair<Matrix,Matrix> QR = E.QR_Decomposition();
    E = QR.second * QR.first;
    error = (E - OLD_E).norm_max();
    OLD_E = E;
  }
  Matrix V(lig, 1, 0.0);
  for (int k=0; k<lig; k++) V.set(k,0, E.get(k,k));
  return V;
}*/

/*****************************************************************************/

scalar Matrix::determinant() const
{
  if (col != lig)
    return 0.0;
  pair<Matrix, bool> Usign = (*this).PLU_Decomposition_optimized_for_determinant();
  if (Usign.first.col == 0 || Usign.first.lig == 0)
    return 0.0;
  else if (Usign.second)
    return -Usign.first.diagonal_product_matrix();
  else
    return Usign.first.diagonal_product_matrix();
}

/*****************************************************************************/

Matrix Matrix::forwardSubstitution(const Matrix& Lx, const Matrix& b)
{
  if (b.col != 1)
    LA_ERROR("forwardSubstitution:: b not vector");
  if (Lx.col != Lx.lig)
    LA_ERROR("forwardSubstitution:: L not square");
  if (Lx.lig != b.lig)
    LA_ERROR("forwardSubstitution:: not same rows");
  int n = b.lig;

  Matrix x(n, 1);
  for (int i = 0; i < n; i++)
  {
    scalar sum = b[i];
    for (int j = 0; j <= i - 1; j++)
    {
      sum -= Lx.get(i, j) * x[j];
    }
    x[i] = sum / Lx.get(i, i);
  }

  return x;
}
Matrix Matrix::backwardSubstitution(const Matrix& Ux, const Matrix& b)
{
  if (b.col != 1)
    LA_ERROR("backwardSubstitution:: b not vector");
  if (Ux.col != Ux.lig)
    LA_ERROR("backwardSubstitution:: L not square");
  if (Ux.lig != b.lig)
    LA_ERROR("backwardSubstitution:: not same rows");
  int n = b.lig;

  Matrix x(n, 1);
  for (int i = n - 1; i >= 0; i--)
  {
    scalar sum = b[i];
    for (int j = i + 1; j < n; j++)
    {
      sum -= Ux.get(i, j) * x[j];
    }
    x[i] = sum / Ux.get(i, i);
  }

  return x;
}

/*****************************************************************************/

/*****************************************************************************/

Matrix Matrix::mean(const vector<Matrix>& matrix_set)
{
  int n = matrix_set.size();

  if (n == 0)
    return Matrix(0, 0);

  Matrix s = matrix_set[0];
  for (int k = 1; k < n; k++)
    s = s + matrix_set[k];

  s = (1.0 / n) * s;
  return s;
}

/*****************************************************************************/

Matrix Matrix::mean_of_lines()
{
  Matrix m(1, col, 0.0);
  if (lig == 0)
    null_matrix(0, 0);
  for (int i = 0; i < lig; i++)
  {
    for (int j = 0; j < col; j++)
      m.set(0, j, m.get(0, j) + get(i, j));
  }
  return (1.0 / (double)lig) * m;
}

Matrix Matrix::std_dev_of_lines()
{
  Matrix sd(1, col, 0.0);
  Matrix sum(1, col, 0.0);
  Matrix sum_sq(1, col, 0.0);
  if (lig == 0)
    null_matrix(0, 0);
  for (int i = 0; i < lig; i++)
  {
    for (int j = 0; j < col; j++)
    {
      sum.set(0, j, sum.get(0, j) + get(i, j));
      sum_sq.set(0, j, sum_sq.get(0, j) + get(i, j) * get(i, j));
    }
  }
  sum = (1.0 / (double)lig) * sum;
  sum_sq = (1.0 / (double)lig) * sum_sq;

  for (int j = 0; j < col; j++)
    sd.set(0, j, sqrt(sum_sq.get(0, j) - sum.get(0, j) * sum.get(0, j)));
  return sd;
}

Matrix Matrix::max_of_lines()
{
  if (lig == 0)
    null_matrix(0, 0);
  Matrix maxl = extract_line(1);
  for (int i = 1; i < lig; i++)
  {
    for (int j = 0; j < col; j++)
      if (get(i, j) > maxl.get(0, j))
        maxl.set(0, j, get(i, j));
  }
  return maxl;
}

Matrix Matrix::min_of_lines()
{
  if (lig == 0)
    null_matrix(0, 0);
  Matrix minl = extract_line(1);
  for (int i = 1; i < lig; i++)
  {
    for (int j = 0; j < col; j++)
      if (get(i, j) < minl.get(0, j))
        minl.set(0, j, get(i, j));
  }
  return minl;
}

Matrix Matrix::mean_of_columns()
{
  return transpose().mean_of_lines().transpose();
}

Matrix Matrix::std_dev_of_columns()
{
  return transpose().std_dev_of_lines().transpose();
}

Matrix Matrix::max_of_columns()
{
  return transpose().max_of_lines().transpose();
}

Matrix Matrix::min_of_columns()
{
  return transpose().min_of_lines().transpose();
}

/*****************************************************************************/

Matrix Matrix::mean_pow(const vector<Matrix>& matrix_set, double p)
{
  int n = matrix_set.size();

  if (n == 0)
    return Matrix(0, 0);

  Matrix s = matrix_set[0];
  for (int k = 1; k < n; k++)
    for (unsigned int i = 0; i < s.size(); i++)
      s[i] = s[i] + pow((matrix_set[k])[i], p);

  s = (1.0 / n) * s;
  return s;
}

/*****************************************************************************/

Matrix Matrix::std_dev(const vector<Matrix>& matrix_set)
{
  int n = matrix_set.size();

  if (n == 0)
    return Matrix(0, 0);

  Matrix s = matrix_set[0];
  Matrix s2(s.lig, s.col, 0.0);
  for (unsigned int k = 0; k < s.size(); k++)
    s2[k] = (matrix_set[0])[k] * (matrix_set[0])[k];

  for (int m = 1; m < n; m++)
  {
    s = s + matrix_set[m];
    for (unsigned int k = 0; k < s.size(); k++)
      s2[k] = s2[k] + (matrix_set[m])[k] * (matrix_set[m])[k];
  }

  s = (1.0 / n) * s;
  s2 = (1.0 / n) * s2;

  Matrix std_dev(s.lig, s.col, 0.0);
  for (unsigned int k = 0; k < s.size(); k++)
  {
    double d = s2[k] - s[k] * s[k];
    std_dev[k] = (d >= 0.0) ? sqrt(d) : 0.0;
  }

  return std_dev;
}

/*****************************************************************************/

vector<Matrix> Matrix::normalize_samples(const vector<Matrix>& samples)
{
  Matrix mu = Matrix::mean(samples);
  Matrix sigma = Matrix::std_dev(samples);
  vector<Matrix> normalized_samples = samples - mu;
  int siz = mu.size();
  for (unsigned int k = 0; k < samples.size(); k++)
    for (int i = 0; i < siz; i++)
      if (!is_zero(sigma[i]))
        (normalized_samples[k])[i] = (normalized_samples[k])[i] / sigma[i];
  return normalized_samples;
}

/*****************************************************************************/

Matrix Matrix::matrix_max(const vector<Matrix>& matrix_set)
{
  int n = matrix_set.size();
  if (n == 0)
    return Matrix(0, 0);

  Matrix res = matrix_set[0];
  for (int m = 1; m < n; m++)
    for (unsigned int k = 0; k < res.size(); k++)
      res[k] = the_max(res[k], (matrix_set[m])[k]);

  return res;
}

/*****************************************************************************/

Matrix Matrix::matrix_min(const vector<Matrix>& matrix_set)
{
  int n = matrix_set.size();
  if (n == 0)
    return Matrix(0, 0);

  Matrix res = matrix_set[0];
  for (int m = 1; m < n; m++)
    for (unsigned int k = 0; k < res.size(); k++)
      res[k] = the_min(res[k], (matrix_set[m])[k]);

  return res;
}

/*****************************************************************************/

int Matrix::idx_of_max(const vector<Matrix>& matrix_set, int min_idx, int max_idx, int l, int c)
{
  if (min_idx < 0)
    min_idx = 0;
  if (max_idx < 0)
    max_idx = 0;
  if ((unsigned int)max_idx >= matrix_set.size())
    max_idx = matrix_set.size();
  if (max_idx < min_idx)
    return 0;
  int idx = min_idx;
  scalar max_v = matrix_set[min_idx].get(l, c);
  for (unsigned int i = min_idx; i <= (unsigned int)max_idx; i++)
    if (matrix_set[i].get(l, c) > max_v)
    {
      idx = i;
      max_v = matrix_set[i].get(l, c);
    }
  return idx;
}

/*****************************************************************************/

int Matrix::idx_of_min(const vector<Matrix>& matrix_set, int min_idx, int max_idx, int l, int c)
{
  if (min_idx < 0)
    min_idx = 0;
  if (max_idx < 0)
    max_idx = 0;
  if ((unsigned int)max_idx >= matrix_set.size())
    max_idx = matrix_set.size();
  if (max_idx < min_idx)
    return 0;
  int idx = min_idx;
  scalar min_v = matrix_set[min_idx].get(l, c);
  for (unsigned int i = min_idx; i <= (unsigned int)max_idx; i++)
    if (matrix_set[i].get(l, c) < min_v)
    {
      idx = i;
      min_v = matrix_set[i].get(l, c);
    }
  return idx;
}

/*****************************************************************************/

vector<Matrix> Matrix::smooth_sequence(const vector<Matrix>& seq, scalar discount_factor, int window_width)
{
  vector<Matrix> smooth_sequence;
  if (seq.size() == 0)
    return smooth_sequence;

  if (discount_factor > 1.0)
    LA_ERROR("smooth_sequence:: discount factor > 1.0");
  if (window_width <= 0)
    LA_ERROR("smooth_sequence:: window_width <= 0");

  int N = seq[0].lig;
  for (unsigned int k = 0; k < seq.size(); k++)
  {
    Matrix sum(N, 1, 0.0);
    scalar gamma = discount_factor;
    unsigned int i;
    for (i = 0; i <= k && i < (unsigned int)window_width; i++)
    {
      sum = sum + gamma * seq[the_max(0, (int)((int)k - (int)i))];
      gamma = gamma * discount_factor;
    }

    if (i != 0)
    {
      if (discount_factor < 1.0)
        sum = (1.0 / (discount_factor * (1.0 - pow(discount_factor, (int)i)) / (1.0 - discount_factor))) * sum;
      else
        sum = (1.0 / (scalar)i) * sum;
    }
    smooth_sequence.push_back(sum);
  }

  return smooth_sequence;
}

Matrix Matrix::smooth_lines(const Matrix& seq, scalar discount_factor, int window_width)
{
  Matrix smooth_sequence(seq.lig, seq.col, 0.0);
  if (seq.lig == 0)
    return smooth_sequence;

  if (discount_factor > 1.0)
    LA_ERROR("smooth_sequence:: discount factor > 1.0");
  if (window_width <= 0)
    LA_ERROR("smooth_sequence:: window_width <= 0");

  for (int k = 0; k < seq.lig; k++)
  {
    Matrix sum(1, seq.col, 0.0);
    scalar gamma = discount_factor;
    int i;
    for (i = 0; i <= k && i < window_width; i++)
    {
      sum = sum + (gamma * seq.extract_line(the_max(0, k - i)));
      gamma = gamma * discount_factor;
    }

    if (i != 0)
    {
      if (discount_factor < 1.0)
        sum = (1.0 / (discount_factor * (1.0 - pow(discount_factor, i)) / (1.0 - discount_factor))) * sum;
      else
        sum = (1.0 / (scalar)i) * sum;
    }
    smooth_sequence.set_line(k, sum);
  }

  return smooth_sequence;
}

/*****************************************************************************/

Matrix Matrix::extract_smoothed_line(const Matrix& seq, scalar discount_factor, int window_width, int idx)
{
  if (discount_factor > 1.0)
    LA_ERROR("smooth_sequence:: discount factor > 1.0");
  if (window_width <= 0)
    LA_ERROR("smooth_sequence:: window_width <= 0");

  Matrix sum(1, seq.col, 0.0);
  scalar gamma = 1.0;
  scalar gamma_sum = 0.0;
  int i;
  for (i = idx; (idx - i) < window_width; i--)
  {
    sum = sum + (gamma * seq.extract_line(the_max(0, i)));
    gamma_sum += gamma;
    gamma = gamma * discount_factor;
  }
  sum = (1.0 / gamma_sum) * sum;

  return sum;
}

/*****************************************************************************/

scalar Matrix::vertical_smoothed_value(scalar discount_factor, int window_width, int col_idx, int idx)
{
  if (discount_factor > 1.0)
    LA_ERROR("smooth_sequence:: discount factor > 1.0");
  if (window_width <= 0)
    LA_ERROR("smooth_sequence:: window_width <= 0");

  scalar sum = 0.0;
  scalar gamma = 1.0;
  scalar gamma_sum = 0.0;
  int i;
  for (i = idx; (idx - i) < window_width; i--)
  {
    sum = sum + (gamma * get(the_max(0, i), col_idx));
    gamma_sum += gamma;
    gamma = gamma * discount_factor;
  }
  sum = sum / gamma_sum;

  return sum;
}

/*****************************************************************************/

void Matrix::statistic_report(const vector<Matrix>& matrix_set)
{
  cout << "Mean:   \t";
  Matrix::mean(matrix_set).transpose().pp(cout);
  cout << "Std Dev:\t";
  Matrix::std_dev(matrix_set).transpose().pp(cout);
  cout << "Min:    \t";
  Matrix::matrix_min(matrix_set).transpose().pp(cout);
  cout << "Max:    \t";
  Matrix::matrix_max(matrix_set).transpose().pp(cout);
}

/*****************************************************************************/

Matrix Matrix::local_mean(const vector<Matrix>& matrix_set, int a, int b)
{
  if (matrix_set.size() == 0)
    return Matrix(0, 0);
  if (a < 0)
    a = 0;
  if ((unsigned int)a >= matrix_set.size())
    a = matrix_set.size() - 1;
  if (b < 0)
    b = 0;
  if ((unsigned int)b >= matrix_set.size())
    b = matrix_set.size() - 1;
  if (a > b)
    return Matrix(0, 0);

  Matrix s = matrix_set[a];
  for (int k = a + 1; k <= b; k++)
    s = s + matrix_set[k];

  s = (1.0 / (scalar)(b - a + 1)) * s;
  return s;
}

/*****************************************************************************/

vector<int> Matrix::get_neighboors(const vector<Matrix>& list, Matrix x, scalar radius, Matrix* basis_change)
{
  if (list.size() == 0)
    return vector<int>();

  vector<int> neighboors;
  for (int i = list.size() - 1; i >= 0; i--)
  {
    if ((x - list[i]).norm2(basis_change) < radius)
      neighboors.push_back(i);
  }
  return neighboors;
}

/*****************************************************************************/

vector<int> Matrix::get_closest_neighboors(vector<Matrix>& list, Matrix x, int n, scalar radius, Matrix* basis_change)
{
  if (n == 0 || list.size() == 0)
    return vector<int>();

  // Optimization for n==1
  if (n == 1)
  {
    int min_idx = 0;
    scalar min_d = radius + 1;
    for (unsigned int i = 0; i < list.size(); i++)
    {
      scalar d = (x - list[i]).norm2(basis_change);
      if (d < radius && d < min_d)
      {
        min_idx = i;
        min_d = d;
      }
    }
    vector<int> result;
    if (min_d < radius)
      result.push_back(min_idx);
    return result;
  }

  // General case
  std::list<pair<scalar, int> > dist_point_list;

  for (int i = list.size() - 1; i >= 0; i--)
  {
    scalar d = (x - list[i]).norm2(basis_change);

    if (d < radius)
    {  // Insert it into the list
      std::list<pair<scalar, int> >::iterator it = dist_point_list.begin();

      while ((it->first < d) && (it != dist_point_list.end()))
        it++;
      dist_point_list.insert(it, pair<scalar, int>(d, i));

      if ((int)dist_point_list.size() > n)
        dist_point_list.pop_back();
    }
  }

  vector<int> neighboors;
  for (std::list<pair<scalar, int> >::iterator it = dist_point_list.begin(); it != dist_point_list.end(); it++)
    neighboors.push_back(it->second);
  return neighboors;
}

/*****************************************************************************/

Matrix Matrix::jacobian(const vector<Matrix>& X, const vector<Matrix>& Y)
{
  if (X.size() != Y.size() || X.size() == 0)
    LA_ERROR("Jacobian: bad matrix vector size!");

  // One suppose that all vectors of X (resp. Y) have the same format
  Matrix X_mean = Matrix::mean(X);
  Matrix Y_mean = Matrix::mean(Y);

  // consider the linear case
  vector<Matrix> delta_X = X - X_mean;
  vector<Matrix> delta_Y = Y - Y_mean;
  delta_X.pop_back();
  delta_Y.pop_back();

  Matrix all_X = horizontal_concat(delta_X);
  Matrix all_Y = horizontal_concat(delta_Y);

  all_X.epsilon_filter();
  all_Y.epsilon_filter();

  Matrix pinv = all_X.pseudo_inverse();
  if (pinv != Matrix(0, 0))
  {
    Matrix Jacobian = all_Y * pinv;
    Jacobian.epsilon_filter();
    return Jacobian;
  }

  else
    return Matrix(0, 0);
}

/*****************************************************************************/

Matrix Matrix::affine_approximation(const vector<Matrix>& X, const vector<Matrix>& Y, const Matrix& x)
{
  if (X.size() != Y.size() || X.size() == 0)
    LA_ERROR("affine approximation: bad vector size");

  // One suppose that all vectors of X (resp. Y) have the same format
  Matrix X_mean = Matrix::mean(X);
  Matrix Y_mean = Matrix::mean(Y);

  Matrix J = Matrix::jacobian(X, Y);
  // if (J.null_matrix()) return Matrix(0,0);

  if (J.null_matrix())
    return Y_mean;
  else
    return Y_mean + (J * (x - X_mean));
}

/*****************************************************************************/

scalar Matrix::scalar_prod(const Matrix& x, const Matrix& y)
{
  if (x.lig != y.lig || x.col != y.col)
    LA_ERROR("scalar product of different size matrix");

  scalar p = 0.0;
  for (unsigned int k = 0; k < x.size(); k++)
    p += x[k] * y[k];

  return p;
}

/*****************************************************************************/

Matrix Matrix::normalize(Matrix* basis_change) const
{
  scalar n = norm2(basis_change);
  if (n == 0.0)
    return Matrix(lig, col, 0.0);
  else
    return (1.0 / n) * (*this);
}

Matrix Matrix::normalize() const
{
  return normalize(NULL);
}

/*****************************************************************************/

scalar Matrix::norm2(Matrix* basis_change) const
{
  Matrix v = *this;
  if (basis_change != NULL)
    v = (*basis_change) * (*this);

  scalar s = 0.0;
  for (unsigned int k = 0; k < size(); k++)
    s += v[k] * v[k];
  return sqrt(s);
}

/*****************************************************************************/
/*
scalar Matrix::distance2(const Matrix &other) const {
    scalar s = 0.0;
    unsigned int n = size();

    for (unsigned int k=0; k<n; k++) {
        scalar t = other[k] - (*this)[k];
        s += t*t;
    }

    return sqrt(s);
}
*/
/*****************************************************************************/

scalar Matrix::norm2() const
{
  scalar s = 0.0;
  unsigned int n = size();

  for (unsigned int k = 0; k < n; k++)
  {
    scalar t = (*this)[k];
    s += t * t;
  }

  return sqrt(s);
}

/*****************************************************************************/

scalar Matrix::norm_max(Matrix* basis_change) const
{
  Matrix v = *this;
  if (basis_change != NULL)
    v = (*basis_change) * (*this);

  scalar m = 0.0;
  for (unsigned int k = 0; k < size(); k++)
    if (m < real_abs(v[k]))
      m = real_abs(v[k]);
  return m;
}

/*****************************************************************************/

scalar Matrix::norm_max() const
{
  return norm_max(NULL);
}

/*****************************************************************************/

Matrix Matrix::abs() const
{
  Matrix abs_matrix(lig, col);
  for (unsigned int k = 0; k < size(); k++)
    abs_matrix[k] = real_abs((*this)[k]);
  return abs_matrix;
}

/*****************************************************************************/

Matrix Matrix::covariance(vector<Matrix> X, vector<Matrix> Y)
{
  if (X.size() != Y.size() || X.size() == 0)
    return Matrix(0, 0);

  int X_dim = X[0].lig;
  int Y_dim = Y[0].lig;

  Matrix X_mean = Matrix::mean(X);
  Matrix Y_mean = Matrix::mean(Y);

  vector<Matrix> X0 = X - X_mean;
  vector<Matrix> Y0 = Y - Y_mean;

  Matrix cov(Y_dim, X_dim);

  for (int i = 0; i < Y_dim; i++)
    for (int j = 0; j < X_dim; j++)
    {
      scalar xy_mean = 0.0;
      for (unsigned int k = 0; k < X.size(); k++)
        xy_mean += X0[k].get(j, 0) * Y0[k].get(i, 0);
      xy_mean = xy_mean / ((scalar)X.size());
      cov.set(i, j, xy_mean);
    }

  return cov;
}

/*****************************************************************************/

vector<double> Matrix::polynomial_approximation(vector<double> X, vector<double> Y, int n)
{
  vector<Matrix> powerX;
  for (int k = 0; k < (int)X.size(); k++)
  {
    Matrix power_x(n, 1, 0.0);
    double x_pow = X[k];
    for (int i = 0; i < n; i++)
    {
      power_x[i] = x_pow;
      x_pow *= X[k];
    }
    powerX.push_back(power_x);
  }

  vector<Matrix> Yvect;
  for (int k = 0; k < (int)Y.size(); k++)
    Yvect.push_back(Matrix(1, 1, Y[k]));

  pair<Matrix, Matrix> AB_approx = Matrix::LS_affine_approximation_equation(powerX, Yvect);

  vector<double> result = AB_approx.second;
  for (int i = 0; i < n; i++)
    result.push_back(AB_approx.first[i]);
  return result;
}

/*****************************************************************************/

Matrix Matrix::LS_affine_approximation(vector<Matrix> X, vector<Matrix> Y, Matrix x)
{
  if (X.size() != Y.size() || X.size() == 0)
    return Matrix(0, 0);

  Matrix Sxx = Matrix::covariance(X, X);
  Matrix Syx = Matrix::covariance(X, Y);

  Matrix InvSxx = Sxx.pseudo_inverse();
  if (InvSxx == Matrix(0, 0))
    return Matrix(0, 0);

  Matrix X_mean = Matrix::mean(X);
  Matrix Y_mean = Matrix::mean(Y);

  return Y_mean + (Syx * (InvSxx * (x - X_mean)));
}

pair<Matrix, Matrix> Matrix::LS_affine_approximation_equation(vector<Matrix> X, vector<Matrix> Y)
{
  if (X.size() != Y.size() || X.size() == 0)
    throw std::runtime_error("LS_affine_approximation_equation: bad input");

  Matrix Sxx = Matrix::covariance(X, X);
  Matrix Syx = Matrix::covariance(X, Y);

  Matrix InvSxx = Sxx.pseudo_inverse();
  if (InvSxx == Matrix(0, 0))
    throw std::runtime_error("LS_affine_approximation_equation: unable to compute inverse");

  Matrix X_mean = Matrix::mean(X);
  Matrix Y_mean = Matrix::mean(Y);

  Matrix A = Syx * InvSxx;
  Matrix B = Y_mean - (A * X_mean);
  return pair<Matrix, Matrix>(A, B);
}

pair<Matrix, Matrix> Matrix::LS_affine_approximation_equation(Matrix X, Matrix Y)
{
  vector<Matrix> X_vect, Y_vect;
  for (int i = 0; i < X.lig; i++)
  {
    X_vect.push_back(X.extract_line(i));
    Y_vect.push_back(Y.extract_line(i));
  }
  return LS_affine_approximation_equation(X_vect, Y_vect);
}

/***************************************************************************/
/* misc                                                                    */
/***************************************************************************/

void Matrix::epsilon_filter()
{
  if (size() == 0)
    return;

  scalar max_c = norm_max();
  if (max_c == 0.0)
    return;

  for (unsigned int i = 0; i < size(); i++)
  {
    if (real_abs((*this)[i]) < ll_epsilon)
      (*this)[i] = 0.0;
  }
}

/*****************************************************************************/

bool Matrix::null_matrix()
{
  for (unsigned int i = 0; i < size(); i++)
    if ((*this)[i] != 0.0)
      return false;
  return true;
}

/*****************************************************************************/

bool Matrix::operator==(const Matrix& other)
{
  if (lig != other.lig || col != other.col || size() != other.size())
    return false;
  for (unsigned int i = 0; i < size(); i++)
  {
    if (!is_zero((*this)[i] - other[i]))
      return false;
  }
  return true;
}

/*****************************************************************************/

bool Matrix::equal_matrix(const Matrix& m1, const Matrix& m2, scalar* error)
{
  scalar max = 0.0;
  if (m1.lig != m2.lig || m1.col != m2.col || m1.size() != m2.size())
    return false;
  for (unsigned int i = 0; i < m1.size(); i++)
  {
    if (real_abs(m1[i] - m2[i]) > max)
      max = real_abs(m1[i] - m2[i]);
  }
  if (error != NULL)
  {
    *error = max;
    return is_zero(*error);
  }
  else
    return is_zero(max);
}

/*****************************************************************************/

int matrix_pp_decimal_number = 6;

void Matrix::pp(ostream& os)
{
  os.precision(20);
  os << fixed;

  for (int i = 0; i < lig; i++)
  {
    for (int j = 0; j < col; j++)
      os << " " << get(i, j) << "\t";
    os << endl;
  }
}

/*****************************************************************************/

ostream& operator<<(ostream& os, const Matrix& m)
{
  os.precision(matrix_pp_decimal_number);
  os << fixed;

  for (int i = 0; i < m.lig; i++)
  {
    for (int j = 0; j < m.col; j++)
      os << m.get(i, j) << " ";
    if (i != m.lig - 1)
      os << endl;
  }

  return os;
}

/*****************************************************************************/

void Matrix::set_pp_precision(int n)
{
  matrix_pp_decimal_number = n;
}

/*****************************************************************************/

void Matrix::export_text(string filename, vector<Matrix> l)
{
  ofstream file;
  file.open(filename.c_str());
  for (unsigned int i = 0; i < l.size(); i++)
  {
    file << i << " ";
    l[i].transpose().pp(file);
  }
  file.close();
}

void Matrix::export_text(string filename, Matrix m)
{
  ofstream file;
  file.open(filename.c_str());
  m.pp(file);
  file.close();
}

/*****************************************************************************/

#define is_space(c) ((c) == ' ' || (c) == '\t' || (c) == '\n')

vector<Matrix> Matrix::vector_list_from_file(string filename)
{
  return Matrix::vector_list_from_file(filename, 0, -1);
}

vector<Matrix> Matrix::vector_list_from_file(string filename, int start_idx, int record_nb)
{
  vector<Matrix> list;
  string line;
  ifstream file(filename.c_str());
  int idx = 0;
  if (file.is_open())
  {
    while (file.good())
    {
      getline(file, line);
      if (idx < start_idx)
        continue;
      Matrix v = Matrix::vector_from_string(line);
      if (v.lig > 0)
        list.push_back(v);
      idx++;
      if (record_nb >= 0 && idx - start_idx >= record_nb)
        break;
    }
    file.close();
  }
  return list;
}

/*! \brief read a matrix from a file */
Matrix Matrix::matrix_from_file(string filename)
{
  vector<Matrix> list = Matrix::vector_list_from_file(filename);
  return Matrix::horizontal_concat(list).transpose();
}

void Matrix::vector_list_to_file(const vector<Matrix>& list, string filename)
{
  ofstream file(filename.c_str());
  if (file.is_open())
  {
    for (unsigned int i = 0; i < list.size(); i++)
    {
      for (unsigned int j = 0; j < list[i].size(); j++)
        file << list[i][j] << " ";
      file << endl;
    }
    file.close();
  }
  else
    throw std::runtime_error("Could not open file");
}

/*****************************************************************************/

Matrix Matrix::vector_from_string(string str)
{
  const char* str_c = str.c_str();
  scalar x;
  Matrix V(0, 1);
  int idx = 0;
  do
  {
    if (str_c[idx] == 0)
      break;
    while (is_space(str_c[idx]))
      idx++;
    if (str_c[idx] == 0)
      break;
    sscanf(&(str_c[idx]), "%lf", &x);
    V.push_back(x);
    V.lig++;
    while (!is_space(str_c[idx]) && str_c[idx] != 0)
      idx++;
    if (str_c[idx] == 0)
      break;
  } while (1);

  return V;
}

/*****************************************************************************/

void Matrix::size_pp(ostream& os)
{
  os << "[" << lig << "," << col << "]";
}

/*****************************************************************************/

void Matrix::square_transpose(Matrix& M, Matrix& result)
{
  for (int i = 0; i < M.lig; i++)
    for (int j = 0; j < M.lig; j++)
    {
      scalar sum = 0.0;
      for (int k = 0; k < M.lig; k++)
        sum += M.get(i, k) * M.get(k, j);
      if (j < i)
      {
        result.set(i, j, sum);
        result.set(j, i, sum);
      }
      else
        result.set(i, i, sum);
    }
}

/*****************************************************************************/

/* return the argument in [0, 2.Pi[ */
double argument(Matrix v)
{
  if ((v.lig != 2) || (v.col != 1))
    MathLog::fatal_error("argument(v) expect a 2x1 vector");
  return argument(v.get(0, 0), v.get(1, 0));
}

Matrix cartesian2polar(Matrix v)
{
  Matrix pol(2);
  pol[0] = v.norm2();
  pol[1] = argument(v);
  return pol;
}

Matrix polar2cartesian(Matrix v)
{
  Matrix M(2);
  M[0] = v[0] * cos(v[1]);
  M[0] = v[0] * sin(v[1]);
  return M;
}

double al_kashi(double a, double b, double c)
{
  if (b == 0.0 || c == 0.0)
    return 0.0;
  double x = (b * b + c * c - a * a) / (2 * b * c);
  if (x >= 1.0)
    return 0.0;
  if (x <= -1.0)
    return M_PI;
  return acos(x);
}

Matrix cartesian_to_spherical(Matrix M)
{
  double radius = M.norm2();
  Matrix M_p = M;
  M_p[2] = 0.0;
  double longitude = normalise_angle(argument(M_p[0], M_p[1]));
  double latitude = normalise_angle(argument(M_p.norm2(), M[2]));
  return Matrix::mk_vector(3, radius, longitude, latitude);
}

Matrix spherical_to_cartesian(Matrix M)
{
  Matrix pos(3);
  pos[2] = M[0] * sin(M[2]);
  double small_radius = M[0] * cos(M[2]);
  pos[0] = small_radius * cos(M[1]);
  pos[1] = small_radius * sin(M[1]);
  return pos;
}

double inverse_rotation(Matrix origin, Matrix target, Matrix axis)
{
  Matrix n_axis = axis.normalize();

  Matrix origin_p = (origin - Matrix::scalar_prod(origin, n_axis) * n_axis).normalize();
  Matrix target_p = (target - Matrix::scalar_prod(target, n_axis) * n_axis).normalize();

  double alpha = al_kashi((origin_p - target_p).norm2(), 1.0, 1.0);

  // determination du signe de alpha
  Matrix cp = Matrix::cross_product(n_axis, origin_p);
  double sp = Matrix::scalar_prod(target_p, cp);
  double sign = sign(sp);
  return sign * alpha;
}

/*****************************************************************************/
/*****************************************************************************/
