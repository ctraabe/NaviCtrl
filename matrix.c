#include "matrix.h"


// =============================================================================
// Private data:

#define MAX_INVERSE_MATRIX_SIZE (9)


// =============================================================================
// Public functions:

float * MatrixAdd(const float * A, const float * B, size_t A_rows,
  size_t A_cols, float * result)
{
  float * result_ptr = result;
  for (size_t i = A_cols * A_rows; i; i--) *result_ptr++ = *A++ + *B++;

  return result;
}

// -----------------------------------------------------------------------------
float * MatrixAddIdentityToSelf(float * A, size_t size)
{
  for (size_t i = 0; i < size; i++) A[i * size + i] += 1.0;

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixAddDiagonalToSelf(float * A, const float * B, size_t size)
{
  for (size_t i = 0; i < size; i++) A[i * size + i] += B[i];

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixAddToSelf(float * A, const float * B, size_t A_rows,
  size_t A_cols)
{
  float * ptr = A;
  for (size_t i = A_cols * A_rows; i; i--) *ptr++ += *B++;

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixCopy(const float * A, size_t A_rows, size_t A_cols,
  float * result)
{
  float * result_ptr = result;
  for (size_t i = A_cols * A_rows; i; i--) *result_ptr++ = *A++;

  return result;
}

// -----------------------------------------------------------------------------
float * MatrixInverse(const float * A, size_t size, float * result)
{
  float A_copy[MAX_INVERSE_MATRIX_SIZE * MAX_INVERSE_MATRIX_SIZE];
  MatrixCopy(A, size, size, A_copy);

  // result starts as an identity matrix.
  SetMatrixToIdentity(result, size);

  // Gauss-Jordan.
  for (size_t k = 0; k < size; k++)
  {
    // Pivoting.
    float max = 0;
    size_t i_pivot = 0;
    for (size_t i = k; i < size; i++)
    {
      if (A_copy[i * size + k] * A_copy[i * size + k] > max * max)
      {
        max = A_copy[i * size + k];
        if (A_copy[i * size + k] < 0) max *= -1;
        i_pivot = i;
      }
    }
    if (i_pivot != k)
    {
      for (size_t j = 0; j < size; j++)
      {
        float temp = A_copy[k * size + j];
        A_copy[k * size + j] = A_copy[i_pivot * size + j];
        A_copy[i_pivot * size + j] = temp;

        temp = result[k * size + j];
        result[k * size + j] = result[i_pivot * size + j];
        result[i_pivot * size + j] = temp;
      }
    }
    // Make diagonal 1.
    float a = A_copy[k*size + k];
    for (size_t j = 0; j < size; j++){
      A_copy[k * size + j] /= a;
      result[k * size + j] /= a;
    }
    // Row subtraction.
    for (size_t i = 0; i < size; i++){
      if (i != k){
        float w = A_copy[i * size + k] / A_copy[k * size + k];
        for (size_t j = 0; j < size; j++){
          A_copy[i * size + j] -= w * A_copy[k * size + j];
          result[i * size + j] -= w * result[k * size + j];
        }
      }
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// This function multiplies matrix A by matrix B on the left.
float * MatrixMultiply(const float * A, const float * B, size_t A_rows,
  size_t A_cols, size_t B_cols, float *result)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < B_cols; j++)
    {
      result[i * B_cols + j] = 0;
      for (size_t k = 0; k < A_cols; k++)
      {
        result[i * B_cols + j] += A[A_cols * i + k] * B[j + B_cols * k];
      }
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// This function multiplies matrix A by a vector representing a diagonal-matrix
// B on the left.
float * MatrixMultiplyByDiagonal(const float * A, const float * B,
  size_t A_rows, size_t A_cols, float *result)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < A_cols; j++)
    {
      result[i * A_cols + j] = A[i * A_cols + j] * B[j];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// This function multiplies a vector representing a diagonal-matrix A by matrix
// B on the left.
float * MatrixMultiplyDiagonalByMatrix(const float * A, const float * B,
  size_t B_rows, size_t B_cols, float *result)
{
  for (size_t i = 0; i < B_rows; i++)
  {
    for (size_t j = 0; j < B_cols; j++)
    {
      result[i * B_cols + j] = A[i] * B[i * B_cols + j];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
// This function multiplies a vector representing a diagonal-matrix A by matrix
// B on the left.
float * MatrixMultiplyDiagonalBySelf(const float * A, float * B,
  size_t B_rows, size_t B_cols)
{
  for (size_t i = 0; i < B_rows; i++)
  {
    for (size_t j = 0; j < B_cols; j++)
    {
      B[i * B_cols + j] *= A[i];
    }
  }

  return B;
}

// -----------------------------------------------------------------------------
// This function multiplies matrix A by a vector representing a diagonal-matrix
// B on the left.
float * MatrixMultiplySelfByDiagonal(float * A, const float * B,
  size_t A_rows, size_t A_cols)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < A_cols; j++)
    {
      A[i * A_cols + j] *= B[j];
    }
  }

  return A;
}

// -----------------------------------------------------------------------------
// This function reverses the sign of each element of matrix A.
float * MatrixNegateSelf(float * A, size_t A_rows, size_t A_cols)
{
  float * ptr = A;
  for (size_t i = A_cols * A_rows; i; i--)
  {
    *ptr = -*ptr;
    ptr++;
  }

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixScale(const float * A, float scale, size_t A_rows, size_t A_cols,
  float * result)
{
  float * result_ptr = result;
  for (size_t i = A_cols * A_rows; i; i--) *result_ptr++ = *A++ * scale;

  return result;
}

// -----------------------------------------------------------------------------
float * MatrixScaleSelf(float * A, float scale, size_t A_rows, size_t A_cols)
{
  float * ptr = A;
  for (size_t i = A_cols * A_rows; i; i--) *ptr++ *= scale;

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixSubtract(const float * A, const float * B, size_t A_rows,
  size_t A_cols, float * result)
{
  float * result_ptr = result;
  for (size_t i = A_cols * A_rows; i; i--) *result_ptr++ = *A++ - *B++;

  return result;
}

// -----------------------------------------------------------------------------
float * MatrixSubtractIdentityFromSelf(float * A, size_t size)
{
  for (size_t i = 0; i < size; i++) A[i * size + i] -= 1.0;

  return A;
}

// -----------------------------------------------------------------------------
float * MatrixSubtractSelfFromIdentity(float * A, size_t size)
{
  return MatrixAddIdentityToSelf(MatrixNegateSelf(A, size, size), size);
}

// -----------------------------------------------------------------------------
float * MatrixTranspose(const float * A, size_t A_rows, size_t A_cols,
  float * result)
{
  for (size_t i = 0; i < A_rows; i++)
  {
    for (size_t j = 0; j < A_cols; j++)
    {
      result[j * A_rows + i] = A[i * A_cols + j];
    }
  }

  return result;
}

// -----------------------------------------------------------------------------
float * SetMatrixToIdentity(float * A, size_t size)
{
  float * ptr = A;
  for (size_t i = size * size; i; i--) *ptr++ = 0.0;
  for (size_t i = 0; i < size; i++) A[i * size + i] = 1.0;

  return A;
}

// -----------------------------------------------------------------------------
float * VectorToDiagonal(const float * v, size_t length, float * result)
{
  float * result_ptr = result;
  for (size_t i = length * length; i; i--) *result_ptr++ = 0.0;
  for (size_t i = 0; i < length; i++) result[i * length + i] = v[i];

  return result;
}