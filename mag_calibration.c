#include "mag_calibration.h"

#include <math.h>
#include <stddef.h>

#include "custom_math.h"
#include "eeprom.h"
#include "lsm303dl.h"
#include "matrix.h"
#include "timing.h"
#include "uart.h"


// =============================================================================
// Private data:

#define N_CALIBRATION_SAMPLES (100)  // 12 bytes per sample
#define OPTIMAL_NEAREST_NEIGHBOR (0.35)  // Optimal distribution for 100 points

static struct MagSample {
  int16_t sample[3];
  uint16_t nearest_neighbor_index;  // 2 bytes for memory alignment (important)
  uint32_t nearest_neighbor_distace;
} __attribute__((packed)) mag_samples_[N_CALIBRATION_SAMPLES];
static size_t worst_sample_index_;


// =============================================================================
// Private function declarations:
static void MagCalibrationAddSample(const int16_t * const new_sample);
static void MagCalibrationCompute(float unitizer[3], int16_t bias[3]);
static uint32_t NormSquared(const int16_t * v1, const int16_t * v2);
static inline float Square(float a);


// =============================================================================
// Public functions:

void MagCalibrationInit(const int16_t * const new_sample)
{
  // TODO: consider allocating and freeing memory for calibration samples.
  for (size_t i = 0; i < N_CALIBRATION_SAMPLES; i++)
  {
    mag_samples_[i].sample[0] = new_sample[0];
    mag_samples_[i].sample[1] = new_sample[1];
    mag_samples_[i].sample[2] = new_sample[2];
    mag_samples_[i].nearest_neighbor_index = i + 1;
    mag_samples_[i].nearest_neighbor_distace = 0;
  }
  mag_samples_[N_CALIBRATION_SAMPLES-1].nearest_neighbor_index = 0;
  worst_sample_index_ = 0;
}

//------------------------------------------------------------------------------
uint32_t MagCalibration(uint32_t mag_calibration)
{
  static uint32_t mag_calibration_latch = 0, mag_calibration_timer = 0;

  if (mag_calibration)
  {
    if (mag_calibration_latch)
    {
      MagCalibrationAddSample(MagnetometerVector());
    }
    else
    {
      UARTPrintf("Calibration start");
      MagCalibrationInit(MagnetometerVector());
      mag_calibration_timer = GetTimestampMillisFromNow(20);
      mag_calibration_latch = 1;
    }

    // Request the next sample.
    RequestLSM303DL();

    // Take a sample every 20 ms.
    while (!TimestampInPast(mag_calibration_timer)) continue;
    mag_calibration_timer += 20;
  }
  else if (mag_calibration_latch)
  {
    int16_t bias[3];
    float unitizer[3];
    MagCalibrationCompute(unitizer, bias);
    WriteMagnetometerUnitizerToEEPROM(unitizer);

    WriteMagnetometerBiasToEEPROM(bias);
    WriteMagnetometerCalibratedToEEPROM(1);

    UARTPrintf("unitizer: %f, %f, %f", unitizer[0], unitizer[1], unitizer[2]);
    UARTPrintf("bias: %i, %i, %i", bias[0], bias[1], bias[2]);

    mag_calibration_latch = 0;
  }

  return mag_calibration;
}


// =============================================================================
// Private functions:

static void MagCalibrationAddSample(const int16_t * const new_sample)
{
  size_t nearest_neighbor_index = 0;
  uint32_t new_sample_distances[N_CALIBRATION_SAMPLES];
  uint32_t nearest_neighbor_distace = 0xFFFFFFFF;  // Max value

  // Search for the nearest neighbor to the new sample.
  for (size_t i = 0; i < N_CALIBRATION_SAMPLES; i++)
  {
    // Skip the worst sample because the new sample will replace it if better.
    if (i == worst_sample_index_) continue;

    // Search for the nearest neighbor to the new sample. Also save the
    // distances in the process, to be used if the new sample is better than the
    // worst sample.
    new_sample_distances[i] = NormSquared(new_sample, mag_samples_[i].sample);
    if (new_sample_distances[i] < nearest_neighbor_distace)
    {
      nearest_neighbor_index = i;
      nearest_neighbor_distace = new_sample_distances[i];
    }
  }

  // Check if new sample is better than the existing worst sample.
  if (nearest_neighbor_distace
    > mag_samples_[worst_sample_index_].nearest_neighbor_distace)
  {
    // Replace the worst sample with the new sample.
    mag_samples_[worst_sample_index_].sample[0] = new_sample[0];
    mag_samples_[worst_sample_index_].sample[1] = new_sample[1];
    mag_samples_[worst_sample_index_].sample[2] = new_sample[2];
    mag_samples_[worst_sample_index_].nearest_neighbor_index
      = nearest_neighbor_index;
    mag_samples_[worst_sample_index_].nearest_neighbor_distace
      = new_sample_distances[nearest_neighbor_index];

    // Review the nearest neighbors for the rest of the existing samples.
    size_t new_sample_index = worst_sample_index_;
    for (size_t i = 0; i < N_CALIBRATION_SAMPLES; i++)
    {
      // Skip the new sample.
      if (i == new_sample_index) continue;

      // Check if the nearest neighbor of an existing sample was the sample that
      // the new sample just replaced.
      if (mag_samples_[i].nearest_neighbor_index == new_sample_index)
      {
        // First assume that the new sample is the nearest neighbor.
        mag_samples_[i].nearest_neighbor_distace = new_sample_distances[i];

        // Search the other samples for a closer neighbor.
        for (size_t j = 0; j < N_CALIBRATION_SAMPLES; j++)
        {
          // Skip self and new sample.
          if ((j == i) || (j == new_sample_index)) continue;

          uint32_t distance = NormSquared(mag_samples_[j].sample,
            mag_samples_[i].sample);
          if (distance < mag_samples_[i].nearest_neighbor_distace)
          {
            mag_samples_[i].nearest_neighbor_index = j;
            mag_samples_[i].nearest_neighbor_distace = distance;
          }
        }
      }
      // Check if the new sample is nearer than the current nearest neighbor.
      else if (new_sample_distances[i]
        < mag_samples_[i].nearest_neighbor_distace)
      {
        mag_samples_[i].nearest_neighbor_index = new_sample_index;
        mag_samples_[i].nearest_neighbor_distace = new_sample_distances[i];
      }

      // Check for the worst existing sample.
      if (mag_samples_[i].nearest_neighbor_distace
        < mag_samples_[worst_sample_index_].nearest_neighbor_distace)
      {
        worst_sample_index_ = i;
      }
    }
  }
}

// -----------------------------------------------------------------------------
static void MagCalibrationCompute(float unitizer[3], int16_t bias[3])
{
  float num[6*1] = { 0.0 }, den[6*6];
  {
    float D_t[6*N_CALIBRATION_SAMPLES];
    for (size_t i = 0; i < N_CALIBRATION_SAMPLES; i++)
    {
      D_t[3*N_CALIBRATION_SAMPLES+i] = (float)mag_samples_[i].sample[0];
      D_t[4*N_CALIBRATION_SAMPLES+i] = (float)mag_samples_[i].sample[1];
      D_t[5*N_CALIBRATION_SAMPLES+i] = (float)mag_samples_[i].sample[2];
      D_t[0*N_CALIBRATION_SAMPLES+i] = Square(D_t[3*N_CALIBRATION_SAMPLES+i]);
      D_t[1*N_CALIBRATION_SAMPLES+i] = Square(D_t[4*N_CALIBRATION_SAMPLES+i]);
      D_t[2*N_CALIBRATION_SAMPLES+i] = Square(D_t[5*N_CALIBRATION_SAMPLES+i]);
      num[0] += D_t[0*N_CALIBRATION_SAMPLES+i];
      num[1] += D_t[1*N_CALIBRATION_SAMPLES+i];
      num[2] += D_t[2*N_CALIBRATION_SAMPLES+i];
      num[3] += D_t[3*N_CALIBRATION_SAMPLES+i];
      num[4] += D_t[4*N_CALIBRATION_SAMPLES+i];
      num[5] += D_t[5*N_CALIBRATION_SAMPLES+i];
    }
    MatrixMultiplyByTranspose(D_t, D_t, 6, N_CALIBRATION_SAMPLES, 6, den);
  }

  float den_inv[6*6], u[6*1];
  MatrixInverse(den, 6, den_inv);
  MatrixMultiply(den_inv, num, 6, 6, 1, u);

  float temp = 1.0 / (1.0 + 0.25 * (u[3] * u[3] / u[0] + u[4] * u[4] / u[1]
    + u[5] * u[5] / u[2]));

  unitizer[0] = sqrt(u[0] * temp);
  unitizer[1] = sqrt(u[1] * temp);
  unitizer[2] = sqrt(u[2] * temp);
  bias[0] = (int16_t)round(-0.5 * u[3] / u[0]);
  bias[1] = (int16_t)round(-0.5 * u[4] / u[1]);
  bias[2] = (int16_t)round(-0.5 * u[5] / u[2]);
}

// -----------------------------------------------------------------------------
static uint32_t NormSquared(const int16_t * v1, const int16_t * v2)
{
  uint32_t result = 0;
  for (size_t i = 0; i < 3; i++)
  {
    int16_t temp = v1[i] - v2[i];
    result += temp * temp;
  }
  return result;
}

// -----------------------------------------------------------------------------
static inline float Square(float a)
{
  return a * a;
}
