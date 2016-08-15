#include "mag_calibration.h"

#include "lsm303dl.h"


// =============================================================================
// Private data:

#define N_CALIBRATION_SAMPLES (100)  // 12 bytes per sample

static struct MagSample {
  int16_t sample[3];
  uint16_t nearest_neighbor_index;  // 2 bytes for memory alignment (important)
  uint32_t nearest_neighbor_distace;
} __attribute__((packed)) mag_samples_[N_CALIBRATION_SAMPLES];


// =============================================================================
// Private function declarations:
static uint32_t NormSquared(const int16_t * v1, const int16_t * v2);


// =============================================================================
// Accessors:


// =============================================================================
// Public functions:

void MagCalibrationInit(void)
{
  // TODO: consider allocating and freeing memory for calibration samples.
}

// -----------------------------------------------------------------------------
void MagCalibrationAddSample(void)
{
  static size_t worst_sample_index_ = 0;

  const int16_t * const new_sample = MagnetometerVector();
  size_t nearest_neighbor_index = 0;
  uint32_t new_sample_distances_[N_CALIBRATION_SAMPLES];
  uint32_t nearest_neighbor_distace = 0xFFFFFFFF;  // Max value

  // Search for the nearest neighbor to the new sample.
  for (size_t i = 0; i < N_CALIBRATION_SAMPLES; i++)
  {
    // Skip the worst sample because the new sample will replace it if better.
    if (i == worst_sample_index_) continue;

    // Search for the nearest neighbor to the new sample. Also save the
    // distances in the process, to be used if the new sample is better than the
    // worst sample.
    new_sample_distances_[i] = NormSquared(new_sample, mag_samples_[i].sample);
    if (new_sample_distances_[i] < nearest_neighbor_distace)
      nearest_neighbor_index = i;
  }

  // Check if new sample is better than the existing worst sample.
  if (new_sample_distances_[nearest_neighbor_index]
    > mag_samples_[worst_sample_index_].nearest_neighbor_distace)
  {
    // Replace the worst sample with the new sample.
    mag_samples_[worst_sample_index_].sample[0] = new_sample[0];
    mag_samples_[worst_sample_index_].sample[1] = new_sample[1];
    mag_samples_[worst_sample_index_].sample[2] = new_sample[2];
    mag_samples_[worst_sample_index_].nearest_neighbor_index
      = nearest_neighbor_index;
    mag_samples_[worst_sample_index_].nearest_neighbor_distace
      = new_sample_distances_[nearest_neighbor_index];

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
        mag_samples_[i].nearest_neighbor_distace = new_sample_distances_[i];

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
      else if (new_sample_distances_[i]
        < mag_samples_[i].nearest_neighbor_distace)
      {
        mag_samples_[i].nearest_neighbor_index = new_sample_index;
        mag_samples_[i].nearest_neighbor_distace = new_sample_distances_[i];
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
void MagCalibratinCopmute(void)
{
}


// =============================================================================
// Private functions:

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
