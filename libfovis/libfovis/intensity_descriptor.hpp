#ifndef __fovis_gray_descriptor_hpp__
#define __fovis_gray_descriptor_hpp__

#include <inttypes.h>
#include <stdlib.h>

namespace fovis
{

class KeypointData;

/**
 * \ingroup FovisCore
 * \brief Extracts mean-normalized intensity patch around a pixel.
 *
 */
class IntensityDescriptorExtractor {
 public:
  IntensityDescriptorExtractor(int raw_gray_stride, int feature_window_size)
  : _raw_gray_stride(raw_gray_stride),
    _feature_window_size(feature_window_size) {
    initialize();
  }

  virtual ~IntensityDescriptorExtractor () {
    delete[] _descriptor_index_offsets;
    free(_descriptor_brightness_offset);
  }

  void populateDescriptorInterp(uint8_t *image,
                                float x, float y,
                                uint8_t* descriptor) const;

  void populateDescriptorAligned(uint8_t *image,
                                 int x, int y,
                                 uint8_t* descriptor) const;

  void populateDescriptorsInterp(uint8_t* image,
                                 const KeypointData* keypoints,
                                 int num_keypoints,
                                 uint8_t* descriptors) const;

  void populateDescriptorsAligned(uint8_t* image,
                                  const KeypointData* keypoints,
                                  int num_keypoints,
                                  uint8_t* descriptors) const;

  int getDescriptorStride() const {
    return _descriptor_stride;
  }

  const int* getDescriptorIndexOffsets() const {
    return _descriptor_index_offsets;
  }

  int getDescriptorLength() const {
    return _descriptor_len;
  }

 private:
  IntensityDescriptorExtractor(const IntensityDescriptorExtractor& other);
  IntensityDescriptorExtractor& operator=(const IntensityDescriptorExtractor& other);

  void initialize();
  void normalizeDescriptor(uint8_t* desc) const;

  int _raw_gray_stride;

  int _num_descriptor_pad_bytes;
  uint8_t* _descriptor_brightness_offset;
  int _brightess_offset_num_sse_ops;

  int _descriptor_len;
  int _feature_window_size;
  int _descriptor_stride;
  int* _descriptor_index_offsets;

};

} /*  */
#endif
