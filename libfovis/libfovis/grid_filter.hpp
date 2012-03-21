#ifndef __fovis_grid_filter_hpp__
#define __fovis_grid_filter_hpp__

#include <math.h>
#include <vector>
#include "keypoint.hpp"

namespace fovis
{

class GridKeyPointFilter {
public:
  GridKeyPointFilter (int img_width, int img_height, int bucket_width, 
                      int bucket_height, int max_keypoints_per_bucket) :
      _img_width(img_width), _img_height(img_height),
      _bucket_width(bucket_width), _bucket_height(bucket_height),
      _max_keypoints_per_bucket(max_keypoints_per_bucket),
      _grid_rows(ceil((float)img_height/bucket_height)), 
      _grid_cols(ceil((float)img_width/bucket_width)),
      _buckets(_grid_rows * _grid_cols)
  { 
  }

  virtual ~GridKeyPointFilter () { }

  void filter(std::vector<KeyPoint>* keypoints);

private:
  int _img_width;
  int _img_height;
  int _bucket_width;
  int _bucket_height;
  int _max_keypoints_per_bucket; 
  int _grid_rows;
  int _grid_cols;
  std::vector<std::vector<KeyPoint> > _buckets;

};

}

#endif
