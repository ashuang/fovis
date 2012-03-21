#ifndef __fovis_feature_matcher_hpp__
#define __fovis_feature_matcher_hpp__

#include "feature_match.hpp"

namespace fovis
{

class FeatureMatcher {
public:
  FeatureMatcher ();
  virtual ~FeatureMatcher ();

#if 0
  // TODO
  void matchFeatures(PyramidLevel* ref_level,
                     PyramidLevel* target_level,
                     const std::binary_function<int, int, bool>& is_legal,
                     std::vector<FeatureMatch>* matches);
#endif

  void matchFeatures(PyramidLevel* ref_level,
                     PyramidLevel* target_level,
                     const std::vector<std::vector<int> >& candidates,
                     FeatureMatch* matches,
                     int* num_matches);

private:
  FeatureMatcher (const FeatureMatcher& other);
  FeatureMatcher& operator=(const FeatureMatcher& other);

  // how many features can be referenced in the temporary workspace buffers
  int _ref_feature_capacity;
  int _target_feature_capacity;

  // temporary workspace buffers for feature matching
  int32_t* _ref_to_target_indices;
  int32_t* _ref_to_target_scores;
  int32_t* _target_to_ref_indices;
  int32_t* _target_to_ref_scores;
};



} /*  */

#endif /* end of include guard: __fovis_feature_matcher_hpp__ */
