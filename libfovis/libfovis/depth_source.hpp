#ifndef __fovis_depth_source_hpp__
#define __fovis_depth_source_hpp__

#include <Eigen/Core>

namespace fovis
{

class OdometryFrame;
class FeatureMatch;

/**
 * \ingroup FovisCore
 * \brief Abstract class representing sources of depth information.
 *
 * TODO
 */
class DepthSource
{
  public:
    /**
     * This should return true if it's not certain there's no depth at (u,v).
     * It should be an inexpensive check that is used to avoid pointless (hah!)
     * creation of keypoints. False positives are fine as ling as getXyz gets
     * rid of them.
     */
    virtual bool haveXyz(int u, int v) = 0;

    /**
     * Populate keypoints in frame with XYZ data.
     */
    virtual void getXyz(OdometryFrame * frame) = 0;

    /**
     * Refine XYZ data of target keypoints in matches (usually after
     * subpixel refinement of the matches across time).
     */
    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame) = 0;

    /**
     * Return baseline of depth source, if applicable. If not applicable (e.g. for
     * OpenNI devices) return 0.
     */
    virtual double getBaseline() const = 0;

};

}

#endif
