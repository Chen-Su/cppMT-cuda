#ifndef CMT_H

#define CMT_H

#include "common.h"
#include "Consensus.h"
#include "Fusion.h"
#include "Matcher.h"
#include "Tracker.h"

#include <opencv2/features2d/features2d.hpp>

#ifdef USE_CUDA

#include "opencv2/cudafeatures2d.hpp"

#endif


using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Ptr;
using cv::RotatedRect;
using cv::Size2f;

namespace cmt
{

class CMT
{
public:
    CMT(){is_detected = true; theta = 0.15;};
    void initialize(const Mat im_gray, const Rect rect);
    void processFrame(const Mat im_gray);

    Fusion fusion;
    Matcher matcher;
    Tracker tracker;
    Consensus consensus;

    vector<Point2f> points_active; //public for visualization purposes
    RotatedRect bb_rot;

    bool is_detected;

private:


	Ptr<cv::ORB> orb_detector;

    Size2f size_initial;

    vector<int> classes_active;

    float theta;
    int O;

    Mat im_prev;
};

} /* namespace CMT */

#endif /* end of include guard: CMT_H */
