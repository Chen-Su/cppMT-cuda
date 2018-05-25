#include "CMT.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <algorithm>

using std::max;
using std::min;

namespace cmt {

void CMT::initialize(const Mat im_gray, const Rect rect)
{
    FILE_LOG(logDEBUG) << "CMT::initialize() call";

    //Remember initial size
    size_initial = rect.size();

    //Remember initial image
    im_prev = im_gray;

    //Compute center of rect
    Point2f center = Point2f(rect.x + rect.width/2.0, rect.y + rect.height/2.0);

    //Initialize rotated bounding box
    bb_rot = RotatedRect(center, size_initial, 0.0);

    //Initialize detector and descriptor

	orb_detector = cv::ORB::create(5000, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE);
    //detector = cv::FastFeatureDetector::create();
    //descriptor = cv::BRISK::create();


    //Get initial keypoints in whole image and compute their descriptors
    vector<KeyPoint> keypoints;
	orb_detector->detect(im_gray, keypoints);

    //Divide keypoints into foreground and background keypoints according to selection
    vector<KeyPoint> keypoints_fg;
    vector<KeyPoint> keypoints_bg;

    for (size_t i = 0; i < keypoints.size(); i++)
    {
        KeyPoint k = keypoints[i];
        Point2f pt = k.pt;

        if (pt.x > rect.x && pt.y > rect.y && pt.x < rect.br().x && pt.y < rect.br().y)
        {
            keypoints_fg.push_back(k);
        }

        else
        {
            keypoints_bg.push_back(k);
        }

    }

    O = keypoints_fg.size();

    //Create foreground classes
    vector<int> classes_fg;
    classes_fg.reserve(keypoints_fg.size());
    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        classes_fg.push_back(i);
    }

    //Compute foreground/background features
    Mat descs_fg;
    Mat descs_bg;
	orb_detector->compute(im_gray, keypoints_fg, descs_fg);
	orb_detector->compute(im_gray, keypoints_bg, descs_bg);

    //Only now is the right time to convert keypoints to points, as compute() might remove some keypoints
    vector<Point2f> points_fg;
    vector<Point2f> points_bg;

    for (size_t i = 0; i < keypoints_fg.size(); i++)
    {
        points_fg.push_back(keypoints_fg[i].pt);
    }

    FILE_LOG(logDEBUG) << points_fg.size() << " foreground points.";

    for (size_t i = 0; i < keypoints_bg.size(); i++)
    {
        points_bg.push_back(keypoints_bg[i].pt);
    }

    //Create normalized points
    vector<Point2f> points_normalized;
    for (size_t i = 0; i < points_fg.size(); i++)
    {
        points_normalized.push_back(points_fg[i] - center);
    }

    //Initialize matcher
    matcher.initialize(points_normalized, descs_fg, classes_fg, descs_bg, center);

    //Initialize consensus
    consensus.initialize(points_normalized);

    //Create initial set of active keypoints
	points_active = points_fg;
	classes_active = classes_fg;


    FILE_LOG(logDEBUG) << "CMT::initialize() return";
}

void CMT::processFrame(Mat im_gray) {

    FILE_LOG(logDEBUG) << "CMT::processFrame() call";

    //Track keypoints
    vector<Point2f> points_tracked;
    vector<unsigned char> status;
    tracker.track(im_prev, im_gray, points_active, points_tracked, status);

    FILE_LOG(logDEBUG) << points_tracked.size() << " tracked points.";

    //keep only successful classes
    vector<int> classes_tracked;
    for (size_t i = 0; i < classes_active.size(); i++)
    {
        if (status[i])
        {
            classes_tracked.push_back(classes_active[i]);
        }

    }

    //Detect keypoints, compute descriptors
    vector<KeyPoint> keypoints;
    Rect bb;
        bb.x = max(static_cast<int>(bb_rot.center.x - 1.5 * bb_rot.size.width), 0);
        bb.y = max(static_cast<int>(bb_rot.center.y - 1.5 * bb_rot.size.height), 0);
        bb.width = min(static_cast<int>(bb_rot.size.width * 2), im_gray.cols - bb.x);
        bb.height = min(static_cast<int>(bb_rot.size.height * 2), im_gray.rows - bb.y);
        Mat mask = Mat::zeros(im_gray.size(), CV_8U);
        Mat roi(mask, bb);
        roi = cv::Scalar(255, 255, 255);

        if (is_detected)
            orb_detector->detect(im_gray, keypoints, mask), printf("partical detect");
        else
            orb_detector->detect(im_gray, keypoints);

    FILE_LOG(logDEBUG) << keypoints.size() << " keypoints found.";

    Mat descriptors;
	orb_detector->compute(im_gray, keypoints, descriptors);

    //Match keypoints globally
    vector<Point2f> points_matched_global;
    vector<int> classes_matched_global;
    matcher.matchGlobal(keypoints, descriptors, points_matched_global, classes_matched_global);

    FILE_LOG(logDEBUG) << points_matched_global.size() << " points matched globally.";

    //Fuse tracked and globally matched points
    vector<Point2f> points_fused;
    vector<int> classes_fused;
    fusion.preferFirst(points_tracked, classes_tracked, points_matched_global, classes_matched_global,
            points_fused, classes_fused);

    FILE_LOG(logDEBUG) << points_fused.size() << " points fused.";

    //Estimate scale and rotation from the fused points
    float scale;
    float rotation;
    consensus.estimateScaleRotation(points_fused, classes_fused, scale, rotation);

    FILE_LOG(logDEBUG) << "scale " << scale << ", " << "rotation " << rotation;

    //Find inliers and the center of their votes
    Point2f center;
    vector<Point2f> points_inlier;
    vector<int> classes_inlier;
    consensus.findConsensus(points_fused, classes_fused, scale, rotation,
            center, points_inlier, classes_inlier);

    FILE_LOG(logDEBUG) << points_inlier.size() << " inlier points.";
    FILE_LOG(logDEBUG) << "center " << center;

    if(points_inlier.size() >= theta * O)
        is_detected = true;
    else
        is_detected = false;

    //Match keypoints locally
    /*vector<Point2f> points_matched_local;
    vector<int> classes_matched_local;
    matcher.matchLocal(keypoints, descriptors, center, scale, rotation, points_matched_local, classes_matched_local);

    FILE_LOG(logDEBUG) << points_matched_local.size() << " points matched locally.";*/

    //Clear active points
    points_active.clear();
    classes_active.clear();

    //Fuse locally matched points and inliers
    //fusion.preferFirst(points_matched_local, classes_matched_local, points_inlier, classes_inlier, points_active, classes_active);
    points_active = points_inlier;
    classes_active = classes_inlier;

    FILE_LOG(logDEBUG) << points_active.size() << " final fused points.";

    //TODO: Use theta to suppress result
    bb_rot = RotatedRect(center,  size_initial * scale, rotation/CV_PI * 180);

    //Remember current image
    im_prev = im_gray;

    FILE_LOG(logDEBUG) << "CMT::processFrame() return";
}

} /* namespace CMT */
