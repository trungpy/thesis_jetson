#pragma once

#include <FrontDistanceEstimator.h>
#include <config.h>

#include <opencv2/opencv.hpp>

#include "kalmanFilter.h"
using namespace cv;
using namespace std;

enum TrackState { New = 0, Tracked, Lost, Removed };

class STrack {
public:
    STrack(vector<float> tlwh_, float score,
           int classId); // Updated constructor
    ~STrack();

    vector<float> static tlbr_to_tlwh(vector<float> &tlbr);
    void static multi_predict(vector<STrack *> &stracks,
                              byte_kalman::KalmanFilter &kalman_filter);
    void static_tlwh();
    void static_tlbr();
    vector<float> tlwh_to_xyah(vector<float> tlwh_tmp);
    vector<float> to_xyah();
    void mark_lost();
    void mark_removed();
    int next_id();
    int end_frame();

    void activate(byte_kalman::KalmanFilter &kalman_filter, int frame_id);
    void re_activate(STrack &new_track, int frame_id, bool new_id = false);
    void update(STrack &new_track, int frame_id);
    inline static FrontDistanceEstimator estimator{0, 0};
    static void initializeEstimator() {
        estimator =
            FrontDistanceEstimator(Config::config.camera.focalLength,
                                   Config::config.camera.realObjectWidth);
    }

public:
    bool is_activated;
    int track_id;
    int state;

    vector<float> _tlwh;
    vector<float> tlwh;
    vector<float> tlbr;
    int frame_id;
    int tracklet_len;
    int start_frame;

    KAL_MEAN mean;
    KAL_COVA covariance;
    float score;
    int classId; // Added classId
    float estimatedDistance = -1.0f;

private:
    byte_kalman::KalmanFilter kalman_filter;
};