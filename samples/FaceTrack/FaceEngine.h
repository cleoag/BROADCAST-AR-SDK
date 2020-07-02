/*###############################################################################
#
# Copyright 2020 NVIDIA Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
###############################################################################*/
#ifndef __FACE_ENGINE__
#define __FACE_ENGINE__

#include <random>
#include "nvAR.h"
#include "nvCVOpenCV.h"
#include "opencv2/opencv.hpp"
#define FITFACE_PRIVATE

class KalmanFilter1D {
 private:
  float Q_;          // Covariance of the process noise
  float xhat_;       // Current prediction
  float xhatminus_;  // Previous prediction
  float P_;          // Estimated accuracy of xhat_
  float Pminus_;     // Previous P_
  float K_;          // Kalman gain
  float R_;          // Covariance of the observation noise
  bool bFirstUse;

 public:
  KalmanFilter1D() { reset(); }

  KalmanFilter1D(float Q, float R) { reset(Q, R); }

  void reset() {
    R_ = 0.005f * 0.005f;
    Q_ = 1e-5f;
    xhat_ = 0.0f;
    xhatminus_ = 0.0f;
    P_ = 1;
    bFirstUse = true;
    Pminus_ = 0.0f;
    K_ = 0.0f;
  }

  void reset(float Q, float R) {
    reset();
    Q_ = Q;
    R_ = R;
  }

  float update(float val) {
    if (bFirstUse) {
      xhat_ = val;
      bFirstUse = false;
    }

    xhatminus_ = xhat_;
    Pminus_ = P_ + Q_;
    K_ = Pminus_ / (Pminus_ + R_);
    xhat_ = xhatminus_ + K_ * (val - xhatminus_);
    P_ = (1 - K_) * Pminus_;

    return xhat_;
  }
};

const char* NvCV_StatusStringFromCode(NvCV_Status code);

bool CheckResult(NvCV_Status nvErr, unsigned line);

#define BAIL_IF_CVERR(nvErr, err, code)  \
  do {                                   \
    if (!CheckResult(nvErr, __LINE__)) { \
      err = code;                        \
      goto bail;                         \
    }                                    \
  } while (0)

/********************************************************************************
 * FaceEngine
 ********************************************************************************/

class FaceEngine {
 public:
  enum Err { errNone, errGeneral, errRun, errInitialization, errRead };
  int input_image_width, input_image_height, input_image_pitch;
  static const int NUM_LANDMARKS = 68;  // TODO: get this instead from the SDK.
  static const int FACE_MODEL_NUM_VERTICES = 3448, FACE_MODEL_NUM_INDICES = 6736;
  static const long LANDMARK_CONF_THRESH = 10.f;

  void setInputImageWidth(int width) { input_image_width = width; }
  void setInputImageHeight(int height) { input_image_height = height; }
  int getInputImageWidth() { return input_image_width; }
  int getInputImageHeight() { return input_image_height; }
  int getInputImagePitch() { return input_image_pitch = input_image_width * 3 * sizeof(unsigned char); }

  Err createFeatures(const char* modelPath, unsigned int _batchSize = 1);
  Err createFaceDetectionFeature(const char* modelPath, CUstream stream);
  Err createLandmarkDetectionFeature(const char* modelPath, unsigned int batchSize, CUstream stream);
  Err createFaceFittingFeature(const char* modelPath, CUstream stream);
  void destroyFeatures();
  void destroyFaceDetectionFeature();
  void destroyLandmarkDetectionFeature();
  void destroyFaceFittingFeature();
  Err initFeatureIOParams();
  Err initFaceDetectionIOParams(NvCVImage* _inputImageBuffer);
  Err initLandmarkDetectionIOParams(NvCVImage* _inputImageBuffer);
  Err initFaceFittingIOParams(NvCVImage* _inputImageBuffer);
  void releaseFeatureIOParams();
  void releaseFaceDetectionIOParams();
  void releaseLandmarkDetectionIOParams();
  void releaseFaceFittingIOParams();

  unsigned findFaceBoxes();
  NvAR_Rect* getLargestBox();
  NvCV_Status findLandmarks();
  NvAR_BBoxes* getBoundingBoxes();
  NvAR_Point2f* getLandmarks();
  NvAR_Quaternion* getPose();
  float* getLandmarksConfidence();
  float getAverageLandmarksConfidence();
  void enlargeAndSquarifyImageBox(float enlarge, NvAR_Rect& box, int FLAG_variant);
  static void jiggleBox(std::mt19937& ran, float minMag, float maxMag, const NvAR_Rect& cleanBox, NvAR_Rect& noisyBox);
  unsigned findLargestFaceBox(NvAR_Rect& faceBox, int variant = 0);
  unsigned acquireFaceBox(cv::Mat& src, NvAR_Rect& faceBox, int variant = 0); 
  unsigned acquireFaceBoxAndLandmarks(cv::Mat& src, NvAR_Point2f* refMarks, NvAR_Rect& faceBox, int variant = 0);
  Err fitFaceModel(cv::Mat& frame = cv::Mat());
  NvAR_FaceMesh* getFaceMesh();
  NvAR_RenderingParams* getRenderingParams();
  void setFaceStabilization(bool);
  void DrawPose(const cv::Mat& src, const NvAR_Quaternion* pose);

  NvCVImage inputImageBuffer{}, tmpImage{};
  NvAR_FeatureHandle faceDetectHandle{}, landmarkDetectHandle{}, faceFitHandle{};
  std::vector<NvAR_Point2f> facial_landmarks;
  std::vector<float> facial_landmarks_confidence;
  std::vector<NvAR_Quaternion> facial_pose;
  NvAR_FaceMesh* face_mesh{};
  NvAR_RenderingParams* rendering_params{};
  CUstream stream{};
  std::vector<NvAR_Rect> output_bbox_data;
  std::vector<float> output_bbox_conf_data;
  NvAR_BBoxes output_bboxes{};
  int batchSize;
  std::mt19937 ran;

  bool bStabilizeFace;
  NvAR_Point2f prevLandmark[NUM_LANDMARKS] = {0};

  FaceEngine() {
    batchSize = 1;
    bStabilizeFace = true;
    appMode = faceMeshGeneration;
    input_image_width = 1280;
    input_image_height = 720;
    input_image_pitch = 3 * input_image_width * sizeof(unsigned char);  // RGB
  }
  enum mode { faceDetection = 0, landmarkDetection, faceMeshGeneration } appMode;
  void setAppMode(FaceEngine::mode _mAppMode);
};
#endif
