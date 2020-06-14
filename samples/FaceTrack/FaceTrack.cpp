#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FaceEngine.h"
#include "nvAR.h"
#include "nvAR_defs.h"
#include "opencv2/opencv.hpp"
#include "RenderingUtils.h"

#include "PlatformBase.h"
#include "RenderAPI/RenderAPI.h"

#ifndef M_PI
#define M_PI 3.1415926535897932385
#endif /* M_PI */
#ifndef M_2PI
#define M_2PI 6.2831853071795864769
#endif /* M_2PI */
#ifndef M_PI_2
#define M_PI_2 1.5707963267948966192
#endif /* M_PI_2 */
#define F_PI ((float)M_PI)
#define F_PI_2 ((float)M_PI_2)
#define F_2PI ((float)M_2PI)

#ifdef _MSC_VER
#define strcasecmp _stricmp
#endif /* _MSC_VER */

#define BAIL(err, code) \
  do {                  \
    err = code;         \
    goto bail;          \
  } while (0)

/********************************************************************************
 * Command-line arguments
 ********************************************************************************/

bool FLAG_debug = false, FLAG_verbose = false, FLAG_temporal = true;
std::string FLAG_modelPath = "models";
std::string FLAG_landmarks;
unsigned int FLAG_batch = 1;


/********************************************************************************
 * StringToFourcc
 ********************************************************************************/

static int StringToFourcc(const std::string &str) {
  union chint {
    int i;
    char c[4];
  };
  chint x = {0};
  for (int n = (str.size() < 4) ? (int)str.size() : 4; n--;) x.c[n] = str[n];
  return x.i;
}


enum {
  myErrNone = 0,
  myErrShader = -1,
  myErrProgram = -2,
  myErrTexture = -3,
};

#if 1
class MyTimer {
 public:
  void start() { t0 = std::chrono::high_resolution_clock::now(); }       /**< Start  the timer. */
  void pause() { dt = std::chrono::high_resolution_clock::now() - t0; }  /**< Pause  the timer. */
  void resume() { t0 = std::chrono::high_resolution_clock::now() - dt; } /**< Resume the timer. */
  void stop() { pause(); }                                               /**< Stop   the timer. */
  double elapsedTimeFloat() const {
    return std::chrono::duration<double>(dt).count();
  } /**< Report the elapsed time as a float. */
 private:
  std::chrono::high_resolution_clock::time_point t0;
  std::chrono::high_resolution_clock::duration dt;
};
#endif

std::string getCalendarTime() {
  // Get the current time
  std::chrono::system_clock::time_point currentTimePoint = std::chrono::system_clock::now();
  // Convert to time_t from time_point
  std::time_t currentTime = std::chrono::system_clock::to_time_t(currentTimePoint);
  // Convert to tm to get structure holding a calendar date and time broken down into its components.
  std::tm brokenTime = *std::localtime(&currentTime);
  std::ostringstream calendarTime;
  calendarTime << std::put_time(
      &brokenTime,
      "%Y-%m-%d-%H-%M-%S");  // (YYYY-MM-DD-HH-mm-ss)<Year>-<Month>-<Date>-<Hour>-<Mins>-<Seconds>
  // Get the time since epoch 0(Thu Jan  1 00:00:00 1970) and the remainder after division is
  // our milliseconds
  std::chrono::milliseconds currentMilliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(currentTimePoint.time_since_epoch()) % 1000;
  // Append the milliseconds to the stream
  calendarTime << "-" << std::setfill('0') << std::setw(3) << currentMilliseconds.count();  // milliseconds
  return calendarTime.str();
}

class DoApp {
 public:
  enum Err {
    errNone,
    errUnimplemented,
    errMissing,
    errVideo,
    errImageSize,
    errNotFound,
    errFaceModelInit,
    errGLFWInit,
    errGLInit,
    errRendererInit,
    errGLResource,
    errGLGeneric,
    errFaceFit,
    errNoFace,
    errSDK,
    errCuda,
    errCancel,
    errInitFaceEngine,
    errOpenCVInit
  };

  FaceEngine face_ar_engine;
  DoApp();
  ~DoApp();

  void stop();
  Err initFaceEngine(const char *modelPath = nullptr);
  Err initCamera(int cameraIndex, int inCameraWidth,int inCameraHeight);
  Err acquireFrame();
  Err acquireFaceBox();
  Err acquireFaceBoxAndLandmarks();
  Err fitFaceModel();
  Err update();
  void setMode(int key);
  void getFPS();
  static const char *errorStringFromCode(Err code);

  cv::VideoCapture cap{};
  cv::Mat frame;
  int inputWidth, inputHeight;
  int camIndex;
  cv::VideoWriter faceDetectOutputVideo{}, landMarkOutputVideo{}, faceFittingOutputVideo{};
  int frameIndex;
  //static const char windowTitle[];
  double frameTime;
  // std::chrono::high_resolution_clock::time_point frameTimer;
  MyTimer frameTimer;
  cv::VideoWriter capturedVideo;
  std::ofstream faceEngineVideoOutputFile;
  bool _continue;

  FaceEngine::Err nvErr;
  float expr[6];
  bool drawVisualization, showFPS, captureVideo, captureFrame;
  std::vector<std::array<uint16_t, 3>> proxyWireframe;
  float scaleOffsetXY[4];
  std::vector<NvAR_Vector3u16> wfMesh_tvi_data;

  NvAR_Rect unity_output_bbox;
  int unity_num_boxes;

  NvAR_Point2f facial_landmarks[FaceEngine::NUM_LANDMARKS];
};

DoApp *gApp = nullptr;
//const char DoApp::windowTitle[] = "WINDOW";

void DoApp::setMode(int mode) {
  switch (mode) {
    case 3:
      face_ar_engine.destroyFeatures();
      face_ar_engine.setAppMode(FaceEngine::mode::faceMeshGeneration);
      nvErr =  face_ar_engine.createFeatures(FLAG_modelPath.c_str());
      // If there is an error, fallback to mode '2' i.e. landmark detection
      if (nvErr == FaceEngine::Err::errNone) {
        face_ar_engine.initFeatureIOParams();
        printf("DoApp::setMode faceMeshGeneration \n");
        break;
      } else if (nvErr == FaceEngine::Err::errInitialization) {
          printf("FaceEngine::Err::errInitialization mode 3\n");
      }
    case 2:
      face_ar_engine.destroyFeatures();
      face_ar_engine.setAppMode(FaceEngine::mode::landmarkDetection);
      face_ar_engine.createFeatures(FLAG_modelPath.c_str());
      face_ar_engine.initFeatureIOParams();
      printf("DoApp::setMode landmarkDetection \n");
      break;
    case 1:
      face_ar_engine.destroyFeatures();
      face_ar_engine.setAppMode(FaceEngine::mode::faceDetection);
      face_ar_engine.createFeatures(FLAG_modelPath.c_str());
      face_ar_engine.initFeatureIOParams();
      printf("DoApp::setMode faceDetection \n");
      break;
    default:
      break;
  }
}

DoApp::Err DoApp::initFaceEngine(const char *modelPath) {
  Err err = errNone;

  //if (!cap.isOpened()) return errVideo;

  nvErr = face_ar_engine.createFeatures(modelPath);
  if (nvErr != FaceEngine::Err::errNone) {
    if (nvErr == FaceEngine::Err::errInitialization && face_ar_engine.appMode == FaceEngine::mode::faceMeshGeneration) {
      printf("FaceEngine::Err::errInitialization initFaceEngine\n");
      face_ar_engine.destroyFeatures();
      face_ar_engine.setAppMode(FaceEngine::mode::landmarkDetection);
      nvErr = face_ar_engine.createFeatures(modelPath);
    }
    if (nvErr != FaceEngine::Err::errNone)
      err = errInitFaceEngine;
  }

  frameIndex = 0;
  printf("FaceEngine:: initFaceEngine OK\n");
  return err;
}

void DoApp::stop() {

  _continue = false;
  face_ar_engine.destroyFeatures();

  cap.release();
}

DoApp::Err DoApp::acquireFrame() {
  Err err = errNone;

  // If the machine goes to sleep with the app running and then wakes up, the camera object is not destroyed but the
  // frames we try to read are empty. So we try to re-initialize the camera with the same resolution settings. If the
  // resolution has changed, you will need to destroy and create the features again with the new camera resolution (not
  // done here) as well as reallocate memory accordingly with FaceEngine::initFeatureIOParams()
  cap >> frame;  // get a new frame from camera
  if (frame.empty()) {
    // try Init one more time if reading frames from camera
    initCamera(camIndex, inputWidth, inputHeight);
    cap >> frame;
    if (frame.empty()) return errVideo;
  }

  return err;
}

DoApp::Err DoApp::acquireFaceBox() {
  Err err = errNone;
  // get landmarks in  original image resolution coordinate space
  unsigned n = face_ar_engine.acquireFaceBox(frame, unity_output_bbox, 0);

  unity_num_boxes = n;

  if (0 == n) return errNoFace;

  return err;
}

DoApp::Err DoApp::acquireFaceBoxAndLandmarks() {
  Err err = errNone;

  // get landmarks in  original image resolution coordinate space
  unsigned n = face_ar_engine.acquireFaceBoxAndLandmarks(frame, facial_landmarks, unity_output_bbox, 0);

  /*
   printf("Landmarks: [\n");
    NvAR_Point2f *pt, *endPt;
    for (endPt = (pt = (NvAR_Point2f *)facial_landmarks) + FaceEngine::NUM_LANDMARKS; pt < endPt; ++pt)
      printf("%7.1f%7.1f\n", pt->x, pt->y);
    printf("]\n");
  */

  if (0 == n) return errNoFace;

  return err;
}

DoApp::Err DoApp::initCamera(int cameraIndex, int inCameraWidth, int inCameraHeight) {
    camIndex = cameraIndex;
  if (cap.open(cameraIndex, CV_CAP_DSHOW)) {

      if (inCameraWidth) cap.set(CV_CAP_PROP_FRAME_WIDTH, inCameraWidth);
      if (inCameraHeight) cap.set(CV_CAP_PROP_FRAME_HEIGHT, inCameraHeight);

      inputWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
      inputHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
      face_ar_engine.setInputImageWidth(inputWidth);
      face_ar_engine.setInputImageHeight(inputHeight);
      printf("DoApp::initCamera - OK size: %dx%d \n",inputWidth, inputHeight);
  } 
  else {
      printf("DoApp::initCamera - Error init camera \n");
      return errOpenCVInit;
    }
  return errNone;
}

DoApp::Err DoApp::fitFaceModel() {
  DoApp::Err doErr = errNone;
  nvErr = face_ar_engine.fitFaceModel(frame);

  if (FaceEngine::Err::errNone == nvErr) {
  } else {
    doErr = errFaceFit;
  }

  return doErr;
}

DoApp::DoApp() {
  // Make sure things are initialized properly
  gApp = this;
  drawVisualization = false;
  showFPS = false;
  captureVideo = false;
  captureFrame = false;
  frameTime = 0;
  frameIndex = 0;
  _continue = true;
  nvErr = FaceEngine::errNone;
  scaleOffsetXY[0] = scaleOffsetXY[2] = 1.f;
  scaleOffsetXY[1] = scaleOffsetXY[3] = 0.f;
}

DoApp::~DoApp() {}

void DoApp::getFPS() {
  const float timeConstant = 16.f;
  frameTimer.stop();
  float t = (float)frameTimer.elapsedTimeFloat();
  if (t < 100.f) {
    if (frameTime)
      frameTime += (t - frameTime) * (1.f / timeConstant);  // 1 pole IIR filter
    else
      frameTime = t;
  } else {            // Ludicrous time interval; reset
    frameTime = 0.f;  // WAKE UP
  }
  frameTimer.start();
}

DoApp::Err DoApp::update() {
    DoApp::Err doErr = errNone;

    if (face_ar_engine.appMode == FaceEngine::mode::faceDetection) {
        doErr = acquireFaceBox();
    }
    else if (face_ar_engine.appMode == FaceEngine::mode::landmarkDetection) {
        doErr = acquireFaceBoxAndLandmarks();
    }
    else if (face_ar_engine.appMode == FaceEngine::mode::faceMeshGeneration) {
        doErr = fitFaceModel();
    }

    return doErr;
}

const char *DoApp::errorStringFromCode(DoApp::Err code) {
  struct LUTEntry {
    Err code;
    const char *str;
  };
  static const LUTEntry lut[] = {
      {errNone, "no error"},
      {errUnimplemented, "the feature is unimplemented"},
      {errMissing, "missing input parameter"},
      {errVideo, "no video source has been found"},
      {errImageSize, "the image size cannot be accommodated"},
      {errNotFound, "the item cannot be found"},
      {errFaceModelInit, "face model initialization failed"},
      {errGLFWInit, "GLFW initialization failed"},
      {errGLInit, "OpenGL initialization failed"},
      {errRendererInit, "renderer initialization failed"},
      {errGLResource, "an OpenGL resource could not be found"},
      {errGLGeneric, "an otherwise unspecified OpenGL error has occurred"},
      {errFaceFit, "an error has occurred while face fitting"},
      {errNoFace, "no face has been found"},
      {errSDK, "an SDK error has occurred"},
      {errCuda, "a CUDA error has occurred"},
      {errCancel, "the user cancelled"},
      {errInitFaceEngine, "an error occurred while initializing the Face Engine"},
      {errOpenCVInit, "an error occurred while initializing the OpenCV"},
  };
  for (const LUTEntry *p = lut; p < &lut[sizeof(lut) / sizeof(lut[0])]; ++p)
    if (p->code == code) return p->str;
  static char msg[18];
  snprintf(msg, sizeof(msg), "error #%d", code);
  return msg;
}

/********************************************************************************
 * main
 ********************************************************************************/



  DoApp app;
  DoApp::Err doErr;
  NvCV_Status nvErr;

  DoApp::Err InitFaceEngine() {
      app.face_ar_engine.setFaceStabilization(FLAG_temporal);

      doErr = app.initFaceEngine(FLAG_modelPath.c_str());
      if (DoApp::errNone != doErr) {
          printf("ERROR: %s\n", app.errorStringFromCode(doErr));
          return doErr;
      }
      else {
          app.face_ar_engine.initFeatureIOParams();
          app.setMode(1);
          //doErr = app.run();
      }
      return DoApp::errNone;
  }


  extern "C" int __declspec(dllexport) __stdcall _Init(int cameraIndex, int inCameraWidth, int inCameraHeight)
  {
      app.face_ar_engine.setFaceStabilization(FLAG_temporal);

      doErr = app.initCamera(cameraIndex, inCameraWidth, inCameraHeight);
      if (doErr != DoApp::errNone) {
          return (int)doErr;
      }

      doErr = app.initFaceEngine(FLAG_modelPath.c_str());
      if (DoApp::errNone != doErr) {
          printf("ERROR: %s\n", app.errorStringFromCode(doErr));
          return doErr;
      }
      else {
          app.face_ar_engine.initFeatureIOParams();
          app.setMode(1);
          //doErr = app.run();
      }

      return (int)DoApp::errNone;
  }

  extern "C" int __declspec(dllexport) __stdcall _Close()
  {
      app.stop();
      return (int)doErr;
  }

  extern "C" void __declspec(dllexport) __stdcall _SetMode(int mode)
  {
      app.setMode(mode);
  }


  extern "C" int __declspec(dllexport) __stdcall _UpdateCamera()
  {

      //printf("image %dx%d\n", app.face_ar_engine.input_image_width, app.face_ar_engine.input_image_height);

      return (int)app.update();
  }

  extern "C" int __declspec(dllexport) __stdcall _GetRawImageBytes(unsigned char* data, int width, int height)
  {
      
      //DoApp::Err doErr = app.acquireFrame();
      //if (doErr != DoApp::errNone) return doErr;
      
      cv::Mat resizedMat(height, width, app.frame.type());
      cv::resize(app.frame, resizedMat, resizedMat.size(), cv::INTER_CUBIC);

      //Convert from RGB to ARGB 
      cv::Mat argb_img;
      cv::cvtColor(resizedMat, argb_img, CV_RGB2BGRA);
      std::vector<cv::Mat> bgra;
      cv::split(argb_img, bgra);
      std::swap(bgra[0], bgra[3]);
      std::swap(bgra[1], bgra[2]);
      std::memcpy(data, argb_img.data, argb_img.total() * argb_img.elemSize());

      return (int)app.update();
  }

  extern "C" int __declspec(dllexport) __stdcall _UpdateBox(NvAR_Rect* unity_output_bbox)
  {

      std::memcpy(unity_output_bbox, &app.unity_output_bbox, sizeof(app.unity_output_bbox));

      return app.unity_num_boxes;

  }

  extern "C" void __declspec(dllexport) __stdcall _UpdateLandmarks(NvAR_Point2f* unity_landmarks, int size, NvAR_Quaternion* unity_pose)
  {

      for (int i = 0; i < size; i++)
      {
          unity_landmarks[i] = app.facial_landmarks[i];
      }

      std::memcpy(unity_pose, app.face_ar_engine.getPose(), sizeof(struct NvAR_Quaternion));

  }

  extern "C" void __declspec(dllexport) __stdcall _UpdateFaceMesh(NvAR_Vector3f * unity_vertices, int vertices_size, NvAR_Vector3u16* unity_triangles, int triangles_size)
  {

      NvAR_FaceMesh* faceMesh = app.face_ar_engine.getFaceMesh();

      printf("faceMesh vertices %d triangles %d\n", faceMesh->num_vertices, faceMesh->num_tri_idx);


      for (int i = 0; i < vertices_size; i++)
      {
          unity_vertices[i].vec[0] = faceMesh->vertices[i].vec[0] / 1000.0;
          unity_vertices[i].vec[1] = faceMesh->vertices[i].vec[1] / 1000.0;
          unity_vertices[i].vec[2] = faceMesh->vertices[i].vec[2] / 1000.0;
      }


    for (int i = 0; i < triangles_size; i++)
    {
        unity_triangles[i].vec[0] = faceMesh->tvi[i].vec[0];
        unity_triangles[i].vec[1] = faceMesh->tvi[i].vec[1];
        unity_triangles[i].vec[2] = faceMesh->tvi[i].vec[2];
    }
  }
  extern "C" void __declspec(dllexport) __stdcall _UpdateRenderingParams(NvAR_RenderingParams * unity_rp)
  {

      NvAR_RenderingParams* rp = app.face_ar_engine.getRenderingParams();

      std::memcpy(unity_rp, rp, sizeof(struct NvAR_RenderingParams));

  }



  static void* g_TextureHandle = NULL;
  static int   g_TextureWidth = 0;
  static int   g_TextureHeight = 0;

  extern "C" int UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API _InitWithTexture(void* textureHandle, int w, int h)
  {
      app.face_ar_engine.setFaceStabilization(FLAG_temporal);

      g_TextureHandle = textureHandle;
      g_TextureWidth = w;
      g_TextureHeight = h;

      cv::Mat unityFrame(g_TextureHeight, g_TextureWidth, CV_8UC4, g_TextureHandle);
      cv::flip(unityFrame, unityFrame, -1);
      cv::cvtColor(unityFrame, app.frame, cv::COLOR_BGRA2RGB);

      //cv::imshow("Unity Texture", unityFrame);
      //cv::imshow("mirrorMat Texture", mirrorMat);
      //cv::imshow("app.frame Texture", app.frame);
      
      app.inputWidth = w;
      app.inputHeight = h;
      app.face_ar_engine.setInputImageWidth(w);
      app.face_ar_engine.setInputImageHeight(h);

      doErr = app.initFaceEngine(FLAG_modelPath.c_str());
      if (DoApp::errNone != doErr) {
          printf("_InitWithTexture - ERROR: %s\n", app.errorStringFromCode(doErr));
          return (int)doErr;
      }
      else {
          app.face_ar_engine.initFeatureIOParams();
          app.setMode(3);
          //doErr = app.run();
      }

      app.face_ar_engine.setFaceStabilization(true);

      return (int)DoApp::errNone;
  }

  // --------------------------------------------------------------------------
// UnitySetInterfaces

  static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

  static IUnityInterfaces* s_UnityInterfaces = NULL;
  static IUnityGraphics* s_Graphics = NULL;

  extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces * unityInterfaces)
  {
      s_UnityInterfaces = unityInterfaces;
      s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
      s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);

#if SUPPORT_VULKAN
      if (s_Graphics->GetRenderer() == kUnityGfxRendererNull)
      {
          extern void RenderAPI_Vulkan_OnPluginLoad(IUnityInterfaces*);
          RenderAPI_Vulkan_OnPluginLoad(unityInterfaces);
      }
#endif // SUPPORT_VULKAN

      // Run OnGraphicsDeviceEvent(initialize) manually on plugin load
      OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
  }

  extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
  {
      s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
  }

  // --------------------------------------------------------------------------
// GraphicsDeviceEvent


  static RenderAPI* s_CurrentAPI = NULL;
  static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;

  static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
  {
      // Create graphics API implementation upon initialization
      if (eventType == kUnityGfxDeviceEventInitialize)
      {
          assert(s_CurrentAPI == NULL);
          s_DeviceType = s_Graphics->GetRenderer();
          s_CurrentAPI = CreateRenderAPI(s_DeviceType);
      }

      // Let the implementation process the device related events
      if (s_CurrentAPI)
      {
          s_CurrentAPI->ProcessDeviceEvent(eventType, s_UnityInterfaces);
      }

      // Cleanup graphics API implementation upon shutdown
      if (eventType == kUnityGfxDeviceEventShutdown)
      {
          delete s_CurrentAPI;
          s_CurrentAPI = NULL;
          s_DeviceType = kUnityGfxRendererNull;
      }
  }


  static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
  {
      // Unknown / unsupported graphics device type? Do nothing
      if (s_CurrentAPI == NULL)
          return;

      cv::Mat unityFrame(g_TextureHeight, g_TextureWidth, CV_8UC4, g_TextureHandle);
      cv::flip(unityFrame, unityFrame, -1);
      cv::cvtColor(unityFrame, app.frame, cv::COLOR_BGRA2RGB);

      if (eventID == 2) {
          cv::imshow("UnityFrame Texture", unityFrame);
          cv::imshow("app.frame Texture", app.frame);
          cv::waitKey(0);
      }
      if (eventID == 3) {
          cv::destroyAllWindows();
      }

  }


  // --------------------------------------------------------------------------
  // GetRenderEventFunc, an example function we export which is used to get a rendering event callback function.

  extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc()
  {
      return OnRenderEvent;
  }

  
