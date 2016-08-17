/*
 *  File:mrpt_ekf_slam_3d.h
 *  Author: Vladislav Tananaev
 *
 */


#ifndef MRPT_EKF_SLAM_3D_H
#define MRPT_EKF_SLAM_3D_H

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/gui/CDisplayWindow3D.h>
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace std;


/**
 * @brief The EKFslam class provides EKF SLAM 3d from MRPT libraries. 
 *   
 */
class EKFslam{

public:
   /**
   * @brief constructor
   */
    EKFslam();
   /**
   * @brief destructor
   */
  virtual  ~EKFslam();
  /**
   * @brief init 3D window from mrpt lib
   */
    void init3Dwindow();
   /**
   * @brief run 3D window update from mrpt lib
   */
    void run3Dwindow();  
   /**
   * @brief convert landmark to 3d point
   */
    void landmark_to_3d(const CRangeBearingKFSLAM::KFArray_FEAT &lm, TPoint3D &p);
  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
    void read_iniFile(std::string ini_filename);
   /**
   * @brief calculate the actions from odometry model for current observation
   *
   * @param _sf  current observation
   * @param _odometry raw odometry
   */   
    void observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry);

protected:
    CRangeBearingKFSLAM mapping;///<EKF slam 3d class

    mrpt::system::TTimeStamp timeLastUpdate_;///< last update of the pose and map

    CActionCollectionPtr action;///< actions
	CSensoryFramePtr sf;///< observations
 
    mrpt::poses::CPose3D odomLastObservation_; ///< last observation of odometry
	CActionRobotMovement3D::TMotionModelOptions motion_model_options_;         ///< used with odom value motion noise

    mrpt::gui::CDisplayWindow3DPtr	win3d;///<MRPT window 
    bool  SHOW_3D_LIVE;
    bool  CAMERA_3DSCENE_FOLLOWS_ROBOT;
    vector<TPose3D>  meanPath;
    CPose3DQuatPDFGaussian	  robotPose_;///< current robot pose
    std::vector<mrpt::math::TPoint3D> 	 LMs_;///< vector of the landmarks
	std::map<unsigned int,CLandmark::TLandmarkID>    LM_IDs_;///< vector of the landmarks ID
	CMatrixDouble  fullCov_;///< full covariance matrix
	CVectorDouble  fullState_;///< full state vector 
};


#endif /* MRPT_EKF_SLAM_3D_H */
