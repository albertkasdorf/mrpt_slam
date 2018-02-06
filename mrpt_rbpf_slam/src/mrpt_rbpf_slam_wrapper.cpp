#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"

PFslamWrapper::PFslamWrapper()
{
  rawlog_play_ = false;
  mrpt_bridge::convert(ros::Time(0), timeLastUpdate_);
}

PFslamWrapper::~PFslamWrapper()
{
}

bool PFslamWrapper::is_file_exists(const std::string& name)
{
  std::ifstream f(name.c_str());
  return f.good();
}

void PFslamWrapper::get_param()
{
  ROS_INFO("READ PARAM FROM LAUNCH FILE");
  n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
  ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

  n_.getParam("rawlog_filename", rawlog_filename);
  ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

  n_.getParam("ini_filename", ini_filename);
  ROS_INFO("ini_filename: %s", ini_filename.c_str());

  n_.param<std::string>("global_frame_id", global_frame_id, "map");
  ROS_INFO("global_frame_id: %s", global_frame_id.c_str());

  n_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

  n_.param<std::string>("base_frame_id", base_frame_id, "base_link");
  ROS_INFO("base_frame_id: %s", base_frame_id.c_str());

  n_.param<std::string>("sensor_source", sensor_source, "scan");
  ROS_INFO("sensor_source: %s", sensor_source.c_str());
}

void PFslamWrapper::init()
{
  // get parameters from ini file
  if (!is_file_exists(ini_filename))
  {
    ROS_ERROR_STREAM("CAN'T READ INI FILE");
    return;
  }
  PFslam::read_iniFile(ini_filename);
  // read rawlog file if it  exists
  if (is_file_exists(rawlog_filename))
  {
    ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
    PFslam::read_rawlog(data, rawlog_filename);
    rawlog_play_ = true;
  }

  /// Create publishers///
  // publish grid map
  pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  pub_metadata_ = n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  // robot pose
  pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
  // ro particles poses
  pub_Particles_Beacons_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud_beacons", 1, true);
  beacon_viz_pub_ = n_.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 1);

  pub_bor = n_.advertise<mrpt_msgs::BeaconObservationResult>("beacon_observation_result", 1);
  pub_pose = n_.advertise<geometry_msgs::PoseStamped>("pose_trajectory", 1);
  pub_pose_estimate = n_.advertise<geometry_msgs::PoseArray>("pose_estimation", 1);

  // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. "
                                      "\"scan\" or \"beacon\")");

  /// Create subscribers///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++)
  {
    if (lstSources[i].find("scan") != std::string::npos)
    {
      sensorSub_[i] = n_.subscribe(lstSources[i], 1, &PFslamWrapper::laserCallback, this);
    }
    else
    {
      sensorSub_[i] = n_.subscribe(lstSources[i], 1, &PFslamWrapper::callbackBeacon, this);
    }
  }

  // init slam
  mapBuilder = new mrpt::slam::CMetricMapBuilderRBPF(rbpfMappingOptions);
  init_slam();
  init3Dwindow();
}

void PFslamWrapper::odometryForCallback(CObservationOdometry::Ptr& _odometry, const std_msgs::Header& _msg_header)
{
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1)))
  {
    _odometry = CObservationOdometry::Create();
    _odometry->sensorLabel = odom_frame_id;
    _odometry->hasEncodersInfo = false;
    _odometry->hasVelocities = false;
    _odometry->odometry.x() = poseOdom.x();
    _odometry->odometry.y() = poseOdom.y();
    _odometry->odometry.phi() = poseOdom.yaw();
  }
}

bool PFslamWrapper::waitForTransform(mrpt::poses::CPose3D& des, const std::string& target_frame,
                                     const std::string& source_frame, const ros::Time& time,
                                     const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
  tf::StampedTransform transform;
  try
  {
    listenerTF_.waitForTransform(target_frame, source_frame, time, timeout, polling_sleep_duration);
    listenerTF_.lookupTransform(target_frame, source_frame, time, transform);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR(
      "Failed to get transform target_frame (%s) to source_frame (%s). TransformException: %s",
      target_frame.c_str(),
      source_frame.c_str(),
      ex.what());
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

void PFslamWrapper::laserCallback(const sensor_msgs::LaserScan& _msg)
{
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif
  CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();

  if (laser_poses_.find(_msg.header.frame_id) == laser_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
    mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id], *laser);

    sf = CSensoryFrame::Create();
    CObservationOdometry::Ptr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservation::Ptr obs = CObservation::Ptr(laser);
    sf->insert(obs);
    observation(sf, odometry);
    timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

    tictac.Tic();
    mapBuilder->processActionObservation(*action, *sf);
    t_exec = tictac.Tac();
    ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);
    publishMapPose(_msg.header);
    run3Dwindow();
    publishTF();
  }
}

void PFslamWrapper::callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& _msg)
{
#if MRPT_VERSION >= 0x130
  using namespace mrpt::maps;
  using namespace mrpt::obs;
#else
  using namespace mrpt::slam;
#endif

  CObservationBeaconRanges::Ptr beacon = CObservationBeaconRanges::Create();
  if (beacon_poses_.find(_msg.header.frame_id) == beacon_poses_.end())
  {
    updateSensorPose(_msg.header.frame_id);
  }
  else
  {
    mrpt_bridge::convert(_msg, beacon_poses_[_msg.header.frame_id], *beacon);

    sf = CSensoryFrame::Create();
    CObservationOdometry::Ptr odometry;
    odometryForCallback(odometry, _msg.header);

    CObservation::Ptr obs = CObservation::Ptr(beacon);
    sf->insert(obs);
    observation(sf, odometry);
    timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

    tictac.Tic();
    mapBuilder->processActionObservation(*action, *sf);
    t_exec = tictac.Tac();
    ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

    publishMapPose(_msg.header);
    run3Dwindow();
  }
}

void PFslamWrapper::publishMapPose(const std_msgs::Header& _msg_header)
{
  // if I received new grid maps from 2D laser scan sensors
  metric_map_ = mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
  mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);
  if (metric_map_->m_gridMaps.size())
  {
    // publish map
    nav_msgs::OccupancyGrid _msg;
    mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
    pub_map_.publish(_msg);
    pub_metadata_.publish(_msg.info);
  }

  // if I received new beacon (range only) map
  if (metric_map_->m_beaconMap)
  {
    mrpt::opengl::CSetOfObjects::Ptr objs;

    objs = mrpt::opengl::CSetOfObjects::Create();
    // Get th map as the set of 3D objects
    metric_map_->m_beaconMap->getAs3DObject(objs);

    geometry_msgs::PoseArray poseArrayBeacons;
    poseArrayBeacons.header.frame_id = global_frame_id;
    poseArrayBeacons.header.stamp = ros::Time::now( );

    // Count the number of beacons
    unsigned int objs_counter = 0;
    while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter))
    {
      objs_counter++;
    }
    poseArrayBeacons.poses.resize(objs_counter);
    mrpt::opengl::CEllipsoid::Ptr beacon_particle;

    for (size_t i = 0; i < objs_counter; i++)
    {
      beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
      mrpt_bridge::convert(mrpt::poses::CPose3D(beacon_particle->getPose()), poseArrayBeacons.poses[i]);
      viz_beacons.push_back(beacon_particle);
    }
    pub_Particles_Beacons_.publish(poseArrayBeacons);
    vizBeacons();
    viz_beacons.clear();

    //----
    // Positionsschätzung für die Beacons
    //----
    mrpt_msgs::BeaconObservationResult bor;
    bor.header.frame_id = global_frame_id;
    bor.header.stamp = _msg_header.stamp;

    for(  CBeaconMap::const_iterator it = metric_map_->m_beaconMap->begin();
          it != metric_map_->m_beaconMap->end();
          ++it )
    {
      mrpt::poses::CPoint3D mean;
      mrpt::math::CMatrixDouble33 cov;

      it->getCovarianceAndMean( cov, mean );
      
      bor.beacon_id = it->m_ID;
      bor.pdf_type = it->m_typePDF;

      bor.mean[0] = mean.x();   bor.mean[1] = mean.y();
      bor.cov[0] = cov(0, 0);   bor.cov[1] = cov(0, 1);
      bor.cov[2] = cov(1, 0);   bor.cov[3] = cov(1, 1);

      if(it->m_typePDF == CBeacon::TTypePDF::pdfMonteCarlo)
      {
        const size_t N = it->m_locationMC.m_particles.size();
        const size_t ITEMS = 3;

        bor.mc.resize(N * ITEMS);

        for(size_t i = 0; i < N; ++i)
        {
          bor.mc[(i * ITEMS) + 0] = it->m_locationMC.m_particles[i].d->x;
          bor.mc[(i * ITEMS) + 1] = it->m_locationMC.m_particles[i].d->y;
          bor.mc[(i * ITEMS) + 2] = it->m_locationMC.m_particles[i].log_w;
        }
      }
      else if(it->m_typePDF == CBeacon::TTypePDF::pdfSOG)
      {
        const size_t N = it->m_locationSOG.size();
        const size_t ITEMS = 7;

        bor.sog.resize(N * ITEMS);

        for(size_t i = 0; i < N; ++i)
        {
          bor.sog[(i * ITEMS) + 0] = it->m_locationSOG[i].val.mean.x();
          bor.sog[(i * ITEMS) + 1] = it->m_locationSOG[i].val.mean.y();
          bor.sog[(i * ITEMS) + 2] = it->m_locationSOG[i].val.cov(0, 0);
          bor.sog[(i * ITEMS) + 3] = it->m_locationSOG[i].val.cov(0, 1);
          bor.sog[(i * ITEMS) + 4] = it->m_locationSOG[i].val.cov(1, 0);
          bor.sog[(i * ITEMS) + 5] = it->m_locationSOG[i].val.cov(1, 1);
          bor.sog[(i * ITEMS) + 6] = it->m_locationSOG[i].log_w;
        }
      }
      else { }

      pub_bor.publish( bor );
    }
  }

  // publish pose
  geometry_msgs::PoseArray poseArray;
  poseArray.header.frame_id = global_frame_id;
  poseArray.header.stamp = ros::Time::now();
  poseArray.poses.resize(curPDF.particlesCount());
  for (size_t i = 0; i < curPDF.particlesCount(); i++)
  {
    mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
    mrpt_bridge::convert(p, poseArray.poses[i]);
  }

  pub_Particles_.publish(poseArray);

  //----
  // Pose auf der Trajektorie zum Zeitpunkt _msg_header.stamp
  //----
  mrpt::poses::CPose3D pose_in_map;
  if(waitForTransform(pose_in_map, global_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1)))
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = global_frame_id;
    pose_stamped.header.stamp = _msg_header.stamp;

    geometry_msgs::Pose pose;

    mrpt_bridge::convert(pose_in_map, pose);
    pose_stamped.pose = pose; 

    pub_pose.publish(pose_stamped); 
  }

  //----
  // Pose Schätzung zum Zeitpunkt _msg_header.stamp
  //----
  geometry_msgs::PoseArray pose_estimation;
  pose_estimation.header.frame_id = global_frame_id;
  pose_estimation.header.stamp = _msg_header.stamp;
  pose_estimation.poses.resize(curPDF.particlesCount());

  for(size_t i = 0; i < curPDF.particlesCount(); ++i)
  {
    mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
    mrpt_bridge::convert(p, pose_estimation.poses[i]);
  }
  pub_pose_estimate.publish(pose_estimation);
}

void PFslamWrapper::vizBeacons()
{
  if (viz_beacons.size() == 0)
  {
    return;
  }
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";

  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1);
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.12;
  marker.scale.y = 0.12;
  marker.scale.z = 0.12;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;

  for (unsigned int i = 0; i < viz_beacons.size(); i++)
  {
    CPose3D meanPose(viz_beacons[i]->getPose());
    marker.type = visualization_msgs::Marker::SPHERE;

    marker.pose.position.x = meanPose.x();
    marker.pose.position.y = meanPose.y();
    marker.pose.position.z = meanPose.z();
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    ma.markers.push_back(marker);
    marker.id++;

    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = std::to_string(i);

    marker.pose.position.x = meanPose.x();
    marker.pose.position.y = meanPose.y();
    marker.pose.position.z = meanPose.z() + 0.12;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    // marker.scale.z = 1;
    ma.markers.push_back(marker);
    marker.id++;
  }

  beacon_viz_pub_.publish(ma);
}

void PFslamWrapper::updateSensorPose(std::string _frame_id)
{
  CPose3D pose;
  tf::StampedTransform transform;
  try
  {
    listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);

    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    tf::Matrix3x3 Rsrc(quat);
    CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[_frame_id] = pose;
    beacon_poses_[_frame_id] = pose;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

bool PFslamWrapper::rawlogPlay()
{
  if (rawlog_play_ == false)
  {
    return false;
  }
  else
  {
    for (unsigned int i = 0; i < data.size(); i++)
    {
      if (ros::ok())
      {
        tictac.Tic();
        mapBuilder->processActionObservation(data[i].first, data[i].second);
        t_exec = tictac.Tac();
        ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);

        ros::Duration(rawlog_play_delay).sleep();

        metric_map_ = mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
        mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);
        // if I received new grid maps from 2D laser scan sensors
        if (metric_map_->m_gridMaps.size())
        {
          nav_msgs::OccupancyGrid _msg;

          // if we have new map for current sensor update it
          mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg);
          pub_map_.publish(_msg);
          pub_metadata_.publish(_msg.info);
        }

        // if I received new beacon (range only) map
        if (metric_map_->m_beaconMap)
        {
          mrpt::opengl::CSetOfObjects::Ptr objs;
          objs = mrpt::opengl::CSetOfObjects::Create();
          metric_map_->m_beaconMap->getAs3DObject(objs);

          geometry_msgs::PoseArray poseArrayBeacons;
          poseArrayBeacons.header.frame_id = global_frame_id;
          poseArrayBeacons.header.stamp = ros::Time::now();

          unsigned int objs_counter = 0;
          while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter))
          {
            objs_counter++;
          }
          poseArrayBeacons.poses.resize(objs_counter);
          mrpt::opengl::CEllipsoid::Ptr beacon_particle;

          for (size_t i = 0; i < objs_counter; i++)
          {
            beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
            mrpt_bridge::convert(mrpt::poses::CPose3D(beacon_particle->getPose()), poseArrayBeacons.poses[i]);
            viz_beacons.push_back(beacon_particle);
          }
          pub_Particles_Beacons_.publish(poseArrayBeacons);
          vizBeacons();
          viz_beacons.clear();
        }

        // publish pose
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = global_frame_id;
        poseArray.header.stamp = ros::Time::now();
        poseArray.poses.resize(curPDF.particlesCount());
        for (size_t i = 0; i < curPDF.particlesCount(); i++)
        {
          mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
          mrpt_bridge::convert(p, poseArray.poses[i]);
        }

        pub_Particles_.publish(poseArray);
      }
      ros::spinOnce();
      run3Dwindow();
    }
  }
  // if there is mrpt_gui it will wait until push any key in order to close the window
  if (win3D)
    win3D->waitForKey();
  return true;
}

void PFslamWrapper::publishTF()
{
  // Most of this code was copy and pase from ros::amcl
  mrpt::poses::CPose3D robotPose;
  mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);

  curPDF.getMean(robotPose);

  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  ros::Time stamp;
  mrpt_bridge::convert(timeLastUpdate_, stamp);
  mrpt_bridge::convert(robotPose, tmp_tf);

  try
  {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp, base_frame_id);
    listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR(
      "Failed to subtract global_frame (%s) from odom_frame (%s). TransformException: %s",
      global_frame_id.c_str(),
      odom_frame_id.c_str(),
      ex.what());
    return;
  }

  tf::Transform latest_tf_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used

  ros::Duration transform_tolerance_(0.5);
  ros::Time transform_expiration = (stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), transform_expiration, global_frame_id, odom_frame_id);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}
