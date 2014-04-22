/*
   Work in progress: patch by Marco (AUG,19th 2012)
   > oni fixed
   > pcl added: mostly to include rgb treatment while grabbing from PCD files obtained by pcl_openni_grab_frame -noend 
   > sync issue fixed
   > volume_size issue fixed
   > world.pcd write exception on windows fixed on new trunk version

   + minor changes
   */

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>

#include <pcl/console/parse.h>

#include <boost/filesystem.hpp>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>

#include <pcl/common/time.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/grabber.h>
#include <pcl/io/pcd_grabber.h>

#include <pcl/common/angles.h>

typedef pcl::ScopeTime ScopeTimeT;

#include "../src/internal.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

using namespace pcl::gpu::kinfuLS;

using pcl::gpu::DeviceArray;
using pcl::gpu::DeviceArray2D;
using pcl::gpu::PtrStepSz;

namespace pc = pcl::console;

namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      void mergePointNormal (const DeviceArray<PointXYZ>& cloud, const DeviceArray<Normal>& normals, DeviceArray<PointNormal>& output);
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

vector<string> getPcdFilesInDir(const string& directory)
{
  namespace fs = boost::filesystem;
  fs::path dir(directory);

  std::cout << "path: " << directory << std::endl;
  if (directory.empty() || !fs::exists(dir) || !fs::is_directory(dir))
    PCL_THROW_EXCEPTION (pcl::IOException, "No valid PCD directory given!\n");

  vector<string> result;
  fs::directory_iterator pos(dir);
  fs::directory_iterator end;           

  for(; pos != end ; ++pos)
    if (fs::is_regular_file(pos->status()) )
      if (fs::extension(*pos) == ".pcd")
      {
#if BOOST_FILESYSTEM_VERSION == 3
        result.push_back (pos->path ().string ());
#else
        result.push_back (pos->path ());
#endif
        cout << "added: " << result.back() << endl;
      }

  return result;  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct SampledScopeTime : public StopWatch
{          
  enum { EACH = 33 };
  SampledScopeTime(int& time_ms) : time_ms_(time_ms) {}
  ~SampledScopeTime()
  {
    static int i_ = 0;
    time_ms_ += getTime ();    
    if (i_ % EACH == 0 && i_)
    {
      cout << "Average frame time = " << time_ms_ / EACH << "ms ( " << 1000.f * EACH / time_ms_ << "fps )" << endl;
      time_ms_ = 0;        
    }
    ++i_;
  }
  private:    
  int& time_ms_;    
};

template<typename CloudT> void
writeCloudFile (int format, const CloudT& cloud);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh);


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  template<typename MergedT, typename PointT>
typename PointCloud<MergedT>::Ptr merge(const PointCloud<PointT>& points, const PointCloud<RGB>& colors)
{    
  typename PointCloud<MergedT>::Ptr merged_ptr(new PointCloud<MergedT>());

  pcl::copyPointCloud (points, *merged_ptr);      
  for (size_t i = 0; i < colors.size (); ++i)
    merged_ptr->points[i].rgba = colors.points[i].rgba;

  return merged_ptr;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const DeviceArray<PointXYZ>& triangles)
{ 
  if (triangles.empty())
    return boost::shared_ptr<pcl::PolygonMesh>();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width  = (int)triangles.size();
  cloud.height = 1;
  triangles.download(cloud.points);

  boost::shared_ptr<pcl::PolygonMesh> mesh_ptr( new pcl::PolygonMesh() ); 
  pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

  mesh_ptr->polygons.resize (triangles.size() / 3);
  for (size_t i = 0; i < mesh_ptr->polygons.size (); ++i)
  {
    pcl::Vertices v;
    v.vertices.push_back(i*3+0);
    v.vertices.push_back(i*3+2);
    v.vertices.push_back(i*3+1);              
    mesh_ptr->polygons[i] = v;
  }    
  return mesh_ptr;
}

struct KinFuLSApp
{
  enum { PCD_BIN = 1, PCD_ASCII = 2, PLY = 3, MESH_PLY = 7, MESH_VTK = 8 };

  KinFuLSApp ( pcl::Grabber& source, float vsz, float shiftDistance, int snapshotRate) : exit_ (false), scan_ (false), scan_mesh_(false), scan_volume_ (false),
  registration_ (false), integrate_colors_ (false), pcd_source_ (false), focal_length_(-1.f), was_lost_(false), time_ms_ (0)
  {    
    //Init Kinfu Tracker
    Eigen::Vector3f volume_size = Vector3f::Constant (vsz/*meters*/);    

    PCL_WARN ("--- CURRENT SETTINGS ---\n");
    PCL_INFO ("Volume size is set to %.2f meters\n", vsz);
    PCL_INFO ("Volume will shift when the camera target point is farther than %.2f meters from the volume center\n", shiftDistance);
    PCL_INFO ("The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6*vsz);
    PCL_WARN ("------------------------\n");

    // warning message if shifting distance is abnormally big compared to volume size
    if(shiftDistance > 2.5 * vsz)
      PCL_WARN ("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n", shiftDistance, vsz);

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shiftDistance);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f (t) * Eigen::AngleAxisf (R);

    kinfu_->setInitialCameraPose (pose);
    kinfu_->volume().setTsdfTruncDist (0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams (0.1f/*meters*/, sin ( pcl::deg2rad(20.f) ));
    //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);

    //Init KinFuLSApp            
    tsdf_cloud_ptr_ = pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>);

    frame_counter_ = 0;
    enable_texture_extraction_ = false;

    Eigen::Matrix3f Rid = Eigen::Matrix3f::Identity ();   // * AngleAxisf( pcl::deg2rad(-30.f), Vector3f::UnitX());
    Eigen::Vector3f T = Vector3f (0, 0, -volume_size(0)*1.5f);
    delta_lost_pose_ = Eigen::Translation3f (T) * Eigen::AngleAxisf (Rid); 

  }

  ~KinFuLSApp()
  {
  }

  void 
    toggleColorIntegration()
    {
      if(registration_)
      {
        const int max_color_integration_weight = 2;
        kinfu_->initColorIntegration(max_color_integration_weight);
        integrate_colors_ = true;      
      }    
      cout << "Color integration: " << (integrate_colors_ ? "On" : "Off ( requires registration mode )") << endl;
    }


  void execute(const PtrStepSz<const unsigned short>& depth, const PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB>& rgb24, bool has_data)
  {        
    bool has_image = false;
    frame_counter_++;

    was_lost_ = kinfu_->icpIsLost();

    if (has_data)
    {
      depth_device_.upload (depth.data, depth.step, depth.rows, depth.cols);
      {
        SampledScopeTime fps(time_ms_);
        //run kinfu algorithm
        has_image = (*kinfu_) (depth_device_);
      }
    }

    if (scan_ || (!was_lost_ && kinfu_->icpIsLost ()) ) //if scan mode is OR and ICP just lost itself => show current volume as point cloud
    {
      scan_ = false;
      //scene_cloud_view_.show (*kinfu_, integrate_colors_); // this triggers out of memory errors, so I comment it out for now (Raph)

      if (scan_volume_)
      {                
        cout << "Downloading TSDF volume from device ... " << flush;
        // kinfu_->volume().downloadTsdfAndWeighs (tsdf_volume_.volumeWriteable (), tsdf_volume_.weightsWriteable ());
        kinfu_->volume ().downloadTsdfAndWeightsLocal ();
        // tsdf_volume_.setHeader (Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z), kinfu_->volume().getSize ());
        kinfu_->volume ().setHeader (Eigen::Vector3i (pcl::device::kinfuLS::VOLUME_X, pcl::device::kinfuLS::VOLUME_Y, pcl::device::kinfuLS::VOLUME_Z), kinfu_->volume().getSize ());
        // cout << "done [" << tsdf_volume_.size () << " voxels]" << endl << endl;
        cout << "done [" << kinfu_->volume ().size () << " voxels]" << endl << endl;

        cout << "Converting volume to TSDF cloud ... " << flush;
        // tsdf_volume_.convertToTsdfCloud (tsdf_cloud_ptr_);
        kinfu_->volume ().convertToTsdfCloud (tsdf_cloud_ptr_);
        // cout << "done [" << tsdf_cloud_ptr_->size () << " points]" << endl << endl;        
        cout << "done [" << kinfu_->volume ().size () << " points]" << endl << endl;
      }
      else
        cout << "[!] tsdf volume download is disabled" << endl << endl;
    }
  }
  /*
     void source_cb2(const boost::shared_ptr<openni_wrapper::Image>& image_wrapper, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_wrapper, float)
     {
     {
     boost::mutex::scoped_try_lock lock(data_ready_mutex_);

     if (exit_ || !lock)
     {
     return;
     }

     depth_.cols = depth_wrapper->getWidth();
     depth_.rows = depth_wrapper->getHeight();
     depth_.step = depth_.cols * depth_.elemSize();

     source_depth_data_.resize(depth_.cols * depth_.rows);
     depth_wrapper->fillDepthImageRaw(depth_.cols, depth_.rows, &source_depth_data_[0]);
     depth_.data = &source_depth_data_[0];      

     rgb24_.cols = image_wrapper->getWidth();
     rgb24_.rows = image_wrapper->getHeight();
     rgb24_.step = rgb24_.cols * rgb24_.elemSize(); 

     source_image_data_.resize(rgb24_.cols * rgb24_.rows);
     image_wrapper->fillRGB(rgb24_.cols, rgb24_.rows, (unsigned char*)&source_image_data_[0]);
     rgb24_.data = &source_image_data_[0];    

     }
     data_ready_cond_.notify_one();
     }
     */

  void
    startMainLoop (bool triggered_capture)
    {   
      bool need_colors = integrate_colors_ || registration_ || enable_texture_extraction_;

      boost::unique_lock<boost::mutex> lock(data_ready_mutex_);

      while (!exit_ && !this->kinfu_->isFinished ())
      { 
        bool has_data = data_ready_cond_.timed_wait (lock, boost::posix_time::millisec(100));

        try { this->execute (depth_, rgb24_, has_data); }
        catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; break; }
        catch (const std::exception& /*e*/) { cout << "Exception" << endl; break; }
      } 
      exit_ = true;
      boost::this_thread::sleep (boost::posix_time::millisec (100));
    }

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*  void
    writeCloud (int format) const
    {      
      const SceneCloudView& view = scene_cloud_view_;

      if (!view.cloud_ptr_->points.empty ())
      {
        if(view.point_colors_ptr_->points.empty()) // no colors
        {
          if (view.valid_combined_)
            writeCloudFile (format, view.combined_ptr_);
          else
            writeCloudFile (format, view.cloud_ptr_);
        }
        else
        {        
          if (view.valid_combined_)
            writeCloudFile (format, merge<PointXYZRGBNormal>(*view.combined_ptr_, *view.point_colors_ptr_));
          else
            writeCloudFile (format, merge<PointXYZRGB>(*view.cloud_ptr_, *view.point_colors_ptr_));
        }
      }
    }*/

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*  void
    writeMesh(int format) const
    {
      if (scene_cloud_view_.mesh_ptr_) 
        writePolygonMeshFile(format, *scene_cloud_view_.mesh_ptr_);
    }*/

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
    printHelp ()
    {
      cout << endl;
      cout << "KinFu app hotkeys" << endl;
      cout << "=================" << endl;
      cout << "    H    : print this help" << endl;
      cout << "   Esc   : exit" << endl;
      cout << "    T    : take cloud" << endl;
      cout << "    A    : take mesh" << endl;
      cout << "    M    : toggle cloud exctraction mode" << endl;
      cout << "    N    : toggle normals exctraction" << endl;
      cout << "    I    : toggle independent camera mode" << endl;
      cout << "    B    : toggle volume bounds" << endl;
      cout << "    *    : toggle scene view painting ( requires registration mode )" << endl;
      cout << "    C    : clear clouds" << endl;    
      cout << "   1,2,3 : save cloud to PCD(binary), PCD(ASCII), PLY(ASCII)" << endl;
      cout << "    7,8  : save mesh to PLY, VTK" << endl;
      cout << "   X, V  : TSDF volume utility" << endl;
      cout << "   L, l  : On the next shift, KinFu will extract the whole current cube, extract the world and stop" << endl;
      cout << "   S, s  : On the next shift, KinFu will extract the world and stop" << endl;
      cout << endl;
    }  

  bool exit_;
  bool scan_;
  bool scan_mesh_;
  bool scan_volume_;

  bool independent_camera_;
  int frame_counter_;
  bool enable_texture_extraction_;
  int snapshot_rate_;

  bool registration_;
  bool integrate_colors_;
  bool pcd_source_;
  float focal_length_;

  KinfuTracker *kinfu_;

  KinfuTracker::DepthMap depth_device_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tsdf_cloud_ptr_;

  boost::mutex data_ready_mutex_;
  boost::condition_variable data_ready_cond_;

  std::vector<pcl::gpu::kinfuLS::PixelRGB> source_image_data_;
  std::vector<unsigned short> source_depth_data_;
  PtrStepSz<const unsigned short> depth_;
  PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24_;  

  Eigen::Affine3f delta_lost_pose_;

  bool was_lost_;

  int time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template<typename CloudPtr> void
writeCloudFile (int format, const CloudPtr& cloud_prt)
{
  if (format == KinFuLSApp::PCD_BIN)
  {
    cout << "Saving point cloud to 'cloud_bin.pcd' (binary)... " << flush;
    pcl::io::savePCDFile ("cloud_bin.pcd", *cloud_prt, true);
  }
  else if (format == KinFuLSApp::PCD_ASCII)
  {
    cout << "Saving point cloud to 'cloud.pcd' (ASCII)... " << flush;
    pcl::io::savePCDFile ("cloud.pcd", *cloud_prt, false);
  }
  else   /* if (format == KinFuLSApp::PLY) */
  {
    cout << "Saving point cloud to 'cloud.ply' (ASCII)... " << flush;
    pcl::io::savePLYFileASCII ("cloud.ply", *cloud_prt);

  }
  cout << "Done" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void
writePolygonMeshFile (int format, const pcl::PolygonMesh& mesh)
{
  if (format == KinFuLSApp::MESH_PLY)
  {
    cout << "Saving mesh to to 'mesh.ply'... " << flush;
    pcl::io::savePLYFile("mesh.ply", mesh);   
  }
  else /* if (format == KinFuLSApp::MESH_VTK) */
  {
    cout << "Saving mesh to to 'mesh.vtk'... " << flush;
    //pcl::io::saveVTKFile("mesh.vtk", mesh);    
  }  
  cout << "Done" << endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  int
print_cli_help ()
{
  cout << "\nKinFu parameters:" << endl;
  cout << "    --help, -h                          : print this message" << endl;  
  cout << "    --registration, -r                  : try to enable registration (source needs to support this)" << endl;
  cout << "    --current-cloud, -cc                : show current frame cloud" << endl;
  cout << "    --save-views, -sv                   : accumulate scene view and save in the end ( Requires OpenCV. Will cause 'bad_alloc' after some time )" << endl;  
  cout << "    --registration, -r                  : enable registration mode" << endl; 
  cout << "    --integrate-colors, -ic             : enable color integration mode (allows to get cloud with colors)" << endl;
  cout << "    --extract-textures, -et             : extract RGB PNG images to KinFuSnapshots folder." << endl;
  cout << "    --volume_size <in_meters>, -vs      : define integration volume size" << endl;
  cout << "    --shifting_distance <in_meters>, -sd : define shifting threshold (distance target-point / cube center)" << endl;
  cout << "    --snapshot_rate <X_frames>, -sr     : Extract RGB textures every <X_frames>. Default: 45  " << endl;
  cout << endl << "";
  cout << "Valid depth data sources:" << endl; 
  cout << "    -dev <device> (default), -oni <oni_file>, -pcd <pcd_file or directory>" << endl;
  cout << endl << "";

  return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  int
main (int argc, char* argv[])
{  
  if (pc::find_switch (argc, argv, "--help") || pc::find_switch (argc, argv, "-h"))
    return print_cli_help ();

  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  //  if (checkIfPreFermiGPU(device))
  //    return cout << endl << "Kinfu is supported only for Fermi and Kepler arhitectures. It is not even compiled for pre-Fermi by default. Exiting..." << endl, 1;

  boost::shared_ptr<pcl::Grabber> capture;
  bool triggered_capture = false;
  bool pcd_input = false;

  std::string match_file, openni_device, oni_file, pcd_dir;
  try
  {    
    if (pc::parse_argument (argc, argv, "-pcd", pcd_dir) > 0)
    {
      float fps_pcd = 15.0f;
      pc::parse_argument (argc, argv, "-pcd_fps", fps_pcd);

      vector<string> pcd_files = getPcdFilesInDir(pcd_dir);    
      // Sort the read files by name
      sort (pcd_files.begin (), pcd_files.end ());
      capture.reset (new pcl::PCDGrabber<pcl::PointXYZRGBA> (pcd_files, fps_pcd, false));
      triggered_capture = true;
      pcd_input = true;
    }
    else
    {
      cout<<"No caputure device specified!"<<endl;
      return -1;
    }
  }
  catch (const pcl::PCLException& /*e*/) { return cout << "Can't open depth source" << endl, -1; }

  float volume_size = pcl::device::kinfuLS::VOLUME_SIZE;
  pc::parse_argument (argc, argv, "--volume_size", volume_size);
  pc::parse_argument (argc, argv, "-vs", volume_size);

  float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
  pc::parse_argument (argc, argv, "--shifting_distance", shift_distance);
  pc::parse_argument (argc, argv, "-sd", shift_distance);

  int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE; // defined in internal.h
  pc::parse_argument (argc, argv, "--snapshot_rate", snapshot_rate);
  pc::parse_argument (argc, argv, "-sr", snapshot_rate);

  KinFuLSApp app (*capture, volume_size, shift_distance, snapshot_rate);

  if (pc::find_switch (argc, argv, "--registration") || pc::find_switch (argc, argv, "-r"))  {
    if (pcd_input) {
      app.pcd_source_   = true;
      app.registration_ = true; // since pcd provides registered rgbd
    }
  }

  if (pc::find_switch (argc, argv, "--integrate-colors") || pc::find_switch (argc, argv, "-ic"))      
    app.toggleColorIntegration();

  if (pc::find_switch (argc, argv, "--extract-textures") || pc::find_switch (argc, argv, "-et"))      
    app.enable_texture_extraction_ = true;

  // executing
  if (triggered_capture) 
    std::cout << "Capture mode: triggered\n";
  else             
    std::cout << "Capture mode: stream\n";

  // set verbosity level
  pcl::console::setVerbosityLevel(pcl::console::L_VERBOSE);
  try { app.startMainLoop (triggered_capture); }  
  catch (const pcl::PCLException& /*e*/) { cout << "PCLException" << endl; }
  catch (const std::bad_alloc& /*e*/) { cout << "Bad alloc" << endl; }
  catch (const std::exception& /*e*/) { cout << "Exception" << endl; }

  std::cout << "pcl_kinfu_largeScale exiting...\n";
  return 0;
}

