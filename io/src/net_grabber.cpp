/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <pcl/pcl_config.h>

#include <pcl/io/net_grabber.h>
#include <pcl/io/openni2/openni2_video_mode.h>
#include <iostream>
#include <pcl/exceptions.h>
#include <exception>
#include <boost/make_shared.hpp>

//#include <boost/pointer_cast.hpp>

using std::cout;
using std::endl;

namespace pcl
{
  NetGrabber::NetGrabber(int port, const string& severAddress)
    : device_()
    , mode_(Client)
    , port_(port)
    , serverAddress_(severAddress)
    , netVars_()
    , clientSocket_()
    , point_cloud_signal_(), point_cloud_i_signal_(), point_cloud_rgb_signal_(), point_cloud_rgba_signal_(), point_cloud_vertices_signal_()
    , depth_image_signal_()
    , running_(false)
    , compressionParameter_(-1)
    , count_(0)
    , useGrabber_(false)
    , readClient_(false)
  {
    point_cloud_signal_ = createSignal<sig_cb_point_cloud>();
    point_cloud_rgb_signal_ = createSignal<sig_cb_point_cloud_rgb>();
    point_cloud_i_signal_ = createSignal<sig_cb_point_cloud_i>();
    point_cloud_rgba_signal_ = createSignal<sig_cb_point_cloud_rgba>();
    depth_image_signal_ = createSignal<sig_cb_depth_image>();
    point_cloud_vertices_signal_ = createSignal<sig_cb_vertices>();

    onInit(port, severAddress);
  }

  NetGrabber::NetGrabber(const boost::shared_ptr<Grabber> grabber, int port, const string& severAddress)
    : device_(grabber)
    , mode_(Client)
    , port_(port)
    , serverAddress_(severAddress)
    , netVars_()
    , clientSocket_()
    , point_cloud_signal_(), point_cloud_i_signal_(), point_cloud_rgb_signal_(), point_cloud_rgba_signal_(), point_cloud_vertices_signal_()
    , depth_image_signal_()
    , running_(false)
    , compressionParameter_(-1)
    , count_(0)
    , useGrabber_(true)
    , readClient_(false)
  {
    point_cloud_signal_ = createSignal<sig_cb_point_cloud>();
    point_cloud_rgb_signal_ = createSignal<sig_cb_point_cloud_rgb>();
    point_cloud_i_signal_ = createSignal<sig_cb_point_cloud_i>();
    point_cloud_rgba_signal_ = createSignal<sig_cb_point_cloud_rgba>();
    depth_image_signal_ = createSignal<sig_cb_depth_image>();
    point_cloud_vertices_signal_ = createSignal<sig_cb_vertices>();

    onInit(port, severAddress);

#ifdef HAVE_OPENNI
    OpenNIGrabber* openni_grabber = dynamic_cast<OpenNIGrabber*> (grabber.get());
    if (openni_grabber)
    {
      boost::function<void(
        const Image::Ptr &image,
        const DepthImage::Ptr &depth_image,
        float focalLength
        )> f =
        boost::bind(&NetGrabber::depth_image_cb_, this, _1, _2, _3);

      grabber->registerCallback(f);
    }
#endif

#ifdef HAVE_OPENNI2
    OpenNIGrabber* openni_grabber = dynamic_cast<OpenNIGrabber*> (grabber.get());
    if (openni_grabber)
    {
      boost::function<void(
        const Image::Ptr &image,
        const DepthImage::Ptr &depth_image,
        float focalLength
        )> f =
        boost::bind(&NetGrabber::depth_image_cb_, this, _1, _2, _3);

      grabber->registerCallback(f);
    }
#endif
  }

  pcl::NetGrabber::NetGrabber(int port, bool readClient)
    : device_()
    , mode_(Server)
    , port_(port)
    , serverAddress_()
    , netVars_()
    , clientSocket_()
    , point_cloud_signal_(), point_cloud_i_signal_(), point_cloud_rgb_signal_(), point_cloud_rgba_signal_(), point_cloud_vertices_signal_()
    , depth_image_signal_()
    , running_(false)
    , compressionParameter_(-1)
    , count_(0)
    , useGrabber_(false)
    , readClient_(readClient)
  {
    point_cloud_signal_ = createSignal<sig_cb_point_cloud>();
    point_cloud_rgb_signal_ = createSignal<sig_cb_point_cloud_rgb>();
    point_cloud_i_signal_ = createSignal<sig_cb_point_cloud_i>();
    point_cloud_rgba_signal_ = createSignal<sig_cb_point_cloud_rgba>();
    depth_image_signal_ = createSignal<sig_cb_depth_image>();
    point_cloud_vertices_signal_ = createSignal<sig_cb_vertices>();

    onInit(port);
  }

  //TODO: cleanup here
  NetGrabber::~NetGrabber() throw (){}

  void NetGrabber::setCompressionParameter(int parameter)
  {
    compressionParameter_ = parameter;
  }

  int NetGrabber::getCompressionParameter() const
  {
    return compressionParameter_;
  }

  void NetGrabber::onInit(int port, const string& serverAddress)
  {
    netVars_ = boost::shared_ptr<NetGrabber::NetworkParameters>(new NetGrabber::NetworkParameters());
    netVars_->io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
    netVars_->resolver = boost::shared_ptr<tcp::resolver>(new tcp::resolver(*(netVars_->io_service)));

    std::string port_s = std::to_string(port);
    netVars_->query = boost::shared_ptr<tcp::resolver::query>(new tcp::resolver::query(serverAddress, port_s));
    netVars_->socket = boost::shared_ptr<tcp::socket>(new tcp::socket(*netVars_->io_service));
    netVars_->resolver->async_resolve(*netVars_->query, boost::bind(&NetGrabber::clientResolveHandler, this, _1, _2));
  }

  void NetGrabber::onInit(int port)
  {
    netVars_ = boost::shared_ptr<NetGrabber::NetworkParameters>(new NetGrabber::NetworkParameters());
    netVars_->io_service = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
    netVars_->endpoint = boost::shared_ptr<tcp::endpoint>(new tcp::endpoint(tcp::v4(), port));
    netVars_->acceptor = boost::shared_ptr<tcp::acceptor>(new tcp::acceptor(*netVars_->io_service, *netVars_->endpoint));
    netVars_->socket = boost::shared_ptr<tcp::socket>(new tcp::socket(*netVars_->io_service));
  }

  PointCloud<pcl::PointXYZ>::Ptr
    NetGrabber::convertToXYZPointCloud
    (
    const OpenNICameraParameters & camSettings,
    const vector<unsigned short>& depthBuffer,
    int frameId
    ) const
  {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);

      cloud->height = camSettings.height;
      cloud->width = camSettings.width;
      cloud->is_dense = false;

      cloud->points.resize(cloud->height * cloud->width);

      float constant_x = 1.0f / camSettings.depthFocalLengthX;
      float constant_y = 1.0f / camSettings.depthFocalLengthY;
      float centerX = ((float) cloud->width - 1.f) / 2.f;
      float centerY = ((float) cloud->height - 1.f) / 2.f;

      if (pcl_isfinite(camSettings.focalLengthX))
        constant_x = 1.0f / static_cast<float> (camSettings.focalLengthX);

      if (pcl_isfinite(camSettings.focalLengthY))
        constant_y = 1.0f / static_cast<float> (camSettings.focalLengthY);

      if (pcl_isfinite(camSettings.principalPointX))
        centerX = static_cast<float> (camSettings.principalPointX);

      if (pcl_isfinite(camSettings.principalPointY))
        centerY = static_cast<float> (camSettings.principalPointY);

      //TODO if depth not registered, use rgb frame id
      cloud->header.frame_id = frameId;

      float bad_point = std::numeric_limits<float>::quiet_NaN();

      int depth_idx = 0;
      for (int v = 0; v < camSettings.height; ++v)
      {
        for (register int u = 0; u < camSettings.width; ++u, ++depth_idx)
        {
          pcl::PointXYZ& pt = cloud->points[depth_idx];
          // Check for invalid measurements
          if (depthBuffer[depth_idx] == 0 ||
            depthBuffer[depth_idx] == camSettings.noSampleValue ||
            depthBuffer[depth_idx] == camSettings.shadowValue)
          {
            // not valid
            pt.x = pt.y = pt.z = bad_point;
            continue;
          }
          pt.z = depthBuffer[depth_idx] * 0.001f;
          pt.x = (static_cast<float> (u) -centerX) * pt.z * constant_x;
          pt.y = (static_cast<float> (v) -centerY) * pt.z * constant_y;
        }
      }

      cloud->sensor_origin_.setZero();
      cloud->sensor_orientation_.w() = 1.0f;
      cloud->sensor_orientation_.x() = 0.0f;
      cloud->sensor_orientation_.y() = 0.0f;
      cloud->sensor_orientation_.z() = 0.0f;
      return (cloud);
  }

  void NetGrabber::start()
  {
    if (mode_ == NetGrabber::Client)
    {
      ioThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&NetGrabber::runIOService, this)));

      if (useGrabber_ && !device_->isRunning())
      {
        device_->start();
      }
    }
    else if (mode_ == NetGrabber::Server)
    {
      ioThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&NetGrabber::serverLoop, this)));
    }

    running_ = true;
  }

  void NetGrabber::sendToClient(std::vector<pcl::Vertices> &verts)
  {
    unsigned compressedLength;
    vector<unsigned char> compressed;
    int level = -1;
    unsigned headerSize;
    
    this->fringeCompression_.encodeData
      (
      &verts[0],
      sizeof(pcl::Vertices),
      verts.size(),
      NetGrabber::VerticesData,
      headerSize,
      compressedLength,
      compressed,
      level
      );
      
    unsigned sizeBytes = compressedLength * sizeof(unsigned char) +headerSize;
    
      boost::asio::write
      (
      *netVars_->socket,
      boost::asio::buffer(compressed, sizeBytes)
      );
  }

  void NetGrabber::sendToClient(pcl::PointCloud<PointXYZRGB> &cloud)
  {
    unsigned compressedLength;
    vector<unsigned char> compressed;
    int level = -1;
    unsigned headerSize;

    this->fringeCompression_.encodeData
      (
      &cloud.points[0],
      sizeof(PointXYZRGB),
      cloud.points.size(),
      NetGrabber::PointCloudData,
      headerSize,
      compressedLength,
      compressed, 
      level
      );

    unsigned sizeBytes = compressedLength * sizeof(unsigned char) + headerSize;

    try
    {
      boost::asio::write
        (
        *netVars_->socket,
        boost::asio::buffer(compressed, sizeBytes)
        );
    }
    catch (boost::exception &e)
    {
      cout << "Got boost exception: " << boost::diagnostic_information(e) << endl;
    }
    catch (...)
    {
      cout << "Caught some other exception" << endl;
    }
  }

  void NetGrabber::clientReadHandler(const boost::system::error_code &ec, std::size_t bytesTransferred)
  {
    if (!ec)
    {
      std::cout << "Read size: " << clientReadLengthBuffer_[0] << " header size: " << clientReadLengthBuffer_[1]<< std::endl;
      
      unsigned compressedElements = clientReadLengthBuffer_[0];
      unsigned headerElements = clientReadLengthBuffer_[1];
      unsigned toRead = compressedElements + headerElements*sizeof(unsigned) / sizeof(char);

      clientReadDataBuffer_.resize(toRead);
      netVars_->socket->read_some(boost::asio::buffer(clientReadDataBuffer_, sizeof(char) *toRead));
      
      unsigned dataId;
      unsigned decompressedLength;
      vector<unsigned char> decompressed;      

      fringeCompression_.decodeData
        (
        &clientReadDataBuffer_[0],
        sizeof(unsigned char),
        compressedElements,
        dataId,
        decompressedLength,
        decompressed
        );

      if (dataId == NetGrabber::PointCloudData)
      {
        cout << "Got PCD with decom length: " << decompressedLength << endl;
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointXYZRGB *dataPoints = (pcl::PointXYZRGB *)decompressed.data();
        int count = decompressed.size()*sizeof(unsigned char) / sizeof(pcl::PointXYZRGB);
        cloud->width = count;
        cloud->height = 1;
        cloud->points.resize(count);
        memcpy(cloud->points.data(), dataPoints, count*sizeof(pcl::PointXYZRGB));

        int numCloudSlots = point_cloud_rgb_signal_->num_slots();
        if (numCloudSlots > 0)
        {
          point_cloud_rgb_signal_->operator ()
            (
            cloud
            );
        }
      }
      else if (dataId == NetGrabber::VerticesData)
      {
        cout << "Got vertices data with decom length: " << decompressedLength << endl;
        
        pcl::Vertices *dataPoints = (pcl::Vertices *)decompressed.data();
        int count = decompressed.size()*sizeof(unsigned char) / sizeof(pcl::Vertices);
        
        boost::shared_ptr<vector<pcl::Vertices> > vertices(new vector<pcl::Vertices>);
        for (int i = 0; i<count; i++)
        {
          vertices->push_back(dataPoints[i]);
        }

        //boost::shared_ptr<vector<pcl::Vertices> > vertices(new vector<pcl::Vertices>(dataPoints, dataPoints+count));
        
        int numCloudSlots = point_cloud_vertices_signal_->num_slots();
        if (numCloudSlots > 0)
        {
          point_cloud_vertices_signal_->operator ()
            (
            vertices
            );
        }
      }
    
      clientConnectHandler(ec);
    }
    else
      printf("error reading\n");
  }

  void NetGrabber::clientConnectHandler(const boost::system::error_code &ec)
  {
    if (!ec)
    {
      int headerSize = 2;
      connectedToServer_ = true;
      printf("client connected HERE !\n");
      clientReadLengthBuffer_.resize(headerSize);
      
      netVars_->socket->async_read_some
        (
        boost::asio::buffer(clientReadLengthBuffer_, headerSize*sizeof(unsigned)),
        boost::bind(&NetGrabber::clientReadHandler, this, _1, _2)
        );
      printf("done async read!");
    }
    else
    {
      connectedToServer_ = false;
      printf("error connecting\n");
    }
  }

  void NetGrabber::clientResolveHandler(const boost::system::error_code &ec, tcp::resolver::iterator it)
  {
    if (!ec)
    {
      printf("Done resolving Trying to connect to server\n");
      netVars_->socket->async_connect(*it, boost::bind(&NetGrabber::clientConnectHandler, this, _1));
      //netVars_->socket->connect(*it);

    }
    else
      printf("error resolving\n");
  }

  void NetGrabber::runIOService()
  {
    netVars_->io_service->run();;
  }

  int NetGrabber::serverLoop()
  {
    waitForClient(netVars_);
    printf("Done waiting for client!\n");
    int frameId = 0;
    while (running_ && readClient_)
    {
      unsigned compressedLength;
      std::vector<unsigned char> compressed;
      int error = serverReadOnce(netVars_->socket, compressedLength, cameraParameters_, compressed);
      if (error != 0)
        return error;
      cout << "Compressed length: " << compressedLength << endl;

      vector<unsigned short> decompressed;
      fringeCompression_.decodePixels(compressedLength, cameraParameters_, compressed, decompressed);

      int numPointCloudSlots = point_cloud_signal_->num_slots();
      if (numPointCloudSlots > 0)
      {
        PointCloud<pcl::PointXYZ>::Ptr pointCloud = convertToXYZPointCloud(cameraParameters_, decompressed, frameId);
        point_cloud_signal_->operator ()(pointCloud);
      }

      int numDepthSlots = depth_image_signal_->num_slots();
      if (numDepthSlots > 0)
      {
          depth_image_signal_->operator ()
          (
          boost::make_shared<const vector<unsigned short> > (decompressed),
          boost::make_shared<const pcl::OpenNICameraParameters> (cameraParameters_)
          );
      }

      frameId++;
    }

    return 0;
  }

  int NetGrabber::serverReadOnce
    (
    boost::shared_ptr<tcp::socket> socket,
    unsigned& compressedLength,
    OpenNICameraParameters & sensorParams,
    std::vector<unsigned char> & dataBuffer
    )
  {
    boost::array<unsigned, 1> compressedLengthBuffer;
    boost::array<OpenNICameraParameters, 1> sensorParamsBuffer;
    boost::system::error_code error;

    size_t len = socket->read_some(boost::asio::buffer(compressedLengthBuffer), error);
    compressedLength = compressedLengthBuffer[0];
    if (error == boost::asio::error::eof)
      return 1; // Connection closed cleanly by peer.
    else if (error)
      throw boost::system::system_error(error); // Some other error.
    cout << "Just read in: " << len << " bytes" << endl;

    len = socket->read_some(boost::asio::buffer(sensorParamsBuffer), error);
    sensorParams = sensorParamsBuffer[0];
    if (error == boost::asio::error::eof)
      return 1; // Connection closed cleanly by peer.
    else if (error)
      throw boost::system::system_error(error); // Some other error.
    cout << "Just read in2: " << len << " bytes" << endl;

    dataBuffer.resize(compressedLength);
    len = socket->read_some(boost::asio::buffer(dataBuffer), error);
    if (error == boost::asio::error::eof)
      return 1; // Connection closed cleanly by peer.
    else if (error)
      throw boost::system::system_error(error); // Some other error.
    cout << "Just read in3: " << len << " bytes" << endl;
    return 0;
  }


  void NetGrabber::stop()
  {
    running_ = false;

    if (mode_ == NetGrabber::Client)
    {
      if (device_->isRunning())
      {
        device_->stop();
      }

      netVars_->socket->close();
    }
    else
    {
      //TODO: close server;
    }
  }

  bool NetGrabber::isRunning() const
  {
    return running_;
  }

  string NetGrabber::getName() const
  {
    return "NetGrabber";
  }

  //TODO
  void NetGrabber::signalsChanged()
  {

  }

  void NetGrabber::waitForClient(boost::shared_ptr<NetGrabber::NetworkParameters> server)
  {
    try
    {
      server->acceptor->accept(*(server->socket));
    }
    catch (boost::system::system_error e)
    {
      cout << "Error on accepting!" << endl;
    }
    
  }

  void NetGrabber::sendCompressedBuffer
    (
    boost::shared_ptr<tcp::socket> socket,
    vector<int>& buffer,
    unsigned compressedLength
    )
  {
    if (connectedToServer_ && compressedLength > 0)
    {
      boost::asio::write(*socket, boost::asio::buffer(buffer, compressedLength * sizeof(int)));
    }
  }

  void NetGrabber::sendCompressedBuffer
    (
    boost::shared_ptr<tcp::socket> socket,
    vector<unsigned char>& buffer,
    unsigned compressedLength
    )
  {
    if (connectedToServer_ && compressedLength > 0)
    {
      boost::asio::write(*socket, boost::asio::buffer(buffer, compressedLength * sizeof (unsigned char)));
    }
  }

#if defined HAVE_OPENNI || defined HAVE_OPENNI2
    void NetGrabber::depth_image_cb_
    (
    const boost::shared_ptr<Image>& rgbImage,
    const boost::shared_ptr<DepthImage>& depthImage,
    float focalLength
    )
    {
      
      cout << "Depth image CB!!!!" << endl;
      if (count_ % 1 == 0)
      {
        OpenNICameraParameters parameters;
        getOpenNICameraParameters(parameters, boost::dynamic_pointer_cast<OpenNIGrabber>(device_), depthImage);

        unsigned compressedLength;
        vector<unsigned char> compressed;
        encodeDepthImage
          (
          depthImage,
          compressionParameter_,
          parameters,
          compressedLength,
          compressed
          );

        sendCompressedBuffer
          (
          netVars_->socket,
          compressed,
          compressedLength
          );
      }

      count_++;
      cout << "DONE WITH Depth image CB!!!!" << endl;
  }

  void NetGrabber::getOpenNICameraParameters
    (
    OpenNICameraParameters& parameters,
    boost::shared_ptr<OpenNIGrabber> openNIGrabber,
    const boost::shared_ptr<DepthImage>& depthImage
    )
  {
    boost::shared_ptr<OpenNIDevice> openNIDevice = openNIGrabber->getDevice();
#ifdef HAVE_OPENNI
    openNIGrabber->getImageDimentions(parameters.width, parameters.height);
    parameters.depthFocalLengthX = openNIDevice->getDepthFocalLength(parameters.width);
    parameters.depthFocalLengthY = openNIDevice->getDepthFocalLength(parameters.height);
#endif

#ifdef HAVE_OPENNI2
    const pcl::io::openni2::OpenNI2VideoMode videoMode = openNIDevice->getDepthVideoMode();
    parameters.width = videoMode.x_resolution_;
    parameters.height = videoMode.y_resolution_;
    parameters.depthFocalLengthX = openNIDevice->getDepthFocalLength();
    parameters.depthFocalLengthY = openNIDevice->getDepthFocalLength();
#endif
    
    parameters.noSampleValue = depthImage->getNoSampleValue();
    parameters.shadowValue = depthImage->getShadowValue();
    openNIGrabber->getDepthCameraIntrinsics(parameters.focalLengthX, parameters.focalLengthY, parameters.principalPointX, parameters.principalPointY);
    parameters.fps = openNIGrabber->getFramesPerSecond();

  }

  float NetGrabber::getFramesPerSecond() const
  {
    return device_->getFramesPerSecond();
  }

  void NetGrabber::encodeDepthImage
    (
    const boost::shared_ptr<DepthImage>& depthImage,
    int encodeParam,
    const OpenNICameraParameters & sensorParameters,
    unsigned & compressedLength,
    vector<unsigned char>& compressed
    )
  {
#ifndef HAVE_OPENNI2
    const xn::DepthMetaData & metaData = depthImage->getDepthMetaData();
    const XnDepthMetaData* xnMetaData = metaData.GetUnderlying();
    const unsigned short *pixels = xnMetaData->pData;

#else
    const unsigned short *pixels = (const unsigned short*) depthImage->getMetaData()->getData();
#endif

    fringeCompression_.encodePixels(pixels, encodeParam, sensorParameters, compressedLength, compressed);
  }
#endif //HAVE_OPENNI || HAVE_OPENNI2
}
