/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2011, Willow Garage, Inc.
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

#ifndef __PCL_IO_NET_GRABBER__
#define __PCL_IO_NET_GRABBER__

//#include <pcl/io/eigen.h>
#include <pcl/io/boost.h>
#include <boost/asio.hpp>
#include <pcl/io/grabber.h>
#include <cstddef>
#include <string>
#include <vector>
//#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
//#include <pcl/common/synchronizer.h>

#include <pcl/io/openni_camera_parameters.h>
#include <pcl/compression/fringe_compression.h>
#include <pcl/io/net_state.h>

#ifdef HAVE_OPENNI2
#include <pcl/io/openni2/openni2_device.h>
#include <pcl/io/openni2_grabber.h>
#endif

#ifdef HAVE_OPENNI
#include <pcl/io/openni_grabber.h>
#endif

using std::string;
using std::vector;
using boost::asio::ip::tcp;

namespace pcl
{
  class PCL_EXPORTS NetGrabber : public Grabber, public std::enable_shared_from_this<NetGrabber>
  {
  public:
    typedef boost::shared_ptr<NetGrabber> Ptr;
    typedef boost::shared_ptr<const NetGrabber> ConstPtr;

    typedef enum
    {
      Client = 0,
      Server = 1
    } Mode;

    typedef enum
    {
      PointCloudData,
      VerticesData
    } DataId;

    //define callback signature typedefs
    typedef void (sig_cb_point_cloud) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
    typedef void (sig_cb_point_cloud_rgb) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
    typedef void (sig_cb_point_cloud_rgba) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
    typedef void (sig_cb_point_cloud_i) (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);
    typedef void (sig_cb_depth_image)
      (
      const boost::shared_ptr<const vector<unsigned short> >&,
      const boost::shared_ptr<const OpenNICameraParameters>&
      );
    typedef void (sig_cb_vertices)(const boost::shared_ptr<vector<pcl::Vertices> >&);

  public:
    /** \brief Constructor for client configuration
     *  \param[in] grabber The original grabber used to capture depth images
     *  \param[in] port The port number of the server
     *  \param[in] severAddress The ip address of the server
     */
    NetGrabber(boost::shared_ptr<Grabber> grabber,
      int port,
      const string& severAddress = ""
      );

    NetGrabber(int port, const string& severAddress);

    /** \brief Constructor for server configuration
     *  \param[in] the port to listen on.
     */
    NetGrabber(int port, bool readClient);

    /** \brief virtual Destructor inherited from the Grabber interface. It never throws. */
    virtual ~NetGrabber() throw ();

    /** \brief Start the data acquisition. */
    virtual void
      start();

    /** \brief Stop the data acquisition. */
    virtual void
      stop();

    /** \brief Check if the data acquisition is still running. */
    virtual bool
      isRunning() const;

    virtual std::string
      getName() const;

    virtual float
      getFramesPerSecond() const;

    /** \brief Obtain the number of frames per second (FPS). */
    inline OpenNICameraParameters
      getCameraParameters() const { return cameraParameters_; };

    int getCompressionParameter() const;

    void setCompressionParameter(int compressionParameter);

    void NetGrabber::sendToClient(std::vector<pcl::Vertices> &verts);
    void NetGrabber::sendToClient(pcl::PointCloud<PointXYZRGB> &cloud);

    inline bool
      isConnectedToServer() { return connectedToServer_; }

  protected:

    /** \brief Sets up the client to talk to a sever at a given address on a given port. */
    void
      onInit(int port, const string& severAddress);

    /** \brief Sets up the sever on a given port. */
    void
      onInit(int port);

    /** \brief Process changed signals. */
    virtual void
      signalsChanged();

    // helper methods

    void waitForClient(boost::shared_ptr<NetState> server);

    void runIOService();

#ifdef HAVE_OPENNI
    typedef openni_wrapper::DepthImage DepthImage;
    typedef openni_wrapper::IRImage IRImage;
    typedef openni_wrapper::Image Image;

    typedef pcl::io::OpenNIGrabber OpenNIGrabber;
    typedef openni_wrapper::OpenNIDevice OpenNIDevice;
#endif

#ifdef HAVE_OPENNI2
    typedef pcl::io::DepthImage DepthImage;
    typedef pcl::io::IRImage IRImage;
    typedef pcl::io::Image Image;

    typedef pcl::io::OpenNI2Grabber OpenNIGrabber;
    typedef pcl::io::openni2::OpenNI2Device OpenNIDevice;
#endif

#if defined HAVE_OPENNI || defined HAVE_OPENNI2
    void depth_image_cb_
      (
      const boost::shared_ptr<Image>& rgbImage,
      const boost::shared_ptr<DepthImage>& depthImage,
      float focalLength
      );

    void getOpenNICameraParameters
      (
      OpenNICameraParameters& parameters,
      boost::shared_ptr<OpenNIGrabber> openNIGrabber,
      const boost::shared_ptr<DepthImage>& depthImage
      );

    void encodeDepthImage
      (
      const boost::shared_ptr<DepthImage>& depthImage,
      int encodeParam,
      const OpenNICameraParameters & sensorParameters,
      unsigned & compressedLength,
      vector<unsigned char>& compressed
      );
#endif

    void sendCompressedBuffer
      (
      tcp::socket& socket,
      vector<unsigned char>& buffer,
      unsigned compressedLength
      );

    void NetGrabber::sendCompressedBuffer
      (
      tcp::socket& socket,
      vector<int>& buffer,
      unsigned compressedLength
      );

    /** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
      * \param[in] depth the depth image to convert
      */
    pcl::PointCloud<pcl::PointXYZ>::Ptr
      convertToXYZPointCloud
      (
      const OpenNICameraParameters & camSettings,
      const vector<unsigned short> & depthBuffer,
      int frameId
      ) const;

    int serverReadOnce
      (
      const boost::shared_ptr<tcp::socket>& socket//,
      /*unsigned& compressedLength,
      OpenNICameraParameters & sensorParams,
      std::vector<unsigned char> & dataBuffer*/
      );

    void NetGrabber::handleReadHeader
      (
      const boost::system::error_code& error,
      std::size_t bytes_transferred
      );

    void NetGrabber::handleReadSensorParams
      (
      const boost::system::error_code& error,
      std::size_t bytes_transferred
      );

    void NetGrabber::handleReadDataBuffer
      (
      const boost::system::error_code& error,
      std::size_t bytes_transferred
      );

    int serverLoop();

    void clientReadHandler(const boost::system::error_code &ec, std::size_t bytesTransferred);
    void clientResolveHandler(const boost::system::error_code &ec, tcp::resolver::iterator it);
    void clientConnectHandler(const boost::system::error_code &ec);

    vector<unsigned> clientReadLengthBuffer_;
    vector<char> clientReadDataBuffer_;
  
    /** \brief The actual grabber. */
    boost::shared_ptr<Grabber> device_;

    boost::signals2::signal<sig_cb_point_cloud>* point_cloud_signal_;
    boost::signals2::signal<sig_cb_point_cloud_i>* point_cloud_i_signal_;
    boost::signals2::signal<sig_cb_point_cloud_rgb>* point_cloud_rgb_signal_;
    boost::signals2::signal<sig_cb_point_cloud_rgba>* point_cloud_rgba_signal_;
    boost::signals2::signal<sig_cb_depth_image>* depth_image_signal_;
    boost::signals2::signal<sig_cb_vertices>* point_cloud_vertices_signal_;

    int compressionParameter_;

    bool running_;

    bool connectedToServer_;

    int port_;

    Mode mode_;

    string serverAddress_;

    boost::shared_ptr<NetState> netVars_;

    boost::shared_ptr<tcp::socket> clientSocket_;

    FringeCompression fringeCompression_;

    boost::shared_ptr<boost::thread> ioThread_;

    boost::shared_ptr<boost::thread> serverThread_;

    OpenNICameraParameters cameraParameters_;

    int count_;

    bool useGrabber_;

    bool readClient_;

    int frameId_;

    //OpenNiMessage messageBuffer_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace pcl
#endif // __PCL_IO_NET_GRABBER__