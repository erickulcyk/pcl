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
#include <string>
#include <vector>
//#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/common/synchronizer.h>

#include <pcl/io/openni_camera_parameters.h>
#include <pcl/compression/fringe_compression.h>

#ifndef HAVE_OPENNI2
#include <pcl/io/openni_grabber.h>
#else
#ifdef HAVE_OPENNI
#include <pcl/io/openni2_grabber.h>
#endif
#endif

using std::string;
using std::vector;
using boost::asio::ip::tcp;

namespace pcl
{
	class PCL_EXPORTS NetGrabber : public Grabber
	{
	public:
		typedef boost::shared_ptr<NetGrabber> Ptr;
		typedef boost::shared_ptr<const NetGrabber> ConstPtr;

		typedef enum
		{
			Client = 0,
			Server = 1
		} Mode;

		//define callback signature typedefs
		typedef void (sig_cb_openni_point_cloud)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&);
		typedef void (sig_cb_openni_point_cloud_rgb)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&);
		typedef void (sig_cb_openni_point_cloud_rgba)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&);
		typedef void (sig_cb_openni_point_cloud_i)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI> >&);

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

		/** \brief Constructor for server configuration
		 *  \param[in] the port to listen on.
		 */
		NetGrabber(int port);

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

		/** \brief Obtain the number of frames per second (FPS). */
		inline OpenNICameraParameters
			getCameraParameters() const { return cameraParameters_; };

	protected:
		struct ServerParameters
		{
			boost::shared_ptr<boost::asio::io_service> io_service;
			boost::shared_ptr<tcp::endpoint> endpoint;
			boost::shared_ptr<tcp::socket> socket;
			boost::shared_ptr<tcp::acceptor> acceptor;
		};

		/** \brief Sets up the client to talk to a sever at a given address on a given port. */
		void
			onInit(boost::shared_ptr<Grabber> grabber, int port, const string& severAddress);

		/** \brief Sets up the sever on a given port. */
		void
			onInit(int port);

		/** \brief Process changed signals. */
		virtual void
			signalsChanged();

		// helper methods

		void waitForClient(boost::shared_ptr<NetGrabber::ServerParameters> server);

#if defined HAVE_OPENNI || defined HAVE_OPENNI2
		void getOpenNICameraParameters(OpenNICameraParameters& parameters, shared_ptr<OpenNIGrabber> openNIGrabber);

		void encodeDepthImage
			(
			const boost::shared_ptr<openni_wrapper::DepthImage>& depthImage,
			int encodeParam,
			const OpenNICameraParameters & sensorParameters,
			unsigned & compressedLength,
			vector<unsigned char>& compressed
			);
#endif
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			decodeXYZPointCloud
			(
			unsigned compressedLength,
			OpenNICameraParameters& parameters,
			vector<unsigned char>& compressed
			);

		void sendCompressedBuffer
			(
			boost::shared_ptr<tcp::socket> socket,
			vector<unsigned char>& buffer,
			unsigned compressedLength
			);

		/** \brief Convert a Depth image to a pcl::PointCloud<pcl::PointXYZ>
		  * \param[in] depth the depth image to convert
		  */
		pcl::PointCloud<pcl::PointXYZ>::Ptr
			convertToXYZPointCloud(
			const OpenNICameraParameters & camSettings,
			const vector<unsigned short> & depthBuffer,
			int frameId) const;

		int serverReadOnce
			(
			boost::shared_ptr<tcp::socket> socket,
			unsigned& compressedLength,
			OpenNICameraParameters & sensorParams,
			std::vector<unsigned char> & dataBuffer
			);

		int serverLoop();

		/** \brief Convert a Depth + RGB image pair to a pcl::PointCloud<PointT>
		  * \param[in] image the RGB image to convert
		  * \param[in] depth_image the depth image to convert
		  */
		/*
  template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
  convertToXYZRGBPointCloud (const boost::shared_ptr<openni_wrapper::Image> &image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;
  */
		/** \brief Convert a Depth + Intensity image pair to a pcl::PointCloud<pcl::PointXYZI>
		  * \param[in] image the IR image to convert
		  * \param[in] depth_image the depth image to convert
		  */
		/*
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> >
  convertToXYZIPointCloud (const boost::shared_ptr<openni_wrapper::IRImage> &image,
  const boost::shared_ptr<openni_wrapper::DepthImage> &depth_image) const;
  */
		/** \brief The actual grabber. */
		boost::shared_ptr<Grabber> device_;

		boost::signals2::signal<sig_cb_openni_point_cloud>* point_cloud_signal_;
		boost::signals2::signal<sig_cb_openni_point_cloud_i>* point_cloud_i_signal_;
		boost::signals2::signal<sig_cb_openni_point_cloud_rgb>* point_cloud_rgb_signal_;
		boost::signals2::signal<sig_cb_openni_point_cloud_rgba>* point_cloud_rgba_signal_;

		bool running_;

		int port_;

		Mode mode_;

		string serverAddress_;

		boost::shared_ptr<ServerParameters> server_;

		boost::shared_ptr<tcp::socket> clientSocket_;

		FringeCompression fringeCompression_;

		boost::shared_ptr<boost::thread> serverThread_;

		OpenNICameraParameters cameraParameters_;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
} // namespace pcl
#endif // __PCL_IO_NET_GRABBER__