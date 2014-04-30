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

#ifndef __PCL_IO_FRINGE_COMPRESSION__
#define __PCL_IO_FRINGE_COMPRESSION__

#include <string>
#include <vector>

#include <pcl/io/openni_camera_parameters.h>

using std::string;
using std::vector;

namespace pcl
{
	class FringeCompression
	{
	public:
    FringeCompression(string lutFileName = "lut");

		void encodePixels
			(
			const unsigned short * pixels,
			int encodeParam,
			const OpenNICameraParameters & sensorParameters,
			unsigned & compressedLength,
			vector<unsigned char>& compressed
			);

		void decodePixels
			(
			unsigned compressedLength,
			OpenNICameraParameters parameters,
			vector<unsigned char>& compressed,
			vector<const unsigned short>& decompressed
			);

	protected:
		int getMeanValue
			(
			const unsigned short *pixels,
			int rows,
			int cols,
			int index,
			int b
			);

		inline void fillLutLocation
			(
			int* lookUpTable,
			int index,
			int compressedIndex,
			int lutCols
			);

		void writeLUT
			(
			string fileName,
			int* lut,
			int columns,
			int rows
			);

		void readLUT
			(
			string fileName,
			int* & lut,
			int & columns,
      int & rows
			);

		void printCompressedLength
			(
			int originalLength,
			int compressedLength,
			int pixelLength
			);

		unsigned short* generatePixelBuffer
			(
			const unsigned short *pixels,
			unsigned columns,
			unsigned rows,
			unsigned & dataLen,
			bool computeLUT
			);

		unsigned getHeaderSize();

    string lutFileName_;
		unsigned short* pixelCopy_;
		int *lut_;
		bool wroteLut_;
	};
}
#endif