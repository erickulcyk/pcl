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
#include <iostream>
#include <string>
#include <memory>

#include <pcl/compression/fringe_compression.h>
#include <pcl/compression/zpipe.h>
#include <pcl/io/openni_camera_parameters.h>


#include <boost/asio.hpp>
//#include <boost/thread/thread.hpp>

using std::cout;
using std::endl;
using boost::asio::ip::tcp;

namespace pcl
{
	FringeCompression::FringeCompression(string lutFileName)
	{
		pixelCopy_ = NULL;
		wroteLut_ = true;
    lut_ = NULL;
    pixelCopy_ = NULL;
    lutFileName_ = lutFileName;
	}

	unsigned FringeCompression::getHeaderSize()
	{
		return sizeof(unsigned) +sizeof(OpenNICameraParameters);
	}

	void FringeCompression::writeLUT
		(
		string fileName,
		int* lut,
		int columns,
		int rows
		)
	{
		FILE * pFile;
#ifdef _MSC_VER
		fopen_s(&pFile, fileName.c_str(), "wb");
#else
		pFile = fopen(fileName.c_str(), "wb");
#endif

		unsigned lutLength = rows*columns;
		int isize = sizeof(int);
		fwrite(&columns, isize, 1, pFile);
		fwrite(&rows, isize, 1, pFile);
		fwrite(lut, isize, lutLength, pFile);
		fclose(pFile);
		wroteLut_ = true;
	}


	void FringeCompression::encodePixels
		(
		const unsigned short * pixels,
		int encodeParam,
		const OpenNICameraParameters & sensorParameters,
		unsigned & compressedLength,
		vector<unsigned char>& compressed
		)
	{
		//To send floats, convert them to character arrays them add them to the main array.
		typedef unsigned char * byte_ptr;
		unsigned dataLen;//Length of the pixel buffer to compress.
		byte_ptr pixelChars = (byte_ptr) generatePixelBuffer(pixels, sensorParameters.width, sensorParameters.height, dataLen, !wroteLut_);
		if (pixelChars == NULL)
		{
			compressedLength = 0;
			return;
		}

		int headerSize = getHeaderSize();
		int compressedBufferLen = sensorParameters.width * sensorParameters.height * 2;
		if (compressed.size() < compressedBufferLen + headerSize)
		{
			compressed.resize(compressedBufferLen + headerSize);
		}

		deflateData(pixelChars, dataLen, &(compressed[0]) + headerSize, compressedBufferLen, encodeParam, compressedLength);
		byte_ptr compressedLenC = (byte_ptr) (&(compressedLength));

    cout << "Uncompressed size: " << dataLen << " " << "compressed size: " << compressedLength << "headersize: " << headerSize << endl;

		for (int i = 0; i < 4; i++)
		{
			compressed[i] = compressedLenC[i];
		}

#ifdef _MSC_VER
		memcpy_s(&(compressed[0]) + 4, compressedBufferLen + headerSize - 4, &sensorParameters, sizeof(OpenNICameraParameters));
#else
		memcpy(compressed + 4, &sensorParameters, sizeof(OpenNICameraParameters));
#endif

		printCompressedLength(dataLen, compressedLength, compressedBufferLen / 2);

		compressedLength += headerSize;
	}

	void FringeCompression::printCompressedLength
		(
		int originalLength,
		int compressedLength,
		int pixelLength
		)
	{
		cout << "Original Len: " << originalLength << endl;
		cout << "CompressedLen: " << compressedLength << endl;
		cout << "Compression Ratio: " << (((double) originalLength) / compressedLength) << endl;
		cout << "Total Compression Ratio: " << (((double) pixelLength * 2) / compressedLength) << endl;
	}

	unsigned short* FringeCompression::generatePixelBuffer
		(
		const unsigned short *pixels,
		unsigned columns,
		unsigned rows,
		unsigned & dataLen,
		bool computeLUT
		)
	{
		unsigned pixelLen = rows*columns;
		if (pixelLen <= 0 || pixels == NULL)
		{
			return NULL;
		}

		if (computeLUT)
		{
			lut_ = new int[pixelLen];
			std::fill(lut_, lut_ + pixelLen, -1);
		}

		if (pixelCopy_ == NULL)
		{
			pixelCopy_ = new unsigned short[pixelLen];
		}

		unsigned a = 500;
		dataLen = 0;
		int b = a*(a + 1);
		int X;
		int rowStart = rows / 3, rowEnd = rows / 3 * 2;
		int columnStart = columns / 3, columnEnd = rows / 3 * 2;

		for (int i = 0; i < pixelLen; i++)
		{
			X = pixels[i];			//== 0 ? 0 : (a*(a+1))/pixels[i];
			int r = i / columns;
			int c = i % columns;
			if (r < rowStart || r >= rowEnd || c<columnStart || c>columnEnd)
			{
				if (r % 3 == 1 && c % 3 == 1)
				{
					X = getMeanValue
						(
						pixels,
						rows,
						columns,
						i,
						b
						);

					if (computeLUT)
					{
						fillLutLocation(lut_, i, dataLen, columns);
					}
				}
				else
					continue;
			}
			else if (computeLUT)
			{
				lut_[i] = dataLen;
			}

			pixelCopy_[dataLen] = X;
			dataLen++;
		}

		dataLen *= 2; //short to chars


    if (!wroteLut_)
    {
      writeLUT(lutFileName_, lut_, columns, rows);
    }

		return pixelCopy_;
	}

	inline void FringeCompression::fillLutLocation
		(
		int* lookUpTable,
		int index,
		int compressedIndex,
		int lutCols
		)
	{
		int min = lutCols - 1;
		int eq = lutCols;
		int max = lutCols + 1;
		lookUpTable[index - max] = compressedIndex;
		lookUpTable[index - eq] = compressedIndex;
		lookUpTable[index - min] = compressedIndex;
		lookUpTable[index - 1] = compressedIndex;
		lookUpTable[index] = compressedIndex;
		lookUpTable[index + 1] = compressedIndex;
		lookUpTable[index + min] = compressedIndex;
		lookUpTable[index + eq] = compressedIndex;
		lookUpTable[index + max] = compressedIndex;
	}

	int FringeCompression::getMeanValue
		(
		const unsigned short *pixels,
		int rows,
		int cols,
		int index,
		int b
		)
	{
		if (pixels[index] == 0)
		{
			return 0;
		}

		//Vals array pattern, where #4 is the current element.
		//0 1 2
		//3 4 5
		//6 7 8
		int vals[9];
		vals[0] = index < (cols + 1) ? -1 : pixels[index - (cols + 1)];
		vals[1] = index <  cols ? -1 : pixels[index - cols];
		vals[2] = index < (cols - 1) ? -1 : pixels[index - (cols - 1)];

		vals[3] = index <1 ? -1 : pixels[index - 1];
		vals[4] = pixels[index];
		vals[5] = index == cols*rows - 1 ? -1 : pixels[index + 1];

		vals[6] = index > cols*(rows - 1) ? -1 : pixels[index + (cols - 1)];
		vals[7] = index > cols*(rows - 1) - 1 ? -1 : pixels[index + cols];
		vals[8] = index > cols*(rows - 1) - 2 ? -1 : pixels[index + (cols + 1)];

		float mean = 0;
		float num = 0;
		for (int i = 0; i < 9; i++)
		{
			if (vals[i] != -1)
			{
				mean += vals[i];
				num++;
			}
		}

		mean /= num;

		int val = 0;
		float diff = -1;
		for (int i = 0; i < 9; i++)
		{
			float valDiff = abs(vals[i] - mean);
			if (vals[i] != -1 && (diff == -1 || valDiff<diff))
			{
				diff = valDiff;
				val = vals[i];
			}
		}

		if (val>0)
		{
			val = b / val;
			val = b / val;
		}

		return val;
	}

  /**
   * Note: compressed vector does not include header
   */
	void FringeCompression::decodePixels
		(
		unsigned compressedLength,
		OpenNICameraParameters parameters,
		vector<unsigned char>& compressed,
		vector<unsigned short>& decompressed
		)
	{
    if (lut_ == NULL)
    {
      readLUT(lutFileName_, lut_, parameters.width, parameters.height);
    }

		unsigned imgSize = parameters.height*parameters.width;
		decompressed.resize(imgSize);
		unsigned decompressedBufferSize = imgSize * 2;
		unsigned char* decompressedChars = new unsigned char[decompressedBufferSize];
		unsigned decompressedLen;
		unsigned char * compressed_ptr = &(compressed[0]);
		inflateData(compressed_ptr, compressedLength, decompressedChars, decompressedBufferSize, decompressedLen);
		cout << "Decompressed Length: " << decompressedLen << " Predicated: " << decompressedBufferSize << endl;

		unsigned short *decomShort = (unsigned short *) decompressedChars;
		for (int j = 0; j < imgSize; j++)
		{
			decompressed[j] = lut_[j] == -1 ? 0 : decomShort[lut_[j]];
		}

		delete [] decompressedChars;
	}

	void FringeCompression::readLUT
		(
		string fileName,
		int* & lut,
		int & columns,
		int & rows
		)
	{
		FILE * pFile;
		long lSize;
		size_t result;

#ifdef _MSC_VER
		fopen_s(&pFile, fileName.c_str(), "rb");
#else
		pFile = fopen(fileName.c_str(), "rb");
    perror("fopen");
#endif

		if (pFile == NULL)
		{
			cout << "File error" << endl;
			exit(1);
		}

		// obtain file size:
		fseek(pFile, 0, SEEK_END);
		lSize = ftell(pFile);
		rewind(pFile);

		if (lSize < 2 * sizeof(int))
		{
			cout << "Lut Size too small: " << lSize << endl;
			return;
		}

		fread(&columns, sizeof(int), 1, pFile);
		fread(&rows, sizeof(int), 1, pFile);

		if (lSize != (2 + rows*columns)*sizeof(int))
		{
			cout << "Lut size not correct.  Expecting: " << (2 + rows*columns)*sizeof(int) << " Got: " << lSize << endl;
		}

    lSize -= 2 * sizeof(int);

		// allocate memory to contain the whole file:
    lut = new int[lSize / sizeof(int)];
    
		if (lut == NULL) { fputs("Memory error", stderr); exit(2); }
    result = fread(lut, sizeof(int), lSize / sizeof(int), pFile);

    if (result*sizeof(int) != lSize)
		{
			cout << "Lut Read size not correct.  Expecting: " << (2 + rows*columns)*sizeof(int) << " Got: " << result << endl;
			exit(3);
		}

		fclose(pFile);
	}

  unsigned FringeCompression::readDataId
    (
    const void* data
    )
  {
    unsigned * uSource = (unsigned*) data;
    return uSource[0];
  }

  void FringeCompression::encodeData
    (
    const void* data,
    const int dataSize,
    const int dataElements,
    const unsigned dataId,
    unsigned &headerSize,
    unsigned &compressedLength,
    vector<unsigned char> &compressed,
    int level
    )
  {
    headerSize = 4*sizeof(unsigned)/sizeof(char);
    byte * source = (byte*) data;
    unsigned sourceLength = dataSize*dataElements / sizeof(byte);
    compressed.resize(sourceLength + headerSize);
    byte * dest = &compressed[headerSize];

    deflateData
      (
      source,
      sourceLength,
      dest,
      sourceLength,
      level,
      compressedLength
      );

    unsigned * uSource = (unsigned*) &compressed[0];
    uSource[0] = compressedLength;
    uSource[1] = 2; //the following two bytes compresize the remaining header size.
    uSource[2] = dataId;
    uSource[3] = sourceLength;
  }

  void FringeCompression::decodeData
    (
    const void* data,
    const int dataSize,
    const int dataElements,
    unsigned &dataId,
    unsigned &decompressedLength,
    vector<unsigned char> &decompressed
    )
  {
    int headerSize = 2 * sizeof(unsigned) / sizeof(byte); //header is smaller because it doesn't contain compressed length, assumed to be read earlier
    byte * source = (byte*) data;
    source += headerSize;
    unsigned * uSource = (unsigned*) data;
    unsigned expectedLength = uSource[1];
    dataId = uSource[0];

    unsigned sourceLength = dataSize*dataElements / sizeof(byte);
    decompressed.resize(expectedLength);
    byte * dest = &decompressed[0];

    inflateData
      (
      source,
      sourceLength,
      dest,
      expectedLength,
      decompressedLength
      );
  }
}