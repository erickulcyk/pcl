#ifndef __PCL_IO_NET_MESSAGE__
#define __PCL_IO_NET_MESSAGE__

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace pcl
{
  class NetMessage
  {

  public:
    enum CompressionType { NoCompression, FringeCompression };

    const static unsigned HeaderLength = sizeof(unsigned) * 2;

    NetMessage(CompressionType compressionType = NoCompression)
      : bodyLength_(HeaderLength),
      compressionType_(compressionType)
    {
    }

    inline const unsigned char* data() const
    {
      return data_.data();
    }

    inline unsigned char* data()
    {
      return data_.data();
    }

    inline  unsigned messageLength() const
    {
      return HeaderLength + bodyLength_;
    }

    inline const unsigned char* body() const
    {
      return data_.data() + HeaderLength;
    }

    inline unsigned char* body()
    {
      return data_.data() + HeaderLength;
    }

    inline unsigned messageType()
    {
      return messageType_;
    }

    inline unsigned bodyLength() const
    {
      return bodyLength_;
    }

    bool decodeHeader()
    {
      bodyLength_ = data_[0];
      messageType_ = data_[1];
      data_.resize(HeaderLength + bodyLength_);

      return bodyLength_ >= 0;
    }

    void encodeMessage(const unsigned char* data, unsigned messageType, unsigned bodyLength)
    {
      bodyLength_ = bodyLength;
      messageType_ = messageType;

      switch (compressionType_)
      {
      case NoCompression:
        data_.resize(bodyLength_ + HeaderLength);
        memcpy(data_.data() + HeaderLength, data, sizeof(const unsigned char) * bodyLength_);
        data_[0] = bodyLength_;
        data_[1] = messageType_;
        break;
      case FringeCompression:
        break;
      default:
        printf("Not a valid compression type!");
        break;
      }
    }

  private:
    unsigned bodyLength_;
    unsigned messageType_;
    CompressionType compressionType_;

    std::vector<unsigned char> data_;
  };
}
#endif // __PCL_IO_NET_MESSAGE__
