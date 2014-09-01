#ifndef PCL_IO_NET_CLIENT_H
#define PCL_IO_NET_CLIENT_H 1

#include <deque>
#include <boost/asio.hpp>
#include <string>

#include <pcl/io/net_message.h>
#include <pcl/io/net_state.h>

using boost::asio::ip::tcp;
using std::string;

namespace pcl
{
  class NetClient
  {
  public:
    NetClient(string serverAddress, int serverPort);

    void write(const NetMessage& msg);

    void close();

  private:
    void connect(tcp::resolver::iterator endpoint_iterator);

    void readHeader();

    void readBody();

    void write();

  private:
    pcl::NetState netState_;
    NetMessage readMessage_;
    std::deque<NetMessage> msgsToWrite_;
  };
}
#endif //PCL_IO_NET_CLIENT_H