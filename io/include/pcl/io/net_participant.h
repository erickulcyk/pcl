#ifndef PCL_IO_NET_PARTICIPANT_H
#define PCL_IO_NET_PARTICIPANT_H 1

#include <deque>
#include <boost/asio.hpp>
#include <pcl/io/net_message.h>

using boost::asio::ip::tcp;

namespace pcl
{
  class NetSession;

  class NetParticipant : public std::enable_shared_from_this<NetParticipant>
  {
    public:
      NetParticipant(tcp::socket socket, NetSession& room);

      void start();

      void deliver(const NetMessage& msg);

    private:
      void readHeader();

      void readBody();

      void write();

      tcp::socket socket_;
      NetSession& session_;
      NetMessage readMsg_;
      std::deque<NetMessage> msgsToWrite_;
  };

  typedef std::shared_ptr<NetParticipant> NetParticipantPtr;
}
#endif //PCL_IO_NET_PARTICIPANT
