#include <pcl/io/net_server.h>
#include <boost/asio.hpp>
#include <pcl/io/net_participant.h>

using boost::asio::ip::tcp;

namespace pcl
{
  NetServer::NetServer(
    int port
    /*boost::asio::io_service& io_service,
    const tcp::endpoint& endpoint)
    : acceptor_(io_service, endpoint),
    socket_(io_service)*/) : netState_(port)
  {
    waitForCLient();
  }

  void NetServer::waitForCLient()
  {
    netState_.acceptor.async_accept(netState_.socket,
      [this](boost::system::error_code ec)
    {
      if (!ec)
      {
        std::make_shared<NetParticipant>(std::move(netState_.socket), session_)->start();
      }

      waitForCLient();
    });
  }
}