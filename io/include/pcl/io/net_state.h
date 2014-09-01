#ifndef __PCL_IO_NET_STATE__
#define __PCL_IO_NET_STATE__

#include <pcl/io/boost.h>
#include <boost/asio.hpp>
#include <thread>

using boost::asio::ip::tcp;

namespace pcl
{
  struct NetState
  {
    NetState() :
      socket(io_service),
      resolver(io_service),
      acceptor(io_service)
    {
      ioThread = std::thread(&NetState::runIOService, this);
      /*
        ioThread = std::thread([&io_service]()
        {
        io_service.run();
        });*/
    }

    NetState(int port) :
      socket(io_service),
      endpoint(tcp::v4(), port),
      resolver(io_service),
      acceptor(io_service, endpoint)
    {
      ioThread = std::thread(&NetState::runIOService, this);
    }

    ~NetState()
    {
      socket.close();
      ioThread.join();
    }

    void runIOService()
    {
      io_service.run();
    }

    boost::asio::io_service io_service;
    tcp::resolver resolver;
    boost::shared_ptr<tcp::resolver::query> query;
    tcp::endpoint endpoint;
    tcp::socket socket;
    std::thread ioThread;
    tcp::acceptor acceptor;
  };
}
#endif //__PCL_IO_NET_STATE__