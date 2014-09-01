#include <cstdlib>
#include <deque>
#include <iostream>
#include <thread>
#include <boost/asio.hpp>
#include <pcl/io/net_message.h>
#include <pcl/io/net_state.h>
#include <string>
#include <pcl/io/net_client.h>

using boost::asio::ip::tcp;
using std::string;

namespace pcl
{
  NetClient::NetClient(string serverAddress, int serverPort)
    /*
    boost::asio::io_service& io_service,
    tcp::resolver::iterator endpoint_iterator)
    : io_service_(io_service),
    socket_(io_service)*/
  {
    auto endpoint_iterator = netState_.resolver.resolve({ serverAddress, std::to_string(serverPort) });
    connect(endpoint_iterator);
  }

  void NetClient::write(const NetMessage& msg)
  {
    netState_.io_service.post(
      [this, msg]()
    {
      bool write_in_progress = !msgsToWrite_.empty();
      msgsToWrite_.push_back(msg);
      if (!write_in_progress)
      {
        write();
      }
    });
  }

  void NetClient::close()
  {
    netState_.io_service.post([this]() { netState_.socket.close(); });
  }

  void NetClient::connect(tcp::resolver::iterator endpoint_iterator)
  {
    boost::asio::async_connect(netState_.socket, endpoint_iterator,
      [this](boost::system::error_code ec, tcp::resolver::iterator)
    {
      if (!ec)
      {
        readHeader();
      }
    });
  }

  void NetClient::readHeader()
  {
    boost::asio::async_read(netState_.socket,
      boost::asio::buffer(readMessage_.data(), NetMessage::HeaderLength),
      [this](boost::system::error_code ec, std::size_t /*length*/)
    {
      if (!ec && readMessage_.decodeHeader())
      {
        readBody();
      }
      else
      {
        netState_.socket.close();
      }
    });
  }

  void NetClient::readBody()
  {
    boost::asio::async_read(netState_.socket,
      boost::asio::buffer(readMessage_.body(), readMessage_.bodyLength()),
      [this](boost::system::error_code ec, std::size_t /*length*/)
    {
      if (!ec)
      {
        std::cout.write((char *) readMessage_.body(), readMessage_.bodyLength());
        std::cout << "\n";
        readHeader();
      }
      else
      {
        netState_.socket.close();
      }
    });
  }

  void NetClient::write()
  {
    boost::asio::async_write(netState_.socket,
      boost::asio::buffer(msgsToWrite_.front().data(),
      msgsToWrite_.front().messageLength()),
      [this](boost::system::error_code ec, std::size_t /*length*/)
    {
      if (!ec)
      {
        msgsToWrite_.pop_front();
        if (!msgsToWrite_.empty())
        {
          write();
        }
      }
      else
      {
        netState_.socket.close();
      }
    });
  }
}

int main(int argc, char* argv [])
{
  try
  {
    if (argc != 3)
    {
      std::cerr << "Usage: chat_client <host> <port>\n";
      return 1;
    }

    //boost::asio::io_service io_service;

    //tcp::resolver resolver(io_service);
    //auto endpoint_iterator = resolver.resolve({ argv[1], argv[2] });
    pcl::NetClient c(argv[1], std::stoi(argv[2]));

    //std::thread t([&io_service](){ io_service.run(); });

    char line[1000 + 1];

    while (std::cin.getline(line, 1000 + 1))
    {
      pcl::NetMessage msg;
      msg.encodeMessage((const unsigned char*) line, 0, strlen(line));
      c.write(msg);
    }

    //c.close();
    //t.join();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}
