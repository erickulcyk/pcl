#include <pcl/io/net_participant.h>
#include <pcl/io/net_session.h>

namespace pcl
{
  NetParticipant::NetParticipant(tcp::socket socket, NetSession& room)
    : socket_(std::move(socket)),
    session_(room)
  {
  }

  void NetParticipant::start()
  {
    session_.join(shared_from_this());
    readHeader();
  }

  void NetParticipant::deliver(const NetMessage& msg)
  {
    bool write_in_progress = !msgsToWrite_.empty();
    msgsToWrite_.push_back(msg);
    if (!write_in_progress)
    {
      write();
    }
  }

  void NetParticipant::readHeader()
  {
    auto self(shared_from_this());
    boost::asio::async_read(socket_,
      boost::asio::buffer(readMsg_.data(), NetMessage::HeaderLength),
      [this, self](boost::system::error_code ec, std::size_t /*length*/)
    {
      if (!ec && readMsg_.decodeHeader())
      {
        readBody();
      }
      else
      {
        session_.leave(shared_from_this());
      }
    });
  }

  void NetParticipant::readBody()
  {
    auto self(shared_from_this());
    boost::asio::async_read(socket_,
      boost::asio::buffer(readMsg_.body(), readMsg_.bodyLength()),
      [this, self](boost::system::error_code ec, std::size_t /*length*/)
    {
      if (!ec)
      {
        session_.deliver(readMsg_);
        readHeader();
      }
      else
      {
        session_.leave(shared_from_this());
      }
    });
  }

  void NetParticipant::write()
  {
    auto self(shared_from_this());
    boost::asio::async_write(socket_,
      boost::asio::buffer(msgsToWrite_.front().data(),
      msgsToWrite_.front().messageLength()),
      [this, self](boost::system::error_code ec, std::size_t /*length*/)
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
        session_.leave(shared_from_this());
      }
    });
  }
}