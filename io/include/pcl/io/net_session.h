#ifndef PCL_IO_NET_SESSION
#define PCL_IO_NET_SESSION 1

#include <pcl/io/net_participant.h>
#include <pcl/io/net_message.h>
#include <deque>
#include <set>

namespace pcl
{
  class NetSession
  {
  public:
    void join(NetParticipantPtr participant);

    void leave(NetParticipantPtr participant);

    void deliver(const NetMessage& msg);

  private:

    std::set<NetParticipantPtr> participants_;

    int maxRecentMsgs = 100;

    std::deque<NetMessage> recentMsgs_;
  };
}
#endif //PCL_IO_NET_SESSION_H