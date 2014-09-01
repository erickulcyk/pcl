#include <pcl/io/net_session.h>

namespace pcl
{
  void NetSession::join(NetParticipantPtr participant)
  {
    participants_.insert(participant);
    for (auto msg : recentMsgs_)
      participant->deliver(msg);
  }

  void NetSession::leave(NetParticipantPtr participant)
  {
    participants_.erase(participant);
  }

  void NetSession::deliver(const NetMessage& msg)
  {
    recentMsgs_.push_back(msg);
    while (recentMsgs_.size() > maxRecentMsgs)
      recentMsgs_.pop_front();

    for (auto participant : participants_)
      participant->deliver(msg);
  }
}