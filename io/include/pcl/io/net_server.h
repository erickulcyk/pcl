#ifndef PCL_IO_NET_SERVER
#define PCL_IO_NET_SERVER 1

#include <pcl/io/net_session.h>
#include <pcl/io/net_state.h>
namespace pcl
{
  class NetServer
  {
  public:
    NetServer(int port);

  private:
    void waitForCLient();

    NetState netState_;
    NetSession session_;
  };
}
#endif //PCL_IO_NET_SERVER