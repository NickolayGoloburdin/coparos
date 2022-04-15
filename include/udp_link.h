#ifndef UDP_LINK_H
#define UDP_LINK_H
#include "abstract_link.h"
#include <fcntl.h>
#include <netinet/in.h>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/types.h>

class UDPLink : public AbstractLink {

public:
  UDPLink(const std::string &address, int r_port, int t_port);
  bool isUp() const override;
  std::string address() override;
  void up() override;
  void down() override;
  void sendData(const unsigned char *data, int size) override;
  std::shared_ptr<std::list<char>> getData() override;

private:
  std::string address_;
  int sock;
  std::string addr;
  struct sockaddr_in in_addr;
  struct sockaddr_in out_addr;
  int r;
};

#endif // ABSTRACT_LINK_H