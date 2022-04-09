#include "udp_link.h"
#include <stdexcept>

UDPLink::UDPLink(const std::string &addr, int r_port, int t_port)
    : address_(addr) {

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  fcntl(sock, F_SETFL, O_NONBLOCK);
  if (sock < 0) {
    perror("socket");
    exit(1);
  }
  in_addr.sin_family = AF_INET;
  in_addr.sin_port = htons(r_port);
  in_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  out_addr.sin_family = AF_INET;
  out_addr.sin_port = htons(t_port);
  out_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
}

bool UDPLink::isUp() const { return r >= 0; }
void UDPLink::up() {
  r = bind(sock, (struct sockaddr *)&in_addr, sizeof(in_addr));
  if (r < 0) {
    throw std::runtime_error("Port already in use");
  }
}
std::string UDPLink::address() { return address_; }
void UDPLink::down() {}
void UDPLink::sendData(const unsigned char *data, int size) {

  sendto(sock, data, size, 0, (struct sockaddr *)&out_addr, sizeof(out_addr));
}
std::tuple<unsigned char *, int> UDPLink::getData() {
  unsigned char *buf = new unsigned char;
  int bytes_read;
  bytes_read = recvfrom(sock, buf, 1024, 0, NULL, NULL);
  return std::make_tuple(buf, bytes_read);
}
