#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H
#include "abstract_link.h"
#include <serial/serial.h>

class SerialLink : public AbstractLink {

public:
  SerialLink(std::string address, int baudrate);
  // : address_(address), baudrate_(baudrate) {}
  bool isUp() const override;
  std::string address() override;
  void up() override;
  void down() override;
  void sendData(const unsigned char *data, int size) override;
  std::tuple<unsigned char *, int> getData() override;

private:
  int baudrate_;
  std::string address_;
  serial::Serial ser_;
};

#endif // ABSTRACT_LINK_H