#include "serial_link.h"
#include <stdexcept>
//***********О
SerialLink::SerialLink(std::string address, int baudrate)
    : address_(address), baudrate_(baudrate) {}
bool SerialLink::isUp() const { return ser_.isOpen(); }
void SerialLink::up() {
  try {
    ser_.setPort(address_);
    ser_.setBaudrate(baudrate_);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();
  } catch (serial::IOException &e) {
    // std::cout << "could not open port";
    throw std::runtime_error("could not open port");
  }
}
std::string SerialLink::address() { return address_; }
void SerialLink::down() { ser_.close(); }
void SerialLink::sendData(const unsigned char *data, int size) {
  try {
    ser_.write(data, size);
  } catch (...) {
    throw std::runtime_error("port is closed");
  }
}
std::tuple<unsigned char *, int> SerialLink::getData() {
  size_t bytesAvailable = ser_.available();
  unsigned char *Buf = new unsigned char[bytesAvailable];
  int size = 0;
  if (bytesAvailable) {

    size = ser_.read(Buf, bytesAvailable);
  }
  return std::make_tuple(Buf, size);
}
