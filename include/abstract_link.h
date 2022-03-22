#ifndef ABSTRACT_LINK_H
#define ABSTRACT_LINK_H
#include <string>
#include <tuple>
class AbstractLink {

public:
  explicit AbstractLink();
  virtual bool isUp() const = 0;
  virtual std::string address() = 0;
  virtual void up() = 0;
  virtual void down() = 0;
  virtual void sendData(const unsigned char *data, int size) = 0;
  virtual std::tuple<unsigned char *, int> getData() = 0;
};
#endif // ABSTRACT_LINK_H