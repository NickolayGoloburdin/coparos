/*
 * serial_link.h
 *
 *  Created on: 10 марта. 2022 г.
 *      Author: Nickolay
 */
#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H
#include "abstract_link.h"
#include <serial/serial.h>
//Модуль свзяи по UART
class SerialLink : public AbstractLink {

public:
  //Модуль иницализируется адресом порта и скоростью передачи данных в бодах
  SerialLink(std::string address, int baudrate);
  // : address_(address), baudrate_(baudrate) {}
  bool isUp() const override;
  std::string address() override;
  void up() override;
  void changeAddress(std::string address) override;
  void down() override;
  void sendData(const unsigned char *data, int size) override;
  std::shared_ptr<std::list<unsigned char>> getData() override;

private:
  int baudrate_;
  std::string address_;
  //В классе находится обект последовательного соединения из библиотеки serial
  serial::Serial ser_;
};

#endif // ABSTRACT_LINK_H