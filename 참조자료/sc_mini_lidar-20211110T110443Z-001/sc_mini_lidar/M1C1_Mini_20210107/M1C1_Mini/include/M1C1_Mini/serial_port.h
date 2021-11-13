#ifndef NODE_SERIAL_H
#define NODE_SERIAL_H

#include "node_lidar.h"
#include <stdexcept>
#include <limits>

using namespace std;

class MillisecondTimer
{
public:
  explicit MillisecondTimer(const uint32_t millis);
  int64_t remaining();

private:
  static timespec timespec_now();
  timespec expiry;
};

result_t waitForData(size_t data_count, uint32_t timeout_t, size_t *returned_size);
result_t getData(uint8_t *data, size_t size);

/*! 返回缓冲区中的字节数　Return the number of characters in the buffer. */
size_t available();

bool open_port(string port,int baudrate);
bool setDTR(bool level);
void close();
size_t write_data(const uint8_t *data, size_t length);

struct Timeout
{
#ifdef max
#undef max
#endif
  static uint32_t max()
  {
    return std::numeric_limits<uint32_t>::max();
  }
  /*!
  * Convenience function to generate Timeout structs using a
  * single absolute timeout.
  *
  * \param timeout A long that defines the time in milliseconds until a
  * timeout occurs after a call to read or write is made.
  *
  * \return Timeout struct that represents this simple timeout provided.
  */
  static Timeout simpleTimeout(uint32_t timeout_t)
  {
    return Timeout(max(), timeout_t, 0, timeout_t, 0);
  }

  /*! Number of milliseconds between bytes received to timeout on. */
  uint32_t inter_byte_timeout;
  /*! A constant number of milliseconds to wait after calling read. */
  uint32_t read_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
  *  calling read.
  */
  uint32_t read_timeout_multiplier;
  /*! A constant number of milliseconds to wait after calling write. */
  uint32_t write_timeout_constant;
  /*! A multiplier against the number of requested bytes to wait after
  *  calling write.
  */
  uint32_t write_timeout_multiplier;

  explicit Timeout(uint32_t inter_byte_timeout_ = 0,
                   uint32_t read_timeout_constant_ = 0,
                   uint32_t read_timeout_multiplier_ = 0,
                   uint32_t write_timeout_constant_ = 0,
                   uint32_t write_timeout_multiplier_ = 0)
      : inter_byte_timeout(inter_byte_timeout_),
        read_timeout_constant(read_timeout_constant_),
        read_timeout_multiplier(read_timeout_multiplier_),
        write_timeout_constant(write_timeout_constant_),
        write_timeout_multiplier(write_timeout_multiplier_)
  {
  }
};

/*!
* Enumeration defines the possible bytesizes for the serial port.
*/
typedef enum {
  fivebits = 5,
  sixbits = 6,
  sevenbits = 7,
  eightbits = 8
} bytesize_t;

/*!
* Enumeration defines the possible parity types for the serial port.
*/
typedef enum {
  parity_none = 0,
  parity_odd = 1,
  parity_even = 2,
  parity_mark = 3,
  parity_space = 4
} parity_t;

/*!
* Enumeration defines the possible stopbit types for the serial port.
*/
typedef enum {
  stopbits_one = 1,
  stopbits_two = 2,
  stopbits_one_point_five
} stopbits_t;

/*!
* Enumeration defines the possible flowcontrol types for the serial port.
*/
typedef enum {
  flowcontrol_none = 0,
  flowcontrol_software,
  flowcontrol_hardware
} flowcontrol_t;

#endif