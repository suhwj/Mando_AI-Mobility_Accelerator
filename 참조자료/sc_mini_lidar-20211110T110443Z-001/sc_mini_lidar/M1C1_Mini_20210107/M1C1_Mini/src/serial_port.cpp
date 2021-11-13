#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include <sys/select.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include "serial_port.h"

using namespace std;

int uart_fd = -1;
bool is_open_ = false;
pid_t pid;

#define SNCCS 19
#define BOTHER 0010000
struct termios2 {
  tcflag_t c_iflag;       /* input mode flags */
  tcflag_t c_oflag;       /* output mode flags */
  tcflag_t c_cflag;       /* control mode flags */
  tcflag_t c_lflag;       /* local mode flags */
  cc_t c_line;            /* line discipline */
  cc_t c_cc[SNCCS];       /* control characters */
  speed_t c_ispeed;       /* input speed */
  speed_t c_ospeed;       /* output speed */
};

MillisecondTimer::MillisecondTimer(const uint32_t millis) : expiry(
																timespec_now())
{
	int64_t tv_nsec = expiry.tv_nsec + (millis * 1e6);

	if (tv_nsec >= 1e9)
	{
		int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
		expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
		expiry.tv_sec += sec_diff;
	}
	else
	{
		expiry.tv_nsec = tv_nsec;
	}
}

int64_t MillisecondTimer::remaining()
{
	timespec now(timespec_now());
	int64_t millis = (expiry.tv_sec - now.tv_sec) * 1e3;
	millis += (expiry.tv_nsec - now.tv_nsec) / 1e6;
	return millis;
}

timespec MillisecondTimer::timespec_now()
{
	timespec time;
#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
	clock_serv_t cclock;
	mach_timespec_t mts;
	host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
	clock_get_time(cclock, &mts);
	mach_port_deallocate(mach_task_self(), cclock);
	time.tv_sec = mts.tv_sec;
	time.tv_nsec = mts.tv_nsec;
#else
	clock_gettime(CLOCK_MONOTONIC, &time);
#endif
	return time;
}

result_t waitForData(size_t data_count, uint32_t timeout, size_t *returned_size)
{
	size_t length = 0;

	if (returned_size == NULL)
	{
		returned_size = (size_t *)&length;
	}

	*returned_size = 0;

	int max_fd;
	fd_set input_set;
	struct timeval timeout_val;

	FD_ZERO(&input_set);//node:将set清零使集合中不含任何fd
	FD_SET(uart_fd, &input_set);//node:将uart_fd加入set集合
	max_fd = uart_fd + 1;

	timeout_val.tv_sec = timeout / 1000;
	timeout_val.tv_usec = (timeout % 1000) * 1000;

	if (is_open_)
	{
    /*得到缓冲区里有多少字节要被读取，然后将字节数放入returned_size里面。*/
		if (ioctl(uart_fd, FIONREAD, returned_size) == -1)
		{
			return -2;
		}

		if (*returned_size >= data_count)
		{
			return 0;
		}
	}

	MillisecondTimer total_timeout(timeout);

	while (is_open_)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();
		//printf("---port:114---timeout_remaining_ms=%d\n",timeout_remaining_ms);

		if ((timeout_remaining_ms <= 0))
		{
			// Timed out
			printf("---port:118---%d\n",timeout_remaining_ms);
			return -1;
		}

		/* Do the select */
		int n = ::select(max_fd, &input_set, NULL, NULL, &timeout_val);

		//printf("---port:126---%d\n",n);

		if (n < 0)
		{
			if (errno == EINTR)
			{
				//printf("---port---129\n");
				return -1;
			}

			// Otherwise there was some error
			return -2;
		}
		else if (n == 0)
		{
			// time out
			//printf("---port---139\n");
			return -1;
		}
		else
		{
			// data avaliable
			assert(FD_ISSET(uart_fd, &input_set));

			if (ioctl(uart_fd, FIONREAD, returned_size) == -1)
			{
				return -2;
			}

			if (*returned_size >= data_count)
			{
				return 0;
			}
			else
			{
				int remain_timeout = timeout_val.tv_sec * 1000000 + timeout_val.tv_usec;
				int expect_remain_time = (data_count - *returned_size) * 1000000 * 8 / 115200;
				if (remain_timeout > expect_remain_time)
				{
					usleep(expect_remain_time);
				}
			}
		}
	}

	return -2;
}

timespec timespec_from_ms(const uint32_t millis)
{
	timespec time;
	time.tv_sec = millis / 1e3;
	time.tv_nsec = (millis - (time.tv_sec * 1e3)) * 1e6;
	return time;
}

bool waitReadable(uint32_t timeout_t)
{
	// Setup a select call to block for serial data or a timeout
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(uart_fd, &readfds);
	timespec timeout_ts(timespec_from_ms(timeout_t));
	int r = pselect(uart_fd + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

	if (r < 0)
	{
		// Select was interrupted
		if (errno == EINTR)
		{
			return false;
		}

		// Otherwise there was some error
		return false;
	}

	// Timeout occurred
	if (r == 0)
	{
		return false;
	}

	// This shouldn't happen, if r > 0 our fd has to be in the list!
	if (!FD_ISSET(uart_fd, &readfds))
	{
		return false;
	}

	// Data available to read.
	return true;
}

size_t available()
{
	if (!is_open_)
	{
		return 0;
	}

	int count = 0;

	if (-1 == ioctl(uart_fd, TIOCINQ, &count))
	{
		return 0;
	}
	else
	{
		return static_cast<size_t>(count);
	}
}

uint32_t byte_time_ns_;

void waitByteTimes(size_t count)
{
	timespec wait_time = {0, static_cast<long>(byte_time_ns_ * count)};
	pselect(0, NULL, NULL, NULL, &wait_time, NULL);
}

result_t getData(uint8_t *buf, size_t size)
{
	// If the port is not open, throw
	if (!is_open_)
	{
		return 0;
	}

	Timeout timeout_;

	size_t bytes_read = 0;

	// Calculate total timeout in milliseconds t_c + (t_m * N)
	long total_timeout_ms = timeout_.read_timeout_constant;
	total_timeout_ms += timeout_.read_timeout_multiplier * static_cast<long>(size);
	MillisecondTimer total_timeout(total_timeout_ms);

	// Pre-fill buffer with available bytes
	{
		ssize_t bytes_read_now = ::read(uart_fd, buf, size);

		if (bytes_read_now > 0)
		{
			bytes_read = bytes_read_now;
		}
	}

	while (bytes_read < size)
	{
		int64_t timeout_remaining_ms = total_timeout.remaining();

		if (timeout_remaining_ms <= 0)
		{
			// Timed out
			break;
		}

		// Timeout for the next select is whichever is less of the remaining
		// total read timeout and the inter-byte timeout.
		uint32_t timeout_t = std::min(static_cast<uint32_t>(timeout_remaining_ms),
									  timeout_.inter_byte_timeout);

		// Wait for the device to be readable, and then attempt to read.
		if (waitReadable(timeout_t))
		{
			// If it's a fixed-length multi-byte read, insert a wait here so that
			// we can attempt to grab the whole thing in a single IO call. Skip
			// this wait if a non-max inter_byte_timeout is specified.
			if (size > 1 && timeout_.inter_byte_timeout == Timeout::max())
			{
				size_t bytes_available = available();

				if (bytes_available + bytes_read < size)
				{
					waitByteTimes(size - (bytes_available + bytes_read));
				}
			}

			// This should be non-blocking returning only what is available now
			//  Then returning so that select can block again.
			ssize_t bytes_read_now = ::read(uart_fd, buf + bytes_read, size - bytes_read);

			// read should always return some data as select reported it was
			// ready to read when we get to this point.
			if (bytes_read_now < 1)
			{
				// Disconnected devices, at least on Linux, show the
				// behavior that they are always ready to read immediately
				// but reading returns nothing.
				continue;
			}

			// Update bytes_read
			bytes_read += static_cast<size_t>(bytes_read_now);

			// If bytes_read == size then we have read everything we need
			if (bytes_read == size)
			{
				break;
			}

			// If bytes_read < size then we have more to read
			if (bytes_read < size)
			{
				continue;
			}

			// If bytes_read > size then we have over read, which shouldn't happen
			if (bytes_read > size)
			{
				break;
			}
		}
	}

	return bytes_read;
}

size_t write_data(const uint8_t *data, size_t length) {
  if (is_open_ == false) {
    return 0;
  }

  Timeout timeout_;
  fd_set writefds;
  size_t bytes_written = 0;

  // Calculate total timeout in milliseconds t_c + (t_m * N)
  long total_timeout_ms = timeout_.write_timeout_constant;
  total_timeout_ms += timeout_.write_timeout_multiplier * static_cast<long>
                      (length);
  MillisecondTimer total_timeout(total_timeout_ms);

  bool first_iteration = true;

  while (bytes_written < length) {
    int64_t timeout_remaining_ms = total_timeout.remaining();

    // Only consider the timeout if it's not the first iteration of the loop
    // otherwise a timeout of 0 won't be allowed through
    if (!first_iteration && (timeout_remaining_ms <= 0)) {
      // Timed out
      break;
    }

    first_iteration = false;

    timespec timeout(timespec_from_ms(timeout_remaining_ms));

    FD_ZERO(&writefds);
    FD_SET(uart_fd, &writefds);

    // Do the select
    int r = pselect(uart_fd + 1, NULL, &writefds, NULL, &timeout, NULL);

    // Figure out what happened by looking at select's response 'r'
    /** Error **/
    if (r < 0) {
      // Select was interrupted, try again
      if (errno == EINTR) {
        continue;
      }

      // Otherwise there was some error
      continue;
    }

    /** Timeout **/
    if (r == 0) {
      break;
    }

    /** Port ready to write **/
    if (r > 0) {
      // Make sure our file descriptor is in the ready to write list
      if (FD_ISSET(uart_fd, &writefds)) {
        // This will write some
        ssize_t bytes_written_now = ::write(uart_fd, data + bytes_written,
                                            length - bytes_written);

        // write should always return some data as select reported it was
        // ready to write when we get to this point.
        if (bytes_written_now < 1) {
          // Disconnected devices, at least on Linux, show the
          // behavior that they are always ready to write immediately
          // but writing returns nothing.
          continue;
        }

        // Update bytes_written
        bytes_written += static_cast<size_t>(bytes_written_now);

        // If bytes_written == size then we have written everything we need to
        if (bytes_written == length) {
          break;
        }

        // If bytes_written < size then we have more to write
        if (bytes_written < length) {
          continue;
        }

        // If bytes_written > size then we have over written, which shouldn't happen
        if (bytes_written > length) {
          break;
        }
      }

      // This shouldn't happen, if r > 0 our fd has to be in the list!
      break;
      //THROW (IOException, "select reports ready to write, but our fd isn't in the list, this shouldn't happen!");
    }
  }

  return bytes_written;
}

bool getTermios(termios *tio) {
  ::memset(tio, 0, sizeof(termios));

  if (::tcgetattr(uart_fd, tio) == -1) {
    return false;
  }

  return true;
}

void set_common_props(termios *tio) {
#ifdef OS_SOLARIS
  tio->c_iflag &= ~(IMAXBEL | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                    ICRNL | IXON);
  tio->c_oflag &= ~OPOST;
  tio->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tio->c_cflag &= ~(CSIZE | PARENB);
  tio->c_cflag |= CS8;
#else
  ::cfmakeraw(tio);
#endif
  tio->c_cflag |= CLOCAL | CREAD;
  tio->c_cc[VTIME] = 0;
  tio->c_cc[VMIN] = 0;
}

bytesize_t bytesize_ ;
parity_t parity_;
stopbits_t stopbits_;
flowcontrol_t flowcontrol_;

void set_databits(termios *tio, bytesize_t databits) {
  tio->c_cflag &= ~CSIZE;

  switch (databits) {
    case fivebits:
      tio->c_cflag |= CS5;
      break;

    case sixbits:
      tio->c_cflag |= CS6;
      break;

    case sevenbits:
      tio->c_cflag |= CS7;
      break;

    case eightbits:
      tio->c_cflag |= CS8;
      break;

    default:
      tio->c_cflag |= CS8;
      break;
  }
}

static inline void set_parity(termios *tio, parity_t parity) {
  tio->c_iflag &= ~(PARMRK | INPCK);
  tio->c_iflag |= IGNPAR;

  switch (parity) {

#ifdef CMSPAR

    // Here Installation parity only for GNU/Linux where the macro CMSPAR.
    case parity_space:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB | CMSPAR;
      break;

    case parity_mark:
      tio->c_cflag |= PARENB | CMSPAR | PARODD;
      break;
#endif

    case parity_none:
      tio->c_cflag &= ~PARENB;
      break;

    case parity_even:
      tio->c_cflag &= ~PARODD;
      tio->c_cflag |= PARENB;
      break;

    case parity_odd:
      tio->c_cflag |= PARENB | PARODD;
      break;

    default:
      tio->c_cflag |= PARENB;
      tio->c_iflag |= PARMRK | INPCK;
      tio->c_iflag &= ~IGNPAR;
      break;
  }
}


static inline void set_stopbits(termios *tio, stopbits_t stopbits) {
  switch (stopbits) {
    case stopbits_one:
      tio->c_cflag &= ~CSTOPB;
      break;

    case stopbits_two:
      tio->c_cflag |= CSTOPB;
      break;

    default:
      tio->c_cflag &= ~CSTOPB;
      break;
  }
}

void set_flowcontrol(termios *tio,flowcontrol_t flowcontrol) {
  switch (flowcontrol) {
    case flowcontrol_none:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;

    case flowcontrol_hardware:
      tio->c_cflag |= CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;

    case flowcontrol_software:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag |= IXON | IXOFF | IXANY;
      break;

    default:
      tio->c_cflag &= ~CRTSCTS;
      tio->c_iflag &= ~(IXON | IXOFF | IXANY);
      break;
  }
}

bool setTermios(const termios *tio) {

  tcflush(uart_fd, TCIFLUSH);

  if (fcntl(uart_fd, F_SETFL, FNDELAY)) {
    return false;
  }

  if (::tcsetattr(uart_fd, TCSANOW, tio) == -1) {
    return false;
  }

  return true;
}

bool is_standardbaudrate(unsigned long baudrate, speed_t &baud) {
  // setup baud rate
  bool custom_baud = false;

  switch (baudrate) {
#ifdef B0

    case 0:
      baud = B0;
      break;
#endif
#ifdef B50

    case 50:
      baud = B50;
      break;
#endif
#ifdef B75

    case 75:
      baud = B75;
      break;
#endif
#ifdef B110

    case 110:
      baud = B110;
      break;
#endif
#ifdef B134

    case 134:
      baud = B134;
      break;
#endif
#ifdef B150

    case 150:
      baud = B150;
      break;
#endif
#ifdef B200

    case 200:
      baud = B200;
      break;
#endif
#ifdef B300

    case 300:
      baud = B300;
      break;
#endif
#ifdef B600

    case 600:
      baud = B600;
      break;
#endif
#ifdef B1200

    case 1200:
      baud = B1200;
      break;
#endif
#ifdef B1800

    case 1800:
      baud = B1800;
      break;
#endif
#ifdef B2400

    case 2400:
      baud = B2400;
      break;
#endif
#ifdef B4800

    case 4800:
      baud = B4800;
      break;
#endif
#ifdef B7200

    case 7200:
      baud = B7200;
      break;
#endif
#ifdef B9600

    case 9600:
      baud = B9600;
      break;
#endif
#ifdef B14400

    case 14400:
      baud = B14400;
      break;
#endif
#ifdef B19200

    case 19200:
      baud = B19200;
      break;
#endif
#ifdef B28800

    case 28800:
      baud = B28800;
      break;
#endif
#ifdef B57600

    case 57600:
      baud = B57600;
      break;
#endif
#ifdef B76800

    case 76800:
      baud = B76800;
      break;
#endif
#ifdef B38400

    case 38400:
      baud = B38400;
      break;
#endif
#ifdef B115200

    case 115200:
      baud = B115200;
      break;
#endif
#ifdef B128000

    case 128000:
      baud = B128000;
      break;
#endif
#ifdef B153600

    case 153600:
      baud = B153600;
      break;
#endif
#ifdef B230400

    case 230400:
      baud = B230400;
      break;
#endif
#ifdef B256000

    case 256000:
      baud = B256000;
      break;
#endif
#ifdef B460800

    case 460800:
      baud = B460800;
      break;
#endif
#ifdef B576000

    case 576000:
      baud = B576000;
      break;
#endif
#ifdef B921600

    case 921600:
      baud = B921600;
      break;
#endif
#ifdef B1000000

    case 1000000:
      baud = B1000000;
      break;
#endif
#ifdef B1152000

    case 1152000:
      baud = B1152000;
      break;
#endif
#ifdef B1500000

    case 1500000:
      baud = B1500000;
      break;
#endif
#ifdef B2000000

    case 2000000:
      baud = B2000000;
      break;
#endif
#ifdef B2500000

    case 2500000:
      baud = B2500000;
      break;
#endif
#ifdef B3000000

    case 3000000:
      baud = B3000000;
      break;
#endif
#ifdef B3500000

    case 3500000:
      baud = B3500000;
      break;
#endif
#ifdef B4000000

    case 4000000:
      baud = B4000000;
      break;
#endif

    default:
      custom_baud = true;
  }

  return !custom_baud;

}

bool setStandardBaudRate(speed_t baudrate) {
#ifdef __linux__
  // try to clear custom baud rate, using termios v2
  struct termios2 tio2;

  if (::ioctl(uart_fd, TCGETS2, &tio2) != -1) {
    if (tio2.c_cflag & BOTHER) {
      tio2.c_cflag &= ~BOTHER;
      tio2.c_cflag |= CBAUD;
      ::ioctl(uart_fd, TCSETS2, &tio2);
    }
  }

  // try to clear custom baud rate, using serial_struct (old way)
  struct serial_struct serial;
  ::memset(&serial, 0, sizeof(serial));

  if (::ioctl(uart_fd, TIOCGSERIAL, &serial) != -1) {
    if (serial.flags & ASYNC_SPD_CUST) {
      serial.flags &= ~ASYNC_SPD_CUST;
      serial.custom_divisor = 0;
      // we don't check on errors because a driver can has not this feature
      ::ioctl(uart_fd, TIOCSSERIAL, &serial);
    }
  }

#endif

  termios tio;

  if (!getTermios(&tio)) {
    return false;
  }

#ifdef _BSD_SOURCE

  if (::cfsetspeed(&tio, baudrate) < 0) {
    return false;
  }

#else

  if (::cfsetispeed(&tio, baudrate) < 0) {
    return false;
  }

  if (::cfsetospeed(&tio, baudrate) < 0) {
    return false;
  }

#endif
  return setTermios(&tio);
}

bool setCustomBaudRate(unsigned long baudrate) {
  struct termios2 tio2;

  if (::ioctl(uart_fd, TCGETS2, &tio2) != -1) {
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER;

    tio2.c_ispeed = baudrate;
    tio2.c_ospeed = baudrate;

    tcflush(uart_fd, TCIFLUSH);

    if (fcntl(uart_fd, F_SETFL, FNDELAY)) {
      return false;
    }

    /*struct flock file_lock;
    file_lock.l_type = F_WRLCK;
    file_lock.l_whence = SEEK_SET;
    file_lock.l_start = 0;
    file_lock.l_len = 0;
    file_lock.l_pid = getpid();
    if (fcntl(fd_, F_SETLK, &file_lock) != 0) {
      return false;
    }*/



    if (::ioctl(uart_fd, TCSETS2, &tio2) != -1 && ::ioctl(uart_fd, TCGETS2, &tio2) != -1) {
      return true;
    }
  }

  struct serial_struct serial;

  if (::ioctl(uart_fd, TIOCGSERIAL, &serial) == -1) {
    return false;
  }

  serial.flags &= ~ASYNC_SPD_MASK;
  serial.flags |= (ASYNC_SPD_CUST /* | ASYNC_LOW_LATENCY*/);
  serial.custom_divisor = serial.baud_base / baudrate;

  if (serial.custom_divisor == 0) {
    return false;
  }

  if (serial.custom_divisor * baudrate != serial.baud_base) {
  }

  if (::ioctl(uart_fd, TIOCSSERIAL, &serial) == -1) {
    return false;
  }

  return setStandardBaudRate(B38400);
}

bool setBaudrate(unsigned long baudrate) {

  if (uart_fd == -1) {
    return false;
  }

  //baudrate_ = baudrate;
  // OS X support
#if defined(MAC_OS_X_VERSION_10_4) && (MAC_OS_X_VERSION_MIN_REQUIRED >= MAC_OS_X_VERSION_10_4)
  // Starting with Tiger, the IOSSIOSPEED ioctl can be used to set arbitrary baud rates
  // other than those specified by POSIX. The driver for the underlying serial hardware
  // ultimately determines which baud rates can be used. This ioctl sets both the input
  // and output speed.
  speed_t new_baud = static_cast<speed_t>(baudrate);

  if (-1 == ioctl(fd_, IOSSIOSPEED, &new_baud, 1)) {
    return false;
  }

  // Linux Support
#elif defined(__linux__) && defined (TIOCSSERIAL)
  speed_t baud;
  bool standard_baud = is_standardbaudrate(baudrate, baud);

  if (!standard_baud) {
    return setCustomBaudRate(baudrate);
  } else {
    return setStandardBaudRate(baud);
  }

#else
  return false;
#endif

}

bool open_port(string port,int baudrate) {
  
  if (port.empty()) {
    return false;
  }

  if (is_open_ == true) {
    return true;
  }

  pid = -1;
  pid = getpid();
#ifdef USE_LOCK_FILE

  if (LOCK(port_.c_str(), pid)) {
    fprintf(stderr, "Could not lock serial port for exclusive access\n");
    return false;
  }

#endif

  uart_fd = ::open(port.c_str(),
               O_RDWR | O_NOCTTY | O_NONBLOCK | O_APPEND | O_NDELAY);

  if (uart_fd == -1) {
    switch (errno) {
      case EINTR:
        // Recurse because this is a recoverable error.
        return open_port(port,baudrate);

      case ENFILE:
      case EMFILE:
      default:
#ifdef USE_LOCK_FILE
        UNLOCK(port_.c_str(), pid);
#endif
        pid = -1;
        return false;
    }
  }

  termios tio;

  if (!getTermios(&tio)) {
#ifdef USE_LOCK_FILE
    UNLOCK(port_.c_str(), pid);
#endif
    return false;
  }
  bytesize_= eightbits;
  parity_ = parity_none;
  stopbits_ = stopbits_one;
  flowcontrol_ = flowcontrol_none;
  set_common_props(&tio);
  set_databits(&tio, bytesize_);
  set_parity(&tio, parity_);
  set_stopbits(&tio, stopbits_);
  set_flowcontrol(&tio, flowcontrol_);

  if (!setTermios(&tio)) {
#ifdef USE_LOCK_FILE
    UNLOCK(port_.c_str(), pid);
#endif
    return false;
  }

  if (!setBaudrate(baudrate)) {
#ifdef USE_LOCK_FILE
    UNLOCK(port_.c_str(), pid);
#endif
    return false;
  }

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_ns = 1e9 / baudrate;
  byte_time_ns_ = bit_time_ns * (1 + bytesize_ + parity_ + stopbits_);

  // Compensate for the stopbits_one_point_five enum being equal to int 3,
  // and not 1.5.
  if (stopbits_ == stopbits_one_point_five) {
    byte_time_ns_ += ((1.5 - stopbits_one_point_five) * bit_time_ns);
  }

  is_open_ = true;
  return true;
}

bool setDTR(bool level) {
  if (is_open_ == false) {
    return false;
  }

  int command = TIOCM_DTR;

  if (level) {
    if (-1 == ioctl(uart_fd, TIOCMBIS, &command)) {
      return false;
    }
  } else {
    if (-1 == ioctl(uart_fd, TIOCMBIC, &command)) {
      return false;
    }
  }

  return true;
}

void close() {
  if (is_open_ == true) {
    if (uart_fd != -1) {
      ::close(uart_fd);
    }

#ifdef USE_LOCK_FILE
    UNLOCK(port_.c_str(), pid);
#endif
    uart_fd = -1;
    pid = -1;
    is_open_ = false;
  }
}



