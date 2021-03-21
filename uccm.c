#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define UCCM_HEADER 0xc500
#define UCCM_TAIL 0xca
#define UCCM_TIMESTAMP_POS 27
#define UCCM_LEAPSECOND_POS 32
#define UCCM_FL0_POS 33
#define UCCM_FL1_POS 34
#define UCCM_FL2_POS 35
#define UCCM_FL3_POS 36
#define UCCM_TAIL_POS 43
#define UCCM_FL0_LOCK_BIT 1<<5
#define UCCM_FL0_LEAP_ANNOUNCE_BIT 1<<1
#define UCCM_FL0_WARMUP_BIT 1
#define UCCM_FL1_INIT_BIT 1<<3
#define UCCM_FL1_GPS_TIME_BIT 1<<2
#define UCCM_FL1_LOW_VOLTAGE_BIT 1<<1
#define UCCM_FL2_SYMMETRICOM_BIT 1<<7
#define UCCM_FL2_TRIMBLE_BIT 1<<6
#define UCCM_FL2_SURVEY_BIT 1<<3
#define UCCM_FL3_NO_ANTENNA_BIT 1<<5
#define UCCM_FL3_NO_GPS_SIGNAL_BIT 1<<4

void handle_uccm(unsigned char* buffer)
{
  unsigned short header = ntohs((unsigned short)*((unsigned short*)buffer));
  unsigned char tail = *(buffer + UCCM_TAIL_POS);
  if (header == UCCM_HEADER && tail == UCCM_TAIL) {
    printf("Valid buffer.\n");
  }

  unsigned long timestamp =
    ntohl((unsigned long)*((unsigned long*)(buffer + UCCM_TIMESTAMP_POS)));
  printf("Timestamp: %ld\n", timestamp);

  unsigned int leap_second = (unsigned int)*(buffer + UCCM_LEAPSECOND_POS);
  printf("Leap second offset: %d\n", leap_second);

  unsigned char fl0 = *(buffer + UCCM_FL0_POS);
  unsigned char fl1 = *(buffer + UCCM_FL1_POS);
  unsigned char fl2 = *(buffer + UCCM_FL2_POS);
  unsigned char fl3 = *(buffer + UCCM_FL3_POS);

  if (fl0&UCCM_FL0_LOCK_BIT) {
    printf("GPS locked.\n");
  }

  if (fl0&UCCM_FL0_LEAP_ANNOUNCE_BIT) {
    printf("Leap second announcement.\n");
  }

  if (fl0&UCCM_FL0_WARMUP_BIT) {
    printf("GPS warmup.\n");
  }

  if (fl1&UCCM_FL1_INIT_BIT) {
    printf("GPS initialisation.\n");
  }

  if (fl1&UCCM_FL1_GPS_TIME_BIT) {
    printf("Have GPS time.\n");
  }

  if (fl1&UCCM_FL1_LOW_VOLTAGE_BIT) {
    printf("Low voltage error.\n");
  }

  if (fl2&UCCM_FL2_SYMMETRICOM_BIT) {
    printf("Symmetricom hardware.\n");
  }

  if (fl2&UCCM_FL2_TRIMBLE_BIT) {
    printf("Trimble hardware.\n");
  }

  if (fl2&UCCM_FL2_SURVEY_BIT) {
    printf("GPS survey on.\n");
  }

  if (fl3&UCCM_FL3_NO_ANTENNA_BIT) {
    printf("Antenna not connected.\n");
  }

  if (fl3&UCCM_FL3_NO_GPS_SIGNAL_BIT) {
    printf("GPS signal not detected.\n");
  }
}

int main() {
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyUSB0", O_RDONLY);

  // Create new termios struc, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 57600
  cfsetispeed(&tty, B57600);
  cfsetospeed(&tty, B57600);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      return 1;
  }

  // Allocate memory for read buffer, set size according to your needs
  unsigned char read_buf [256];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  unsigned char* pos = read_buf;
  while (1) {
    int num_bytes = read(serial_port, pos, read_buf - pos + sizeof(read_buf));
    pos += num_bytes;
    if (num_bytes == 0) {
      if ((pos - read_buf) == 44) {
	handle_uccm(read_buf);
      }
      pos = read_buf;
    }
    if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      return 1;
    }
  }
  
  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  //  printf("Read %i bytes. Received message: %s", num_bytes, read_buf);

  close(serial_port);
  return 0; // success
}
