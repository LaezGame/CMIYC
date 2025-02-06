#include <stdio.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>
#include <cstring>
#include <iostream>
#include <errno.h>


// Define Constants
const char *uart_target = "/dev/ttyUSB0";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     MSG_LEN        6
#define     BAUDRATE       B9600

// Define methods
void uart_setup(int &fid);
bool write_serial(int fid, std::string msg);
std::string read_serial(int fid, int message_length);

int main()
{
    int fid= -1;
    
    uart_setup(fid);
    
    if (fid == -1)
    {
        std::cout << "Error - Unable to open UART.  Ensure it is not in use by another application\n";
    }

    usleep(500000);   // 0.5 sec delay

    std::cout << write_serial(fid, "S\n") << std::endl;

    usleep(1000000);  // 1 sec delay
    
    std::string input;

    while(true) {
		input = read_serial(fid, MSG_LEN);
		if (input.size() == MSG_LEN) {
			std::cout << input << std::endl;
		}
	}

    close(fid);
}

void uart_setup(int &fid) {
    // SETUP SERIAL WORLD
    struct termios  port_options;   // Create the structure

    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port


    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );
    
    if (fid == -1)
    {
        std::cerr << "Error - Unable to open UART.  Ensure it is not in use by another application\n";
    }

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(1000);  // 1 sec delay

    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)

    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB)
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,

    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 10;           // Wait one second

    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 ) {
        std::cout << "\nERROR in Setting port attributes" << std::endl;
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);
}

bool write_serial(int fid, std::string msg) {
    unsigned char tx_buffer[msg.size()];
    unsigned char *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    for(char i : msg) {
        *p_tx_buffer++ = i;
    }

    if (fid != -1)
    {
        int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
        usleep(1000);   // .001 sec delay
        if (count < 0)  return false; // write failed
    }
    return true;
}

std::string read_serial(int fid, int message_length) {
    char rx_buffer[message_length];
    std::string serial_message;
    bool          pickup = true;
    int           rx_length;
    int           nread = 0;

    tcflush(fid, TCIOFLUSH);

    usleep(1000);   // .001 sec delay

    // Now ready to recieve message
    serial_message.reserve(message_length);

    while (pickup && fid != -1) {
        int rx_length = read(fid, (void*)rx_buffer, message_length);
		
		if (rx_length == -1) {
			std::cerr << "Read error: " << strerror(errno) << std::endl;
			switch (errno) {
				case EAGAIN:
					std::cerr << "The file descriptor is set to non-blocking and no data is available." << std::endl;
					break;
				case EBADF:
					std::cerr << "The file descriptor is not valid or not open for reading." << std::endl;
					break;
				case EFAULT:
					std::cerr << "Buffer points to an invalid memory location." << std::endl;
					break;
				case EINTR:
					std::cerr << "The call was interrupted by a signal before any data was read." << std::endl;
					break;
				case EIO:
					std::cerr << "I/O error occurred." << std::endl;
					break;
				default:
					std::cerr << "Unknown error occurred." << std::endl;
			}
		}
		
        if (rx_length > 0) {
            for (int i = 0; i < rx_length; i++) {
                if (nread < NSERIAL_CHAR - 1) {
                    // Append each byte to the message buffer
                    serial_message += rx_buffer[i];
                    nread++;

                    // Check for terminator, newline
                    if (rx_buffer[i] == '\n') {
                        pickup = false;  // End receiving if terminator is found
                        break;
                    }
                } else {
                    // Buffer overflow, stop collecting data
                    std::cout << "Buffer overflow detected! Message too long.\n" << std::endl;
                    pickup = false;
                    break;
                }
            }

            // Debug: Print raw data
            /*printf("Raw data: ");
            for (int i = 0; i < rx_length; i++) {
                printf("%02X ", rx_buffer[i]);  // Print as hexadecimal
            }
            printf("\n");*/
        }
    }
    return serial_message;
}
