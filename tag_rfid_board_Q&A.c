#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#include <termios.h> // Add header file related to UART

#define SPI_DEVICE "/dev/spidev0.0" // SPI device path (X: bus number, Y: device number)
#define UART_DEVICE "/dev/ttyS0" // UART device path
#define SS_PIN 328 // SPI SS(Chip Select) GPIO pin number
#define MOSI_PIN 339
#define MISO_PIN 340
#define SCK_PIN 338

#define MFRC522_PICC_TYPE_MIFARE_MINI 0x09
#define MFRC522_PICC_TYPE_MIFARE_1K 0x04
#define MFRC522_PICC_TYPE_MIFARE_4K 0x02

int spi_fd;
int gpio_fd;
int uart_fd; // add UART file descriptor

// initialize SPI communication
int SpiInit() {
     spi_fd = open(SPI_DEVICE, O_RDWR);

     // set SPI mode
     uint8_t mode = SPI_MODE_0;
     ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);

     // set the maximum baud rate (1MHz)
     uint32_t speed = 1000000;
     ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
     return 0;
}

// send and receive SPI data
int SpiTransfer(uint8_t* tx_data, uint8_t* rx_data, int len) {
     struct spi_ioc_transfer spi_transfer = {
         .tx_buf = (unsigned long)tx_data,
         .rx_buf = (unsigned long)rx_data,
         .len = len,
         .delay_usecs = 0,
         .speed_hz = 1000000,
         .bits_per_word = 8,
     };
     if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 0) {
         printf("SPI transfer error\n");
         return -1;
     }
     return 0;
}

// Initialize MFRC522
void MFRC522Init() {
     uint8_t tx_data[] = {0x0F}; // Data value for sending softreset command to MFRC522
     uint8_t rx_data[sizeof(tx_data)];

     if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {
         printf("Failed to send SPI message\n");
         return;
     }
     usleep(5000); // wait a while
}

// Check if the card exists
int PiccIsNewCardPresent() {
     uint8_t tx_data[] = {0x52}; // Data pattern value for transmitting the corresponding function to MFRC522, 0x52 : find all the cards antenna area
     uint8_t rx_data[sizeof(tx_data)];
     memset(rx_data, 0, sizeof(rx_data));
     if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {
         printf("Failed to transfer data\n");
         return -1;
     }
     return rx_data[1];
}

// read card UID information
int PiccReadCardSerial(uint8_t* uid) {
     uint8_t tx_data[] = {0x93}; // Data pattern value to transmit the corresponding function to MFRC522, 0x93 : election card
     uint8_t rx_data[sizeof(tx_data)];
     memset(rx_data, 0, sizeof(rx_data));
     if (SpiTransfer(tx_data, rx_data, sizeof(tx_data)) < 0) {
         printf("Failed to transfer data\n");
         return -1;
     }
     memcpy(uid, &rx_data[1], 4);
     return 0;
}

// get card type
uint8_t PiccGetType(uint8_t sak) { // select Acknowledge, returns a 7-bit value representing card type information
     return sak & 0x7F;
}

// get card type name
const char* PiccGetTypeName(uint8_t piccType) {
     switch (piccType) {
         case MFRC522_PICC_TYPE_MIFARE_MINI:
             return "MIFARE Mini";
         case MFRC522_PICC_TYPE_MIFARE_1K:
             return "MIFARE 1K";
         case MFRC522_PICC_TYPE_MIFARE_4K:
             return "MIFARE 4K";
         default:
             return "Unknown";
     }
}

// Convert to hexadecimal and output
void printHex(uint8_t* data, uint8_t length) {
     for (uint8_t i = 0; i < length; i++) {
         printf("%02X", data[i]);
     }
}

// Initialize UART
int UartInit() {
     uart_fd = open(UART_DEVICE, O_RDWR | O_NOCTTY);
     if (uart_fd < 0) {
         printf("Failed to open UART\n");
         return -1;
     }

     struct termios options;
     tcgetattr(uart_fd, &options); // import current setting

     // set baud rate (9600 bps)
     cfsetispeed(&options, B9600);
     cfsetospeed(&options, B9600);

     // set data bit, parity bit, stop bit
     options.c_cflag &= ~PARENB;
     options.c_cflag &= ~CSTOPB;
     options.c_cflag &= ~CSIZE;
     options.c_cflag |= CS8;

     // set control options
     // options.c_cflag &= ~CRTSCTS;
     options.c_cflag |= CREAD | CLOCAL;
     options.c_iflag &= ~(IXON | IXOFF | IXANY);
     options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
     options.c_oflag &= ~OPOST;

     tcsetattr(uart_fd, TCSANOW, &options); // apply new settings to uart_fd

     return 0;
}

// Send data to PC via UART
void WriteUidUart(const char* data) {
     write(uart_fd, data, strlen(data));
}


int main() {
     // set SS pin
     int ss_gpio_fd;
     char ss_gpio_path[64];

// open SS pin
     snprintf(ss_gpio_path, sizeof(ss_gpio_path), "/sys/class/gpio/gpio%d/value", SS_PIN);
     ss_gpio_fd = open(ss_gpio_path, O_WRONLY); // O_WRONLY : Open file write-only flag

// set SS pin as output
     write(ss_gpio_fd, "out", 3);

// MOSI pin settings
     int mosi_gpio_fd;
     char mosi_gpio_path[64];

// open MOSI pin
     snprintf(mosi_gpio_path, sizeof(mosi_gpio_path), "/sys/class/gpio/gpio%d/value", MOSI_PIN);
     mosi_gpio_fd = open(mosi_gpio_path, O_WRONLY);

// set the MOSI pin as an output
     write(mosi_gpio_fd, "out", 3);

// close MOSI pin
     close(mosi_gpio_fd);

// set MISO pin
     int miso_gpio_fd;
     char miso_gpio_path[64];

// open MISO pin
     snprintf(miso_gpio_path, sizeof(miso_gpio_path), "/sys/class/gpio/gpio%d/value", MISO_PIN);
     miso_gpio_fd = open(miso_gpio_path, O_WRONLY);

// set MISO pin as input
     write(miso_gpio_fd, "in", 2);

// close MISO pin
     close(miso_gpio_fd);

// set SCK pin
     int sck_gpio_fd;
     char sck_gpio_path[64];

// open SCK pin
     snprintf(sck_gpio_path, sizeof(sck_gpio_path), "/sys/class/gpio/gpio%d/value", SCK_PIN);
     sck_gpio_fd = open(sck_gpio_path, O_WRONLY);

// set SCK pin as output
     write(sck_gpio_fd, "out", 3);

// close SCK pin
     close(sck_gpio_fd);

     // initialize SPI communication
     if (SpiInit() != 0) {
         return -1;
     }

     // Initialize MFRC522
     MFRC522Init();

     // Initialize UART communication
     if (UartInit() != 0) {
         return -1;
     }

     uint8_t nuidPICC[4] = {0}; // Save the previous card UID

     while (1) {
         // If the card is recognized, move on to the next step, otherwise do not run any further
         if (!PiccIsNewCardPresent())
             continue;

         // If the ID has been read, move on to the next one, otherwise don't run any more
         if (!PiccReadCardSerial(nuidPICC))
             continue;

         // Write value to SS pin (set to 0)
         write(ss_gpio_fd, "0", 1); // low

         printf("PICC type: ");

         // read the card type
         uint8_t piccType = PiccGetType(nuidPICC[0]);

         // output to monitor
         printf("%s\n", PiccGetTypeName(piccType));

         // check if it is a MIFARE method and return otherwise
         if (piccType != MFRC522_PICC_TYPE_MIFARE_MINI &&
             piccType != MFRC522_PICC_TYPE_MIFARE_1K &&
             piccType != MFRC522_PICC_TYPE_MIFARE_4K) {
             printf("Your tag is not of type MIFARE Classic.\n");
             continue;
         }

         // If it is different from the RF card recognized just before. That is, to prevent duplicate card detection.
         if (nuidPICC[0] != nuidPICC[0] ||
             nuidPICC[1] != nuidPICC[1] ||
             nuidPICC[2] != nuidPICC[2] ||
             nuidPICC[3] != nuidPICC[3]) {
             printf("A new card has been detected.\n");

             // I saved the ID
             memcpy(nuidPICC, nuidPICC, sizeof(nuidPICC));

             // monitor output
             printf("The NUID tag is:\n");
             printf("In hex: ");
             // Convert to hexadecimal and output
             printHex(nuidPICC, sizeof(nuidPICC));
             printf("\n");

             // Send card information to PC via UART
             char uartData[9];
             snprintf(uartData, sizeof(uartData), "%02X%02X%02X%02X\n", nuidPICC[0], nuidPICC[1], nuidPICC[2], nuidPICC[3]);
             WriteUidUart(uartData);
         }

         // Write value to SS pin (set to 1)
         write(ss_gpio_fd, "1", 1); // high

         // exit PICC
         usleep(100000);

         // When the type is read, the while infinite loop ends
         if (piccType != MFRC522_PICC_TYPE_MIFARE_MINI &&
             piccType != MFRC522_PICC_TYPE_MIFARE_1K &&
             piccType != MFRC522_PICC_TYPE_MIFARE_4K) {
             break;
         }
     }

     close(spi_fd);
     close(uart_fd);

     // close SS pin
     close(ss_gpio_fd);
     return 0;
}
