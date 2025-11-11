#ifndef ADS1256_HPP
#define ADS1256_HPP

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <wiringPiSPI.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <wiringPi.h>
#include <fcntl.h>

#include <string>
#include <sstream>

#define SPICHANNEL 1

//Differential inputs
#define DIFF_0_1 0b00000001  //A0 + A1 as differential input
#define DIFF_2_3 0b00100011  //A2 + A3 as differential input
#define DIFF_4_5 0b01000101  //A4 + A5 as differential input
#define DIFF_6_7 0b01100111  //A6 + A7 as differential input

//Single-ended inputs
#define SING_7 0b1000  //A0 + GND (common) as single-ended input
#define SING_0 0b1001  //A1 + GND (common) as single-ended input
#define SING_1 0b1010  //A2 + GND (common) as single-ended input
#define SING_2 0b1011  //A3 + GND (common) as single-ended input
#define SING_3 0b1100  //A4 + GND (common) as single-ended input
#define SING_4 0b1101  //A5 + GND (common) as single-ended input
#define SING_5 0b1110  //A6 + GND (common) as single-ended input
#define SING_6 0b1111  //A7 + GND (common) as single-ended input

// PGA
#define	ADS1256_GAIN_1 	0  /* GAIN   1 ± 5 V  */
#define	ADS1256_GAIN_2 	1  /* GAIN   2  ± 2.5 ± */
#define	ADS1256_GAIN_4 	2  /* GAIN   4 ± 1.25 ± */
#define	ADS1256_GAIN_8  3  /* GAIN   8 ± 625 mV */
#define	ADS1256_GAIN_16 4  /* GAIN  16 ± 312.5 mV */
#define	ADS1256_GAIN_32 5  /* GAIN  32 + 156.25 mV */
#define	ADS1256_GAIN_64 6  /* GAIN  64  ± 78.125 mV  */

/* Sampling speed*/
/*
  11110000 = 30,000SPS (default)
  11100000 = 15,000SPS
  11010000 = 7,500SPS
  11000000 = 3,750SPS
  10110000 = 2,000SPS
  10100001 = 1,000SPS
  10010010 = 500SPS
  10000010 = 100SPS
  01110010 = 60SPS
  01100011 = 50SPS
  01010011 = 30SPS
  01000011 = 25SPS
  00110011 = 15SPS
  00100011 = 10SPS
  00010011 = 5SPS
  00000011 = 2.5SPS
*/
#define ADS1256_DRATE_30000SPS 0xF0
#define ADS1256_DRATE_15000SPS 0xE0
#define ADS1256_DRATE_7500SPS 0xD0
#define ADS1256_DRATE_3750SPS 0xC0
#define ADS1256_DRATE_2000SPS 0xB0
#define ADS1256_DRATE_1000SPS 0xA1
#define ADS1256_DRATE_500SPS 0x92
#define ADS1256_DRATE_100SPS 0x82
#define ADS1256_DRATE_60SPS 0x72
#define ADS1256_DRATE_50SPS 0x63
#define ADS1256_DRATE_30SPS 0x53
#define ADS1256_DRATE_25SPS 0x43
#define ADS1256_DRATE_15SPS 0x33
#define ADS1256_DRATE_10SPS 0x20
#define ADS1256_DRATE_5SPS 0x13
#define ADS1256_DRATE_2SPS 0x03

/*Register definition£º Table 23. Register Map --- ADS1256 datasheet Page 30*/
	/*Register address, followed by reset the default values */
#define ADS1256_REG_STATUS 0  // x1H
#define ADS1256_REG_MUX    1  // 01H
#define ADS1256_REG_ADCON  2  // 20H
#define ADS1256_REG_DRATE  3  // F0H
#define ADS1256_REG_IO     4  // E0H
#define ADS1256_REG_OFC0   5  // xxH
#define ADS1256_REG_OFC1   6  // xxH
#define ADS1256_REG_OFC2   7  // xxH
#define ADS1256_REG_FSC0   8  // xxH
#define ADS1256_REG_FSC1   9  // xxH
#define ADS1256_REG_FSC2   10 // xxH

/* Command definition£º TTable 24. Command Definitions --- ADS1256 datasheet Page 34 */
#define ADS1256_CMD_WAKEUP   0x00 // Completes SYNC and Exits Standby Mode 0000  0000 (00h)
#define ADS1256_CMD_RDATA    0x01 // Read Data 0000  0001 (01h)
#define ADS1256_CMD_RDATAC   0x03 // Read Data Continuously 0000   0011 (03h)
#define ADS1256_CMD_SDATAC   0x0F // Stop Read Data Continuously 0000   1111 (0Fh)
#define ADS1256_CMD_RREG     0x10 // Read from REG rrr 0001 rrrr (1xh)
#define ADS1256_CMD_WREG     0x50 // Write to REG rrr 0101 rrrr (5xh)
#define ADS1256_CMD_SELFCAL  0xF0 // Offset and Gain Self-Calibration 1111    0000 (F0h)
#define ADS1256_CMD_SELFOCAL 0xF1 // Offset Self-Calibration 1111    0001 (F1h)
#define ADS1256_CMD_SELFGCAL 0xF2 // Gain Self-Calibration 1111    0010 (F2h)
#define ADS1256_CMD_SYSOCAL  0xF3 // System Offset Calibration 1111   0011 (F3h)
#define ADS1256_CMD_SYSGCAL  0xF4 // System Gain Calibration 1111    0100 (F4h)
#define ADS1256_CMD_SYNC     0xFC // Synchronize the A/D Conversion 1111   1100 (FCh)
#define ADS1256_CMD_STANDBY  0xFD // Begin Standby Mode 1111   1101 (FDh)
#define ADS1256_CMD_RESET    0xFE // Reset to Power-Up Values 1111   1110 (FEh)

class ADS1256 {
public:
	void init(int *spiHandle, int spiSpeed, int _ADS1256_DRDY, int ADS1256_RESET, int ADS1256_CS, int ADS1256_PDWN, char _inputgain, char _samplerate, bool _inputbuffer);

	void delayDATA(void);

	void waitDRDY();

	int32_t readData();

	uint8_t readReg(uint8_t _regID);

	uint8_t readChipID();

	int openSPI(int spiSpeed);

	void writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite);

	double convertToVoltage(int input);

	int readChannel(int channel);

	int readDiffChannel(int channel);

	int cfg_spiSpeed=-1;
	float VREF = 2.5f;
	char cfg_ADS1256_input_gain=ADS1256_GAIN_1;
	char cfg_ADS1256_sample_rate=ADS1256_DRATE_1000SPS;
	bool cfg_ADS1256_input_buffer=false;

	int ADS1256_spiHandle=-1;

	int ADS1256_DRDY;
	int ADS1256_RESET;
	int ADS1256_CS;
	int ADS1256_PDWN;
};

#endif
