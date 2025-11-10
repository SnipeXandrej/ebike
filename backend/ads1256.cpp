#include "ads1256.hpp"

void ADS1256::initPins() {
	printf("Initializing pins\n");

	pinMode(ADS1256_DRDY, INPUT);
	pinMode(ADS1256_RESET, OUTPUT);
	pinMode(ADS1256_CS, OUTPUT);
	pinMode(ADS1256_PDWN, OUTPUT);

	digitalWrite(ADS1256_CS, HIGH); // pi.write(22, 1)    // ADS1256 /CS deselected
	digitalWrite(ADS1256_PDWN, HIGH); // pi.write(27, 1)    // ADS1256 /PDWN high
	digitalWrite(ADS1256_RESET, HIGH); // pi.write(18, 1)    // ADS1256 /RESET high

	digitalWrite(ADS1256_RESET, LOW);//pi.write(18, 0)    # ADS1256 /RESET low
	usleep(1000); // wait 1 msec
	digitalWrite(ADS1256_RESET, HIGH); //pi.write(18, 1)    # ADS1256 /RESET high
	usleep(500 * 1000); // wait 500 msec
}

/**
 *  according to the datasheet t6 must be at least 50*1/CLKIN = 6.5µs
 *  at a clock rate of 200kHz the time between clock low and next clock rise would be 5µs which means we would have to wait for just 1.5µs
 */
void ADS1256::delayDATA(void)
{
	/*
	   Delay from last SCLK edge for DIN to first SCLK rising edge for DOUT: RDATA, RDATAC,RREG Commands
	   min  50   CLK = 50 * 0.13uS = 6.5uS
	 */
	//bsp_DelayUS(10);        /* The minimum time delay 6.5us */
	// usleep(7);
	delayMicroseconds(7);
}


/**
 * delay time wait for automatic calibration
 *       parameter:  NULL
 *       The return value:  NULL
 * wait until ADS1256_DRDY goes to zero. using a timeout of 0.5s
 */
void ADS1256::waitDRDY() {
	while (digitalRead(ADS1256_DRDY)) {
		usleep(250);
	}
}

/**
 * read ADC value
 * @return sample value
 */
int32_t ADS1256::readData()
{
	uint32_t ret = 0;
	static uint8_t buf[3];

	digitalWrite(ADS1256_CS, LOW) ; //CS_0(); /* SPI  cs  = 0 */

	//ADS1256_Send8Bit(CMD_RDATA);    /* read ADC command  */
	buf[0]=ADS1256_CMD_RDATA;
	assert(write(ADS1256_spiHandle, buf, 1) == 1);

	delayDATA();    /*delay time  */

	/*Read the sample results 24bit*/
	//buf[0] = ADS1256_Recive8Bit();
	//buf[1] = ADS1256_Recive8Bit();
	//buf[2] = ADS1256_Recive8Bit();
	assert(read(ADS1256_spiHandle, buf, 3) == 3);

	ret = ((uint32_t)buf[0] << 16) & 0x00FF0000;
	ret |= ((uint32_t)buf[1] << 8);  /* Pay attention to It is wrong   read |= (buf[1] << 8) */
	ret |= buf[2];

	digitalWrite(ADS1256_CS, HIGH); // CS_1(); /* SPIƬѡ = 1 */

	/* Extend a signed number*/
	if (ret & 0x800000)
	{
		ret |= 0xFF000000;
	}

	return (int32_t)ret;
}

/**
 * Read the corresponding register
 * @param _RegID: register  ID
 * @return read register value
 */
uint8_t ADS1256::readReg(uint8_t _regID)
{

	digitalWrite(ADS1256_CS, LOW) ; //CS_0(); /* SPI  cs  = 0 */
/*
	ADS1256_Send8Bit(CMD_RREG | _regID);    // Write command register
	ADS1256_Send8Bit(0x00); // Write the register number
*/
	unsigned char buffer[2];
	buffer[0] = ADS1256_CMD_RREG | _regID;
	buffer[1] = 0;
	write(ADS1256_spiHandle, buffer, sizeof(buffer));

	delayDATA();    //delay time

	// uint8_t read = ADS1256_Recive8Bit();    /* Read the register values */
	if(read(ADS1256_spiHandle, buffer, 1) != 1) {
		perror("error reading data");
		exit(1);
	}
	digitalWrite(ADS1256_CS, HIGH); // CS_1(); /* SPI   cs  = 1 */

	return buffer[0];
}

/**
 * ADS1256_ReadChipID - Read the chip ID
 * @return four high status register
 */
uint8_t ADS1256::readChipID()
{
	uint8_t id;

	waitDRDY();
	id = readReg(ADS1256_REG_STATUS);
	return (id >> 4);
}

void ADS1256::init(int *spiHandle, int spiSpeed) {
	cfg_spiSpeed = spiSpeed;
	ADS1256_spiHandle = *spiHandle;
	ADS1256_spiHandle = openSPI(cfg_spiSpeed);

	waitDRDY();

	int id=readChipID();
	printf("ADS1256 chip ID: %d\n", id);

	if (ADS1256_RESET != 0) {
		digitalWrite(ADS1256_RESET, LOW);
		delay(200);
		digitalWrite(ADS1256_RESET, HIGH);  //RESET is set to high
		delay(1000);
	}

	if (ADS1256_PDWN != 0) {
		digitalWrite(ADS1256_PDWN, HIGH);  //SYNC is set to high
	}

	// 0xfe: soft-reset command
	digitalWrite(ADS1256_CS, LOW);
	waitDRDY();
	unsigned char data_sr[1];
	data_sr[0] = '\xfe';
	write(ADS1256_spiHandle, data_sr, 1);
	usleep(300000);

	// set register 00 (STATUS) reg.00,one byte,no autocal, buffer=2
	waitDRDY();
	digitalWrite(ADS1256_CS, LOW);
	unsigned char data_status[5];
	data_status[0] = '\xfc';
	data_status[1] = '\x00';
	data_status[2] = (ADS1256_CMD_WREG | ADS1256_REG_STATUS);
	data_status[3] = '\x00';
	data_status[4] = (char)(cfg_ADS1256_input_buffer << 1); /*buffer enable*/
	write(ADS1256_spiHandle, data_status, 5);
	usleep(100000); // wait 0.1 msec

	// set register 02 (ADCON)
	waitDRDY();
	digitalWrite(ADS1256_CS, LOW);
	unsigned char data_adcon[5];
	data_adcon[0] = '\xfc';
	data_adcon[1] = '\x00';
	data_adcon[2] = (ADS1256_CMD_WREG | ADS1256_REG_ADCON);
	data_adcon[3] = '\x00';
	data_adcon[4] = cfg_ADS1256_input_gain;
	write(ADS1256_spiHandle, data_adcon, 5);
	usleep(100000); // wait 0.1 msec

	// register 03 (DRATE) reg.03,one byte,10 samples per secondc
	waitDRDY();
	digitalWrite(ADS1256_CS, LOW);
	unsigned char data_drate[5];
	data_drate[0] = '\xfc';
	data_drate[1] = '\x00';
	data_drate[2] = (ADS1256_CMD_WREG | ADS1256_REG_DRATE);
	data_drate[3] = '\x00';
	data_drate[4] = cfg_ADS1256_sample_rate;
	write(ADS1256_spiHandle, data_drate, 5);
	usleep(100000); // wait 0.1 msec
}

int ADS1256::openSPI(int spiSpeed) {
	cfg_spiSpeed=spiSpeed;
	int fd;
	if ((fd = open ("/dev/spidev0.0", O_RDWR)) < 0) {
		printf("error opening spi\n");
		exit(1);
	}
	printf("fd: %d\n", fd);
	__u8 mode=-1;
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		printf("error reading spi mode\n");
		exit(1);
	}
	printf("current mode: %d\n", mode);

	int speed=-1;
	if(ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0) {
		printf("error reading spi speed\n");
		exit(1);
	}
	printf("current speed: %d\n", speed);
	close(fd);

	int spiFd=wiringPiSPISetupMode(/* channel */ SPICHANNEL , /* speed */ spiSpeed, SPI_MODE_1) ;
	//int spiChannel=wiringPiSPISetup(/* channel */ SPICHANNEL , /* speed */ 200000) ;

	if(spiFd < 0) {
		fprintf(stderr, "error: %s\n", strerror(errno));
		exit(1);
	}

	printf("spiFd: %d\n", spiFd);
	if(ioctl(fd, SPI_IOC_RD_MODE, &mode) < 0) {
		printf("error reading spi mode\n");
		exit(1);
	}
	printf("current mode: %d\n", mode);

	return spiFd;
}

void ADS1256::writeRegister(uint8_t registerAddress, uint8_t registerValueToWrite) {
	digitalWrite(ADS1256_CS, LOW);  //CS must stay LOW during the entire sequence [Ref: P34, T24]
	delayDATA();  // see t6 in the datasheet
	unsigned char data[3];
	data[0] = registerAddress;
	data[1] = '\x00';
	data[2] = registerValueToWrite;
	write(ADS1256_spiHandle, data, 3);

	digitalWrite(ADS1256_CS, HIGH);
	delay(200);
}

double ADS1256::convertToVoltage(int input) {
	// if the 24th digit (sign) is 1, the number is negative
	if (input >> 23 == 1)
	{
		input = input - 16777216; // 16777216 - (2^24)
	}

	return ((2.0 * VREF) / 8388608.0) * (input) / (pow(2, cfg_ADS1256_input_gain)); // 8388608 = 2^23
}

int ADS1256::readChannel(int channel) {
	waitDRDY();
	digitalWrite(ADS1256_CS, LOW);

	unsigned char databyte[6];
	databyte[0] = (ADS1256_CMD_WREG | ADS1256_REG_MUX);
	databyte[1] = '\x00';
	databyte[2] = (unsigned char)((channel << 4) | (1 << 3)); //(((channel+1) % cfg_max_channels) << 4 | 8);
	databyte[3] = ADS1256_CMD_SYNC;
	databyte[4] = ADS1256_CMD_WAKEUP;
	databyte[5] = ADS1256_CMD_RDATA;
	write(ADS1256_spiHandle, databyte, 6);

	delayDATA();

	read(ADS1256_spiHandle, databyte, 3);

	digitalWrite(ADS1256_CS, HIGH);

	int output = (databyte[0] << 16 | databyte[1] << 8 | databyte[2]);
	return output;
}

int ADS1256::readDiffChannel(int channel) {
	int byte = 0;
	switch (channel) {
	case 0:
		byte = (0 << 4) | 1;
		break;
	case 1:
		byte = (2 << 4) | 3;
		break;
	case 2:
		byte = (4 << 4) | 5;
		break;
	case 3:
		byte = (6 << 4) | 7;
		break;
	}

	waitDRDY();
	digitalWrite(ADS1256_CS, LOW);

	unsigned char databyte[6];
	databyte[0] = (ADS1256_CMD_WREG | ADS1256_REG_MUX);
	databyte[1] = '\x00';
	databyte[2] = (unsigned char)(byte);
	databyte[3] = ADS1256_CMD_SYNC;
	databyte[4] = ADS1256_CMD_WAKEUP;
	databyte[5] = ADS1256_CMD_RDATA;
	write(ADS1256_spiHandle, databyte, 6);

	delayDATA();

	read(ADS1256_spiHandle, databyte, 3);

	digitalWrite(ADS1256_CS, HIGH);

	int output = (databyte[0] << 16 | databyte[1] << 8 | databyte[2]);
	return output;
}