/*  MPU9250.cpp
*
*   Class to control the MPU9250 motion sensor
*
*/

#include "MPU9250.h"
#include <SPI.h>

#define CSPIN 4

void MPU9250Class::init()
{
	initvars();

	// Fire up the SPI interface
	SPI.begin(CSPIN);
	SPI.setClockDivider(CSPIN, 84);

	// Read the WHO_AM_I register to test communication
	uint8_t readval = readByte(WHO_AM_I_MPU9250);
	if (readval != 0x71)
	{
		Serial3.println("...MPU9250 not found");
		return;
	}
	Serial3.println("...MPU9250 found");

	MPU9250SelfTest(SelfTest); // Start by performing self test and reporting values
	Serial3.print("x-axis self test: acceleration trim within : "); Serial3.print(SelfTest[0], 1); Serial3.println("% of factory value");
	Serial3.print("y-axis self test: acceleration trim within : "); Serial3.print(SelfTest[1], 1); Serial3.println("% of factory value");
	Serial3.print("z-axis self test: acceleration trim within : "); Serial3.print(SelfTest[2], 1); Serial3.println("% of factory value");
	Serial3.print("x-axis self test: gyration trim within : "); Serial3.print(SelfTest[3], 1); Serial3.println("% of factory value");
	Serial3.print("y-axis self test: gyration trim within : "); Serial3.print(SelfTest[4], 1); Serial3.println("% of factory value");
	Serial3.print("z-axis self test: gyration trim within : "); Serial3.print(SelfTest[5], 1); Serial3.println("% of factory value");

	calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

	Serial3.println("MPU9250 bias");
	Serial3.println("\tx\ty\tz");

	Serial3.print("\t"); Serial3.print((int)(1000 * accelBias[0]));
	Serial3.print("\t"); Serial3.print((int)(1000 * accelBias[1]));
	Serial3.print("\t"); Serial3.print((int)(1000 * accelBias[2]));
	Serial3.println("\tmg");

	Serial3.print("\t"); Serial3.print(gyroBias[0], 1);
	Serial3.print("\t"); Serial3.print(gyroBias[1], 1);
	Serial3.print("\t"); Serial3.print(gyroBias[2], 1);
	Serial3.print("\t"); Serial3.println("o/s");

	initMPU9250();
	Serial3.println("MPU9250 initialized for active data mode....");

	// Start the interface to the Magnetometer (AK8963)
	writeByte(I2C_MST_CTRL, 0x0D);					// I2C Speed 400 kHz
	writeByte(USER_CTRL, 0x20);						// Enable AUX

	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	readval = readMagByte(WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
	Serial3.print("AK8963 "); Serial3.print("I AM "); Serial3.print(readval, HEX); Serial3.print(" I should be "); Serial3.println(0x48, HEX);

	// Get magnetometer calibration from AK8963 ROM
	initAK8963(magCalibration); Serial3.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer


	Serial3.println("Calibration values: ");
	Serial3.print("X-Axis sensitivity adjustment value "); Serial3.println(magCalibration[0], 2);
	Serial3.print("Y-Axis sensitivity adjustment value "); Serial3.println(magCalibration[1], 2);
	Serial3.print("Z-Axis sensitivity adjustment value "); Serial3.println(magCalibration[2], 2);
}

void MPU9250Class::tilt_compensation(float ax, float ay, float az, float raw_mx, float raw_my, float raw_mz) 
{
	float accXnorm = ax / sqrt(ax * ax + ay * ay + az * az);
	float accYnorm = ay / sqrt(ax * ax + ay * ay + az * az);
	float pitch = asin(accXnorm);
	float roll = -asin(accYnorm / cos(pitch));
	float magXcomp = raw_mx * cos(pitch) + raw_mz * sin(pitch);
	float magYcomp = raw_mx * sin(roll) * sin(pitch) + raw_my * cos(roll) - raw_mz * sin(roll) * cos(pitch);
	float heading = 180 * atan2(magYcomp, magXcomp) / PI;
	SerialUSB.print("Compensated X");  SerialUSB.println(magXcomp);
	SerialUSB.print("Compensated Y");  SerialUSB.println(magYcomp);
	SerialUSB.print("Compensated Heading");  SerialUSB.println(heading);
}

void MPU9250Class::initvars()
{
	int i;
	Gscale = GFS_250DPS;
	Ascale = AFS_2G;
	Mscale = MFS_16BITS;
	Mmode = 0x02;

	for (i = 0; i < 3; i++)
	{
		magCalibration[i] = 0;
		gyroBias[i] = 0;
		accelBias[i] = 0;
		eInt[i] = 0.0f;
	}

	magbias[0] = 0; // +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
	magbias[1] = 0; //+120.;  // User environmental y-axis correction in milliGauss
	magbias[2] = 0; //+125.;  // User environmental z-axis correction in milliGauss

	GyroMeasError = PI * (40.0f / 180.0f);
	GyroMeasDrift = PI * (0.0f / 180.0f);
	beta = sqrt(3.0f / 4.0f) * GyroMeasError;
	zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;

	delt_t = 0;
	count = 0;
	sumCount = 0;
	deltat = 0.0f;
	sum = 0.0f;
	lastUpdate = 0;
	firstUpdate = 0;
	Now = 0;

	q[0] = 1.0f;
	for (i = 1; i < 4; i++)
		q[i] = 0.0f;
}

void MPU9250Class::loop()
{
	// If intPin goes high, all data registers have new data
	if (readByte(INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt
		readAccelData(accelCount);  // Read the x/y/z adc values
		getAres();

		// Now we'll calculate the accleration value into actual g's
		ax = (float)accelCount[0] * aRes; // - accelBias[0];  // get actual g value, this depends on scale being set
		ay = (float)accelCount[1] * aRes; // - accelBias[1];
		az = (float)accelCount[2] * aRes; // - accelBias[2];

		readGyroData(gyroCount);  // Read the x/y/z adc values
		getGres();

		// Calculate the gyro value into actual degrees per second
		gx = (float)gyroCount[0] * gRes;  // get actual gyro value, this depends on scale being set
		gy = (float)gyroCount[1] * gRes;
		gz = (float)gyroCount[2] * gRes;

		readMagData(magCount);  // Read the x/y/z adc values
		getMres();

		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental corrections
		mx = (float)magCount[0] * mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
		my = (float)magCount[1] * mRes*magCalibration[1] - magbias[1];
		mz = (float)magCount[2] * mRes*magCalibration[2] - magbias[2];
	}

	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;

	sum += deltat; // sum for averaging filter update rate
	sumCount++;

	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	// This is ok by aircraft orientation standards!
	// Pass gyro rate as rad/s
	//MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
	MahonyQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f, my, mx, mz);


	if (!AHRS) {
		delt_t = millis() - count;
		if (delt_t > 500) {
			if (SerialDebug) {

				// Print acceleration values in milligs
				SerialUSB.print("X-acceleration: "); SerialUSB.print(1000 * ax); SerialUSB.print(" mg ");
				SerialUSB.print("Y-acceleration: "); SerialUSB.print(1000 * ay); SerialUSB.print(" mg ");
				SerialUSB.print("Z-acceleration: "); SerialUSB.print(1000 * az); SerialUSB.println(" mg ");

				// Print gyro values in degree/sec
				SerialUSB.print("X-gyro rate: "); SerialUSB.print(gx, 3); SerialUSB.print(" degrees/sec ");
				SerialUSB.print("Y-gyro rate: "); SerialUSB.print(gy, 3); SerialUSB.print(" degrees/sec ");
				SerialUSB.print("Z-gyro rate: "); SerialUSB.print(gz, 3); SerialUSB.println(" degrees/sec");

				// Print mag values in degree/sec
				SerialUSB.print("X-mag field: "); SerialUSB.print(mx); SerialUSB.print(" mG ");
				SerialUSB.print("Y-mag field: "); SerialUSB.print(my); SerialUSB.print(" mG ");
				SerialUSB.print("Z-mag field: "); SerialUSB.print(mz); SerialUSB.println(" mG");

				tempCount = readTempData();  // Read the adc values
				temperature = ((float)tempCount) / 333.87 + 21.0; // Temperature in degrees Centigrade
				// Print temperature in degrees Centigrade
				SerialUSB.print("Temperature is ");  SerialUSB.print(temperature, 1);  SerialUSB.println(" degrees C"); // Print T values to tenths of s degree C
			}
			count = millis();
		}
	}
	else {

		// Serial print and/or display at 0.5 s rate independent of data rates
		delt_t = millis() - count;
		if (delt_t > 500) { // update LCD once per half-second independent of read rate

			if (SerialDebug) {
				SerialUSB.print("ax = "); SerialUSB.print((int)1000 * ax);
				SerialUSB.print(" ay = "); SerialUSB.print((int)1000 * ay);
				SerialUSB.print(" az = "); SerialUSB.print((int)1000 * az); SerialUSB.println(" mg");
				SerialUSB.print("gx = "); SerialUSB.print(gx, 2);
				SerialUSB.print(" gy = "); SerialUSB.print(gy, 2);
				SerialUSB.print(" gz = "); SerialUSB.print(gz, 2); SerialUSB.println(" deg/s");
				SerialUSB.print("mx = "); SerialUSB.print((int)mx);
				SerialUSB.print(" my = "); SerialUSB.print((int)my);
				SerialUSB.print(" mz = "); SerialUSB.print((int)mz); SerialUSB.println(" mG");

				SerialUSB.print("q0 = "); SerialUSB.print(q[0]);
				SerialUSB.print(" qx = "); SerialUSB.print(q[1]);
				SerialUSB.print(" qy = "); SerialUSB.print(q[2]);
				SerialUSB.print(" qz = "); SerialUSB.println(q[3]);
			}

			// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
			// In this coordinate system, the positive z-axis is down toward Earth.
			// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
			// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
			// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
			// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
			// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
			// applied in the correct order which for this configuration is yaw, pitch, and then roll.
			// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
			yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
			pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
			roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
			pitch *= 180.0f / PI;
			yaw *= 180.0f / PI;
			yaw -= 0.75; // Magnetic Declination in Cambridge UK is -0 deg 45' - http://www.magnetic-declination.com/Great%20Britain%20(UK)/Cambridge/897966.html
			roll *= 180.0f / PI;

			if (SerialDebug) {
				SerialUSB.print("Yaw:  \t"); SerialUSB.println(yaw, 2);
				SerialUSB.print("Pitch:\t"); SerialUSB.println(pitch, 2);
				SerialUSB.print("Roll: \t"); SerialUSB.println(roll, 2);

				SerialUSB.print("rate = "); SerialUSB.print((float)sumCount / sum, 2); SerialUSB.println(" Hz");
				tilt_compensation(ax, ay, az, mx, my, mz);
				SerialUSB.print("\x1b[11F");  // Scroll back 11 lines
			}

			// With these settings the filter is updating at a ~145 Hz rate using the Madgwick scheme and
			// >200 Hz using the Mahony scheme even though the display refreshes at only 2 Hz.
			// The filter update rate is determined mostly by the mathematical steps in the respective algorithms,
			// the processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
			// an ODR of 10 Hz for the magnetometer produce the above rates, maximum magnetometer ODR of 100 Hz produces
			// filter update rates of 36 - 145 and ~38 Hz for the Madgwick and Mahony schemes, respectively.
			// This is presumably because the magnetometer read takes longer than the gyro or accelerometer reads.
			// This filter update rate should be fast enough to maintain accurate platform orientation for
			// stabilization control of a fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
			// produced by the on-board Digital Motion Processor of Invensense's MPU6050 6 DoF and MPU9150 9DoF sensors.
			// The 3.3 V 8 MHz Pro Mini is doing pretty well!

			count = millis();
			sumCount = 0;
			sum = 0;
		}
	}

}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

void MPU9250Class::getMres() 
{
	switch (Mscale)
	{
		// Possible magnetometer scales (and their register bit settings) are:
		// 14 bit resolution (0) and 16 bit resolution (1)
	case MFS_14BITS:
		mRes = 10.*4912. / 8190.; // Proper scale to return milliGauss
		break;
	case MFS_16BITS:
		mRes = 10.*4912. / 32760.0; // Proper scale to return milliGauss
		break;
	}
}

void MPU9250Class::getGres() 
{
	switch (Gscale)
	{
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case GFS_250DPS:
		gRes = 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		gRes = 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		gRes = 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		gRes = 2000.0 / 32768.0;
		break;
	}
}

void MPU9250Class::getAres() 
{
	switch (Ascale)
	{
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
	case AFS_2G:
		aRes = 2.0 / 32768.0;
		break;
	case AFS_4G:
		aRes = 4.0 / 32768.0;
		break;
	case AFS_8G:
		aRes = 8.0 / 32768.0;
		break;
	case AFS_16G:
		aRes = 16.0 / 32768.0;
		break;
	}
}


void MPU9250Class::readAccelData(int16_t * destination)
{
	destination[0] = readWord(ACCEL_XOUT_H);
	destination[1] = readWord(ACCEL_YOUT_H);
	destination[2] = readWord(ACCEL_ZOUT_H);
}


void MPU9250Class::readGyroData(int16_t * destination)
{
	destination[0] = readWord(GYRO_XOUT_H);
	destination[1] = readWord(GYRO_YOUT_H);
	destination[2] = readWord(GYRO_ZOUT_H);
}

bool MPU9250Class::readMagData(int16_t * destination)
{
	int i;
	uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	if (readMagByte(AK8963_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		for (i = 0; i < 7; i++)
			rawData[i] = readMagByte(AK8963_XOUT_L + i);
		if (!(rawData[6] & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
			destination[0] = ((int16_t)rawData[1] << 8) | rawData[0];  // Turn the MSB and LSB into a signed 16-bit value
			destination[1] = ((int16_t)rawData[3] << 8) | rawData[2];  // Data stored as little Endian
			destination[2] = ((int16_t)rawData[5] << 8) | rawData[4];
			return true;
		}
	}
	return false;
}

int16_t MPU9250Class::readTempData()
{
	return readWord(TEMP_OUT_H);
}

void MPU9250Class::initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];  // x/y/z gyro calibration data stored here
	writeMagByte(AK8963_CNTL, 0x00); // Power down magnetometer
	writeMagByte(AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	rawData[0] = readMagByte(AK8963_ASAX);
	rawData[1] = readMagByte(AK8963_ASAY);
	rawData[2] = readMagByte(AK8963_ASAZ);
	destination[0] = (float)(rawData[0] - 128) / 256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	destination[1] = (float)(rawData[1] - 128) / 256. + 1.;
	destination[2] = (float)(rawData[2] - 128) / 256. + 1.;
	writeMagByte(AK8963_CNTL, 0x00); // Power down magnetometer
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	writeMagByte(AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
}

void MPU9250Class::initMPU9250()
{
	// wake up device
	writeByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
	delay(100); // Wait for all registers to reset

	// get stable time source
	writeByte(PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
	delay(200);

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
	// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
	// be higher than 1 / 0.0059 = 170 Hz
	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
	// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
	writeByte(CONFIG, 0x03);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	writeByte(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
								  // determined inset in CONFIG above

	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t c = readByte(GYRO_CONFIG);
	//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
	writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
	// writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

	// Set accelerometer full-scale range configuration
	c = readByte(ACCEL_CONFIG);
	//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
	writeByte(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	c = readByte(ACCEL_CONFIG2);
	writeByte(ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	writeByte(ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	writeByte(INT_PIN_CFG, 0x22);
	writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
	delay(100);
}


// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU9250Class::calibrateMPU9250(float * dest1, float * dest2)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	// reset device
	writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	writeByte(PWR_MGMT_1, 0x01);
	writeByte(PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	writeByte(INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(FIFO_EN, 0x00);      // Disable FIFO
	writeByte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	uint16_t  gyrosensitivity = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(USER_CTRL, 0x40);   // Enable FIFO
	writeByte(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	fifo_count = readWord(FIFO_COUNTH); // read FIFO sample count
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		accel_bias[0] += readFIFOWord(); // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += readFIFOWord();
		accel_bias[2] += readFIFOWord();
		gyro_bias[0] += readFIFOWord();
		gyro_bias[1] += readFIFOWord();
		gyro_bias[2] += readFIFOWord();
	}
	accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else { accel_bias[2] += (int32_t)accelsensitivity; }

	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4) & 0xFF;
	data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4) & 0xFF;

	// Push gyro biases to hardware registers
	writeByte(XG_OFFSET_H, data[0]);
	writeByte(XG_OFFSET_L, data[1]);
	writeByte(YG_OFFSET_H, data[2]);
	writeByte(YG_OFFSET_L, data[3]);
	writeByte(ZG_OFFSET_H, data[4]);
	writeByte(ZG_OFFSET_L, data[5]);

	// Output scaled gyro biases for display in the main program
	dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
	accel_bias_reg[0] = readWord(XA_OFFSET_H); // Read factory accelerometer trim values
	accel_bias_reg[1] = readWord(YA_OFFSET_H);
	accel_bias_reg[2] = readWord(ZA_OFFSET_H);

	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

	for (ii = 0; ii < 3; ii++) {
		if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0]) & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1]) & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2]) & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	writeByte(XA_OFFSET_H, data[0]);
	writeByte(XA_OFFSET_L, data[1]);
	writeByte(YA_OFFSET_H, data[2]);
	writeByte(YA_OFFSET_L, data[3]);
	writeByte(ZA_OFFSET_H, data[4]);
	writeByte(ZA_OFFSET_L, data[5]);

	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
	dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
	dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250Class::MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
	uint8_t selfTest[6];
	int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
	float factoryTrim[6];
	uint8_t FS = 0;

	writeByte(SMPLRT_DIV, 0x00);		// Set gyro sample rate to 1 kHz
	writeByte(CONFIG, 0x02);			// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	writeByte(GYRO_CONFIG, 1 << FS);	// Set full scale range for the gyro to 250 dps
	writeByte(ACCEL_CONFIG2, 0x02);		// Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	writeByte(ACCEL_CONFIG, 1 << FS);	// Set full scale range for the accelerometer to 2 g

	for (int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer
		aAvg[0] += readWord(ACCEL_XOUT_H);
		aAvg[1] += readWord(ACCEL_YOUT_H);
		aAvg[2] += readWord(ACCEL_ZOUT_H);

		gAvg[0] += readWord(GYRO_XOUT_H);
		gAvg[1] += readWord(GYRO_YOUT_H);
		gAvg[2] += readWord(GYRO_ZOUT_H);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}

	// Configure the accelerometer for self-test
	writeByte(ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	writeByte(GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	delay(25);  // Delay a while to let the device stabilize

	for (int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
		aSTAvg[0] += readWord(ACCEL_XOUT_H);
		aSTAvg[1] += readWord(ACCEL_YOUT_H);
		aSTAvg[2] += readWord(ACCEL_ZOUT_H);

		gSTAvg[0] += readWord(GYRO_XOUT_H);
		gSTAvg[1] += readWord(GYRO_YOUT_H);
		gSTAvg[2] += readWord(GYRO_ZOUT_H);
	}

	for (int ii = 0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	writeByte(ACCEL_CONFIG, 0x00);
	writeByte(GYRO_CONFIG, 0x00);
	delay(25);  // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	selfTest[0] = readByte(SELF_TEST_X_ACCEL); // X-axis accel self-test results
	selfTest[1] = readByte(SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
	selfTest[2] = readByte(SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
	selfTest[3] = readByte(SELF_TEST_X_GYRO);  // X-axis gyro self-test results
	selfTest[4] = readByte(SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
	selfTest[5] = readByte(SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

	// Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0))); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (int i = 0; i < 3; i++) {
		destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i];   // Report percent differences
		destination[i + 3] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
	}
}

uint8_t MPU9250Class::readByte(uint8_t reg)
{
	SPI.transfer(CSPIN, reg | 0x80, SPI_CONTINUE);
	uint8_t data = SPI.transfer(CSPIN, 0, SPI_LAST);
	return data;
}

void MPU9250Class::writeByte(uint8_t reg, uint8_t data)
{
	SPI.transfer(CSPIN, reg, SPI_CONTINUE);
	SPI.transfer(CSPIN, data, SPI_LAST);
}

uint16_t MPU9250Class::readWord(uint8_t reg)
{
	uint16_t retval;
	retval = readByte(reg);			// read high byte
	retval <<= 8;					// shift read value to top byte of return word
	retval |= readByte(reg + 1);	// OR in low byte
	return retval;
}

uint16_t MPU9250Class::readFIFOWord()
{
	uint16_t retval;
	retval = readByte(FIFO_R_W);	// read high byte
	retval <<= 8;					// shift read value to top byte of return word
	retval |= readByte(FIFO_R_W);	// OR in low byte
	return retval;
}

uint8_t MPU9250Class::readMagByte(uint8_t reg)
{
	uint8_t data;

	writeByte(I2C_SLV0_ADDR, AK8963_ADDRESS | 0x80);	// Set the I2C slave address of AK8963 and set for write (top bit = 1).
	writeByte(I2C_SLV0_REG, reg);				// I2C slave 0 register address from where to begin data transfer
	writeByte(I2C_SLV0_CTRL, 0x81);				// Start transfer, data goes into EXT_SENS_DATA_00
	delay(10);
	data = readByte(EXT_SENS_DATA_00);
	return data;
}

void MPU9250Class::writeMagByte(uint8_t reg, uint8_t data)
{
	writeByte(I2C_SLV0_ADDR, AK8963_ADDRESS);	// Set the I2C slave address of AK8963 and set for write (top bit = 0).
	writeByte(I2C_SLV0_REG, reg);				// I2C slave 0 register address from where to begin data transfer
	writeByte(I2C_SLV0_DO, data);				// Load up the data to send
	writeByte(I2C_SLV0_CTRL, 0x81);				// Start transfer
	delay(10);
}

MPU9250Class MPU9250;

