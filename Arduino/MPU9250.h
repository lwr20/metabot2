// MPU9250.h

#ifndef _MPU9250_h
#define _MPU9250_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "MPU9250Reg.h"

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 1
#if ADO
#define MPU9250_ADDRESS 0x69  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer
#endif  

#define AHRS true         // set to false for basic data read
#define SerialDebug true   // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {
	AFS_2G = 0,
	AFS_4G,
	AFS_8G,
	AFS_16G
};

enum Gscale {
	GFS_250DPS = 0,
	GFS_500DPS,
	GFS_1000DPS,
	GFS_2000DPS
};

enum Mscale {
	MFS_14BITS = 0, // 0.6 mG per LSB
	MFS_16BITS      // 0.15 mG per LSB
};

class MPU9250Class
{
 public:
	void init();
	void loop();

 private:
	 void initAK8963(float * destination);
	 void initMPU9250();
	 void calibrateMPU9250(float * dest1, float * dest2);
	 void MPU9250SelfTest(float * destination);
	 void getMres();
	 void getGres();
	 void getAres();
	 void readAccelData(int16_t * destination);
	 void readGyroData(int16_t * destination);
	 void readMagData(int16_t * destination);
	 int16_t readTempData();
	 void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
	 void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

	uint8_t readByte(uint8_t reg);
	uint16_t readWord(uint8_t reg);
	uint16_t readFIFOWord();
	void writeByte(uint8_t reg, uint8_t data);
	uint8_t readMagByte(uint8_t reg);
	void writeMagByte(uint8_t reg, uint8_t data);

	// Specify sensor full scale
	uint8_t Gscale = GFS_250DPS;
	uint8_t Ascale = AFS_2G;
	uint8_t Mscale = MFS_16BITS;					// Choose either 14-bit or 16-bit magnetometer resolution
	uint8_t Mmode = 0x02;							// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
	float aRes, gRes, mRes;							// scale resolutions per LSB for the sensors

	int16_t accelCount[3];							// Stores the 16-bit signed accelerometer sensor output
	int16_t gyroCount[3];							// Stores the 16-bit signed gyro sensor output
	int16_t magCount[3];							// Stores the 16-bit signed magnetometer sensor output
	float magCalibration[3] = { 0, 0, 0 }, magbias[3] = { 0, 0, 0 };  // Factory mag calibration and mag bias
	float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 };      // Bias corrections for gyro and accelerometer
	int16_t tempCount;								// temperature raw count output
	float   temperature;							// Stores the real internal chip temperature in degrees Celsius
	float   SelfTest[6];							// holds results of gyro and accelerometer self test

	// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
	float GyroMeasError = PI * (40.0f / 180.0f);	// gyroscope measurement error in rads/s (start at 40 deg/s)
	float GyroMeasDrift = PI * (0.0f / 180.0f);		// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
													// There is a tradeoff in the beta parameter between accuracy and response speed.
													// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
													// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
													// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
													// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
													// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
													// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
													// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;	// compute beta
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;	// compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f								// these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

	uint32_t delt_t = 0;							// used to control display output rate
	uint32_t count = 0, sumCount = 0;				// used to control display output rate
	float pitch, yaw, roll;
	float deltat = 0.0f, sum = 0.0f;				// integration interval for both filter schemes
	uint32_t lastUpdate = 0, firstUpdate = 0;		// used to calculate integration interval
	uint32_t Now = 0;								// used to calculate integration interval

	float ax, ay, az, gx, gy, gz, mx, my, mz;		// variables to hold latest sensor data values 
	float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };		// vector to hold quaternion
	float eInt[3] = { 0.0f, 0.0f, 0.0f };			// vector to hold integral error for Mahony method

};

extern MPU9250Class MPU9250;

#endif

