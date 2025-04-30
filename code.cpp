/**
 * SelfBalancingRobot.cpp
 * Implementation of a my self-balancing robot controller 
 * 
 * This program reads data from an MPU6050 accelerometer/gyroscope,
 * calculates the robot's orientation using a complementary filter,
 * and controls stepper motors to maintain balance using PID control.
 */

#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

class MPU6050 {
public:
    // MPU6050 I2C address
    static constexpr uint8_t I2C_ADDRESS = 0x68;

    // Register addresses
    enum Register {
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        PWR_MGMT_1 = 0x6B,
        ACCEL_CONFIG = 0x1C,
        GYRO_CONFIG = 0x1B
    };

    // Sensor read types
    enum ReadType {
        GYRO_X = 1,
        GYRO_Y = 2,
        GYRO_Z = 3,
        ACCEL_X = 4,
        ACCEL_Y = 5,
        ACCEL_Z = 6
    };

    MPU6050() = default;

    /**
     * Initialize the MPU6050 sensor
     */
    void begin() {
        Wire.begin();
        
        // Wake up the MPU6050
        writeRegister(Register::PWR_MGMT_1, 0);
        
        // Configure accelerometer (±8g range)
        writeRegister(Register::ACCEL_CONFIG, 0x10);
        
        // Configure gyroscope (±500deg/s range)
        writeRegister(Register::GYRO_CONFIG, 0x08);
    }

    /**
     * Read a 16-bit value from the MPU6050
     * @param type The type of reading to perform
     * @return The 16-bit raw sensor value
     */
    int16_t read(ReadType type) {
        Register highReg;
        
        switch (type) {
            case GYRO_X:
                highReg = Register::GYRO_XOUT_H;
                break;
            case GYRO_Y:
                highReg = Register::GYRO_YOUT_H;
                break;
            case GYRO_Z:
                highReg = Register::GYRO_ZOUT_H;
                break;
            case ACCEL_X:
                highReg = Register::ACCEL_XOUT_H;
                break;
            case ACCEL_Y:
                highReg = Register::ACCEL_YOUT_H;
                break;
            case ACCEL_Z:
                highReg = Register::ACCEL_ZOUT_H;
                break;
            default:
                return 0;
        }
        
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(static_cast<uint8_t>(highReg));
        Wire.endTransmission(false);
        Wire.requestFrom(I2C_ADDRESS, 2, true);
        
        return static_cast<int16_t>(Wire.read() << 8 | Wire.read());
    }

private:
    /**
     * Write a value to an MPU6050 register
     * @param reg The register to write to
     * @param value The value to write
     */
    void writeRegister(Register reg, uint8_t value) {
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(static_cast<uint8_t>(reg));
        Wire.write(value);
        Wire.endTransmission(true);
    }
};

class StepperMotorController {
public:
    /**
     * @param enablePin Pin connected to the stepper driver enable
     * @param dirPinA Direction pin for motor A
     * @param dirPinB Direction pin for motor B
     * @param stepPinA Step pin for motor A
     * @param stepPinB Step pin for motor B
     * @param ms1Pin MS1 microstepping control pin
     * @param ms3Pin MS3 microstepping control pin
     */
    StepperMotorController(
        uint8_t enablePin, 
        uint8_t dirPinA, uint8_t dirPinB,
        uint8_t stepPinA, uint8_t stepPinB,
        uint8_t ms1Pin, uint8_t ms3Pin
    ) : m_enablePin(enablePin),
        m_dirPinA(dirPinA), m_dirPinB(dirPinB),
        m_stepPinA(stepPinA), m_stepPinB(stepPinB),
        m_ms1Pin(ms1Pin), m_ms3Pin(ms3Pin) {}

    /**
     * Initialize the stepper motor controller pins
     */
    void begin() {
        pinMode(m_enablePin, OUTPUT);
        pinMode(m_dirPinA, OUTPUT);
        pinMode(m_dirPinB, OUTPUT);
        pinMode(m_ms1Pin, OUTPUT);
        pinMode(m_ms3Pin, OUTPUT);
        pinMode(m_stepPinA, OUTPUT);
        pinMode(m_stepPinB, OUTPUT);
        
        // Set microstepping mode (1/16 step)
        digitalWrite(m_ms1Pin, HIGH);
        digitalWrite(m_ms3Pin, HIGH);
        
        // Initially disable motors
        disable();
        
        // Configure Timer2 for step pulse generation
        TCCR2A = _BV(WGM21) | _BV(COM2B0);  // CTC mode, toggle OC2B on compare match
        TCCR2B = _BV(CS21) | _BV(CS22);     // Prescaler 256
        OCR2A = 0;                          // Initialize compare value
    }

    /**
     * Enable the stepper motors
     */
    void enable() {
        digitalWrite(m_enablePin, LOW);  // Active LOW
    }

    /**
     * Disable the stepper motors
     */
    void disable() {
        digitalWrite(m_enablePin, HIGH);  // Active LOW
    }

    /**
     * Set the motor step frequency
     * @param frequency Step frequency in Hz
     * @param direction 0 for forward, 1 for backward
     */
    void setMotion(double frequency, bool direction) {
        if (frequency < 400) {
            disable();
            OCR2A = 0;
            return;
        }
        
        enable();
        
        // Set direction
        if (direction) {
            digitalWrite(m_dirPinA, HIGH);
            digitalWrite(m_dirPinB, LOW);
        } else {
            digitalWrite(m_dirPinA, LOW);
            digitalWrite(m_dirPinB, HIGH);
        }
        
        // Calculate and set timer compare value for step frequency
        // F_CPU is the CPU frequency (16MHz for most Arduinos)
        OCR2A = static_cast<uint8_t>(F_CPU / 256 / frequency / 2);
    }

private:
    uint8_t m_enablePin;
    uint8_t m_dirPinA, m_dirPinB;
    uint8_t m_stepPinA, m_stepPinB;
    uint8_t m_ms1Pin, m_ms3Pin;
};

class OrientationSensor {
public:
    OrientationSensor() : 
        m_gyroScaleFactor(65.5f),  // For ±500deg/s range
        m_filterAlpha(0.96f),      // Complementary filter coefficient
        m_lastTime(0),
        m_angleX(0), m_angleY(0), m_angleZ(0),
        m_lastAngleX(0), m_lastAngleY(0), m_lastAngleZ(0) {}

    /**
     * Initialize the orientation sensor
     */
    void begin() {
        m_mpu.begin();
        calibrate();
        m_lastTime = millis();
    }

    /**
     * Calibrate the sensor by taking multiple readings to establish baseline values
     * @param samples Number of samples to use for calibration
     */
    void calibrate(uint16_t samples = 200) {
        Serial.print("Calibrating sensor");
        
        float gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
        float accelXSum = 0, accelYSum = 0, accelZSum = 0;
        
        for (uint16_t i = 0; i < samples; i++) {
            if ((i % 25) == 0) {
                Serial.print(".");
            }
            
            gyroXSum += m_mpu.read(MPU6050::GYRO_X);
            gyroYSum += m_mpu.read(MPU6050::GYRO_Y);
            gyroZSum += m_mpu.read(MPU6050::GYRO_Z);
            accelXSum += m_mpu.read(MPU6050::ACCEL_X);
            accelYSum += m_mpu.read(MPU6050::ACCEL_Y);
            accelZSum += m_mpu.read(MPU6050::ACCEL_Z);
            
            delay(5);
        }
        
        m_baseGyroX = gyroXSum / samples;
        m_baseGyroY = gyroYSum / samples;
        m_baseGyroZ = gyroZSum / samples;
        m_baseAccelX = accelXSum / samples;
        m_baseAccelY = accelYSum / samples;
        m_baseAccelZ = accelZSum / samples;
        
        Serial.println("calibration complete!");
    }

    /**
     * Update the orientation calculation based on latest sensor readings
     */
    void update() {
        // Read raw sensor values
        int16_t gyroX = m_mpu.read(MPU6050::GYRO_X);
        int16_t gyroY = m_mpu.read(MPU6050::GYRO_Y);
        int16_t gyroZ = m_mpu.read(MPU6050::GYRO_Z);
        int16_t accelX = m_mpu.read(MPU6050::ACCEL_X);
        int16_t accelY = m_mpu.read(MPU6050::ACCEL_Y);
        int16_t accelZ = m_mpu.read(MPU6050::ACCEL_Z);
        
        // Calculate time delta
        unsigned long currentTime = millis();
        float dt = (currentTime - m_lastTime) / 1000.0f;
        
        // Convert gyro values to degrees per second
        float gx = (gyroX - m_baseGyroX) / m_gyroScaleFactor;
        float gy = (gyroY - m_baseGyroY) / m_gyroScaleFactor;
        float gz = (gyroZ - m_baseGyroZ) / m_gyroScaleFactor;
        
        // Calculate angles from accelerometer data (in degrees)
        float accelAngleX = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
        float accelAngleY = atan2(accelY, sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180.0f / M_PI;
        
        // Integrate gyro rates to get angles
        float gyroAngleX = gx * dt + m_lastAngleX;
        float gyroAngleY = gy * dt + m_lastAngleY;
        float gyroAngleZ = gz * dt + m_lastAngleZ;
        
        // Apply complementary filter to combine accelerometer and gyro data
        m_angleX = m_filterAlpha * gyroAngleX + (1.0f - m_filterAlpha) * accelAngleX;
        m_angleY = m_filterAlpha * gyroAngleY + (1.0f - m_filterAlpha) * accelAngleY;
        m_angleZ = gyroAngleZ;  // No accelerometer reference for Z rotation
        
        // Save current values for next iteration
        m_lastAngleX = m_angleX;
        m_lastAngleY = m_angleY;
        m_lastAngleZ = m_angleZ;
        m_lastTime = currentTime;
    }

    // Getters for orientation angles
    float getAngleX() const { return m_angleX; }
    float getAngleY() const { return m_angleY; }
    float getAngleZ() const { return m_angleZ; }

private:
    MPU6050 m_mpu;
    
    float m_gyroScaleFactor;  // Conversion factor for gyro readings
    float m_filterAlpha;      // Complementary filter coefficient
    
    unsigned long m_lastTime;
    
    // Calibration baseline values
    float m_baseGyroX = 0, m_baseGyroY = 0, m_baseGyroZ = 0;
    float m_baseAccelX = 0, m_baseAccelY = 0, m_baseAccelZ = 0;
    
    // Current orientation angles
    float m_angleX, m_angleY, m_angleZ;
    float m_lastAngleX, m_lastAngleY, m_lastAngleZ;
};

class SelfBalancingRobot {
public:
    SelfBalancingRobot() : 
        m_motorController(ENB, DRA, DRB, STPA, STPB, MS1, MS3),
        m_pid(&m_inputAngle, &m_outputFrequency, &m_setpoint, KP, KI, KD, REVERSE) {}

    /**
     * Initialize the robot systems
     */
    void begin() {
        // Initialize components
        m_orientationSensor.begin();
        m_motorController.begin();
        
        // Configure PID controller
        m_pid.SetMode(AUTOMATIC);
        m_pid.SetOutputLimits(125, 20000);  // Motor frequency limits
        
        // Set balance point
        m_setpoint = 0;  // Target angle (upright)
    }

    /**
     * Main control loop for the robot
     */
    void update() {
        // Update orientation data
        m_orientationSensor.update();
        
        // Get current tilt angle (X axis)
        float rawAngle = m_orientationSensor.getAngleX();
        
        // Determine tilt direction
        bool direction = (rawAngle > 0);
        
        // Use absolute angle for PID calculation
        m_inputAngle = fabs(rawAngle);
        
        // Compute required motor frequency
        m_pid.Compute();
        
        // Debug output
        Serial.println(m_outputFrequency);
        
        // Update motor control
        m_motorController.setMotion(m_outputFrequency, direction);
    }

    /**
     * Run the robot's main loop
     */
    void run() {
        while (true) {
            update();
        }
    }

private:
    // Component pin definitions
    static constexpr uint8_t ENB = 5;   // Enable pin
    static constexpr uint8_t STPA = 3;  // Step pin A
    static constexpr uint8_t STPB = 9;  // Step pin B
    static constexpr uint8_t DRA = 4;   // Direction pin A
    static constexpr uint8_t DRB = 7;   // Direction pin B
    static constexpr uint8_t MS1 = 8;   // Microstepping config 1
    static constexpr uint8_t MS3 = 10;  // Microstepping config 3
    
    // PID controller constants
    static constexpr double KP = 280.0;
    static constexpr double KI = 1.2;
    static constexpr double KD = 4.0;
    
    // System components
    OrientationSensor m_orientationSensor;
    StepperMotorController m_motorController;
    
    // PID controller
    double m_inputAngle = 0;       // Current angle (PID input)
    double m_outputFrequency = 0;  // Motor step frequency (PID output)
    double m_setpoint = 0;         // Target angle (PID setpoint)
    PID m_pid;
};

// Main program
SelfBalancingRobot robot;

void setup() {
    Serial.begin(9600);
    robot.begin();
}

void loop() {
    robot.run();  // This function contains an infinite loop
}