#pragma once

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <vector>
#include <chrono>
#include "rt/bno_protocol.hpp"
#include <eigen3/Eigen/Dense>

// Packets can be up to 32k but we don't have that much RAM.
#define MAX_PACKET_SIZE 4096

// This is in words. There can be many but we mostly only care about the first 9 (Qs, range,
// etc)
#define MAX_METADATA_SIZE 9

// Not sure about this one
#define I2C_BUFFER_LENGTH 32

#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

class BNO055 {
 public:
  static constexpr int I2C_ADDR = 0x4B;

  const uint8_t CHANNEL_COMMAND = 0;
  const uint8_t CHANNEL_EXECUTABLE = 1;
  const uint8_t CHANNEL_CONTROL = 2;
  const uint8_t CHANNEL_REPORTS = 3;
  const uint8_t CHANNEL_WAKE_REPORTS = 4;
  const uint8_t CHANNEL_GYRO = 5;

  int16_t rotationVector_Q1 = 14;
  int16_t accelerometer_Q1 = 8;
  int16_t linear_accelerometer_Q1 = 8;
  int16_t gyro_Q1 = 9;
  int16_t magnetometer_Q1 = 4;

  BNO055(int i2c_device_number, uint32_t micros_between_reports);

  ~BNO055();

  BNO055(const BNO055 &) = delete;

  BNO055 &operator=(const BNO055 &) = delete;

  struct Output {
    Eigen::Quaternionf quat{0, 0, 0, 0};
    Eigen::Vector3f acc = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    bool quaternion_valid = false, gyro_valid = false, acceleration_valid = false;
    uint8_t quaternion_accuracy = 0, gyro_accuracy = 0;
    // Host receipt clocks, independently updated only by accepted new reports.
    // They are NOT sensor measurement timestamps or proof of calibration.
    std::chrono::steady_clock::time_point quaternion_receipt_time{};
    std::chrono::steady_clock::time_point gyro_receipt_time{};
    // Lower bounds from signed SH-2 batch offsets, not exact measurement ages.
    // Actual receipt timestamps above are never shifted to fake sensor time.
    std::chrono::microseconds quaternion_minimum_age_at_receipt{0};
    std::chrono::microseconds gyro_minimum_age_at_receipt{0};
    uint32_t legacy_base_timestamp_raw = 0;

    // Timestamp of when the data was sent from the sensor to the Raspberry Pi
    std::chrono::high_resolution_clock::time_point packet_timestamp;

    // DEPRECATED, UNVERIFIED legacy age calculation: raw base ticks treated as us.
    // Retained for compatibility; do not use for freshness or actual sensor age.
    std::chrono::high_resolution_clock::time_point measurement_timestamp;
  };

  Output sample();
  void read_vector(int addr, float scale, int num_elt, float *out);

 private:
  void enter_mode(int mode);
  void write_byte(int addr, int value);

  template <int dataLength>
  bool sendPacket(uint8_t channelNumber) {
    constexpr uint8_t packetLength = dataLength + 4;  // Add four bytes for the header
    std::array<uint8_t, packetLength> buffer;

    buffer[0] = packetLength & 0xFF;              // Packet length LSB
    buffer[1] = packetLength >> 8;                // Packet length MSB
    buffer[2] = channelNumber;                    // Channel number
    buffer[3] = sequenceNumber[channelNumber]++;  // Send the sequence number, increments with each
                                                  // packet sent, different counter for each channel

    // Send the user's data packet
    for (uint8_t i = 0; i < dataLength; i++) {
      buffer[i + 4] = shtpData[i];
    }
    if (write(m_fd, buffer.data(), packetLength) != packetLength) {
      printf("[BNO055] failed to write for sendPacket: %s\n", strerror(errno));
      return false;
    }

    return true;
  }

  void softReset(void);
  bool receivePacket(void);
  bool waitForI2C(void);
  void setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports, uint32_t specificConfig);
  void enableRotationVector(uint16_t timeBetweenReports);
  void enableGyro(uint16_t timeBetweenReports);
  void enableLinearAccelerometer(uint16_t timeBetweenReports);
  void enableAccelerometer(uint16_t timeBetweenReports);
  void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
  bool dataAvailable(void);
  float getQuatI();
  float getQuatJ();
  float getQuatK();
  float getQuatReal();
  float getLinAccelX();
  float getLinAccelY();
  float getLinAccelZ();
  float getAccelX();
  float getAccelY();
  float getAccelZ();
  float getGyroX();
  float getGyroY();
  float getGyroZ();
  uint32_t getTimestamp();

  float qToFloat(int16_t fixedPointValue, uint8_t qPoint);
  void parseInputReport(void);
  void parseCommandReport(void);

  int m_fd = 0;
  bool m_okay = false;

  // Timestamp of when the data was sent from the sensor to the Raspberry Pi
  std::chrono::high_resolution_clock::time_point latest_packet_timestamp;

  uint16_t rawAccelX=0, rawAccelY=0, rawAccelZ=0, accelAccuracy=0;
  uint16_t rawLinAccelX=0, rawLinAccelY=0, rawLinAccelZ=0, accelLinAccuracy=0;
  uint16_t rawGyroX=0, rawGyroY=0, rawGyroZ=0, gyroAccuracy=0;
  uint16_t rawMagX=0, rawMagY=0, rawMagZ=0, magAccuracy=0;
  uint16_t rawQuatI=0, rawQuatJ=0, rawQuatK=0, rawQuatReal=0, rawQuatRadianAccuracy=0, quatAccuracy=0;
  uint16_t stepCount;
  uint32_t timeStamp = 0;
  size_t payload_size_ = 0;
  bool quaternion_valid_ = false, gyro_valid_ = false;
  pupper_bno::ReportSequence quaternion_sequence_, gyro_sequence_;
  std::chrono::steady_clock::time_point quaternion_receipt_time_{}, gyro_receipt_time_{};
  std::chrono::microseconds quaternion_minimum_age_at_receipt_{0}, gyro_minimum_age_at_receipt_{0};
  uint8_t stabilityClassifier;
  uint8_t activityClassifier;
  uint8_t *_activityConfidences;  // Array that store the confidences of the 9 possible activities
  uint8_t calibrationStatus = 255;      // Byte R0 of ME Calibration Response

  uint8_t shtpHeader[4]{};  // Each packet has a header of 4 bytes
  uint8_t shtpData[MAX_PACKET_SIZE]{};
  uint8_t sequenceNumber[6] = {0, 0, 0, 0,
                               0, 0};  // There are 6 com channels. Each channel has its own seqnum
  uint8_t commandSequenceNumber = 0;   // Commands have a seqNum as well. These are inside command
                                       // packet, the header uses its own seqNum per channel
  uint32_t metaData[MAX_METADATA_SIZE];  // There is more than 10 words in a metadata record but
                                         // we'll stop at Q point 3
};