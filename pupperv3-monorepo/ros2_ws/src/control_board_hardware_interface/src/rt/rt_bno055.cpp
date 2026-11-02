
#include "rt/rt_bno055.h"

#include <string>
#include <stdexcept>
// #include <i2c/smbus.h>

#include <chrono>  // for std::chrono::milliseconds
#include <iostream>
#include <thread>  // for std::this_thread::sleep_for

BNO055::BNO055(int i2c_device_number, uint32_t micros_between_reports) {
  // the name
  std::string device_name = std::string("/dev/i2c-") + std::to_string(i2c_device_number);
  printf("[BNO055] Using device name %s\n", device_name.c_str());

  // open the device
  m_fd = open(device_name.c_str(), O_RDWR);
  if (m_fd < 0) {
    printf("[BNO055] failed to open: %s\n", strerror(errno));
    m_okay = false;
  } else {
    m_okay = true;
  }

  if (m_okay) {
    if (ioctl(m_fd, I2C_SLAVE, I2C_ADDR) < 0) {
      m_okay = false;
      printf("[BNO055] failed to set slave: %s\n", strerror(errno));
    }
  }
  if (!m_okay) {
    if (m_fd >= 0) ::close(m_fd);
    m_fd = -1;
    throw std::runtime_error("BNO I2C open/address selection failed");
  }
  printf("Connected\r\n");
  softReset();
  printf("Reset\r\n");
  shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST;  // Request the product ID and reset info
  shtpData[1] = 0;                               // Reserved

  // Transmit packet on channel 2, 2 bytes
  sendPacket<2>(CHANNEL_CONTROL);
  printf("Sent first packet\r\n");

  // Now we wait for response
  if (receivePacket() == true) {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE) {
      printf("Correct Product ID\r\n");
    }
  }

  std::cout << "Enabling rot vec + gyro. Micros between reports: " << micros_between_reports
            << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  enableRotationVector(micros_between_reports);
  // IMPORTANT: NOT ENABLING ACCELEROMETER
  enableGyro(micros_between_reports);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

BNO055::~BNO055() {
  close(m_fd);
}

void BNO055::softReset(void) {
  shtpData[0] = 1;  // Reset

  // Attempt to start communication with sensor
  sendPacket<1>(CHANNEL_EXECUTABLE);  // Transmit packet on channel 1, 1 byte

  // Read all incoming data and flush it
  std::chrono::milliseconds duration(200);  // duration of 50 milliseconds
  std::this_thread::sleep_for(duration);    // pause for the specified duration
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < deadline && receivePacket()) {}
  }
  printf("First receive finished\r\n");
  std::this_thread::sleep_for(duration);  // pause for the specified duration
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    while (std::chrono::steady_clock::now() < deadline && receivePacket()) {}
  }
  printf("second receive finished\r\n");
}

bool BNO055::receivePacket(void) {
  pupper_bno::Packet packet;
  const auto result = pupper_bno::read_packet(packet,
      [this](uint8_t *dst, size_t count) { return ::read(m_fd, dst, count); },
      []() { return uint64_t(std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count()); },
      []() { std::this_thread::sleep_for(std::chrono::milliseconds(1)); });
  payload_size_ = 0;
  std::fill(std::begin(shtpHeader), std::end(shtpHeader), uint8_t(0));
  std::fill(std::begin(shtpData), std::end(shtpData), uint8_t(0));
  if (result != pupper_bno::ReadResult::packet) return false;
  std::copy(packet.header.begin(), packet.header.end(), shtpHeader);
  std::copy(packet.payload.begin(), packet.payload.end(), shtpData);
  payload_size_ = packet.payload.size();
  return true;
}

bool BNO055::waitForI2C() {
  uint8_t buffer[1];
  int timeoutCounter = 100;

  while (timeoutCounter > 0) {
    int bytesRead = read(m_fd, &buffer, 1);
    if (bytesRead == -1) {
      printf("[BNO055] failed to read for waitForI2C: %s\n", strerror(errno));
      return false;
    } else if (bytesRead == 1) {
      return true;
    }

    timeoutCounter--;
  }

  return false;
}

void BNO055::setFeatureCommand(uint8_t reportID, uint32_t microsBetweenReports,
                               uint32_t specificConfig) {
  shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;  // Set feature command. Reference page 55
  shtpData[1] = reportID;  // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
  shtpData[2] = 0;         // Feature flags
  shtpData[3] = 0;         // Change sensitivity (LSB)
  shtpData[4] = 0;         // Change sensitivity (MSB)
  shtpData[5] = (microsBetweenReports >> 0) & 0xFF;   // Report interval (LSB) in microseconds.
  shtpData[6] = (microsBetweenReports >> 8) & 0xFF;   // Report interval
  shtpData[7] = (microsBetweenReports >> 16) & 0xFF;  // Report interval
  shtpData[8] = (microsBetweenReports >> 24) & 0xFF;  // Report interval (MSB)
  shtpData[9] = 0;                                    // Batch Interval (LSB)
  shtpData[10] = 0;                                   // Batch Interval
  shtpData[11] = 0;                                   // Batch Interval
  shtpData[12] = 0;                                   // Batch Interval (MSB)
  shtpData[13] = (specificConfig >> 0) & 0xFF;        // Sensor-specific config (LSB)
  shtpData[14] = (specificConfig >> 8) & 0xFF;        // Sensor-specific config
  shtpData[15] = (specificConfig >> 16) & 0xFF;       // Sensor-specific config
  shtpData[16] = (specificConfig >> 24) & 0xFF;       // Sensor-specific config (MSB)

  // Transmit packet on channel 2, 17 bytes
  sendPacket<17>(CHANNEL_CONTROL);
}
void BNO055::enableRotationVector(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}
void BNO055::enableGyro(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}
void BNO055::enableLinearAccelerometer(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}
void BNO055::enableAccelerometer(uint16_t timeBetweenReports) {
  setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

void BNO055::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports) {
  setFeatureCommand(reportID, timeBetweenReports, 0);  // No specific config
}
bool BNO055::dataAvailable(void) {
  if (receivePacket() == true) {
    // Check to see if this packet is a sensor reporting its data to us
    // printf("here\n");
    if (shtpHeader[2] == CHANNEL_REPORTS) {
      //   printf("got channel report. shtpData[0]=%d\n", static_cast<int>(shtpData[0]));
      if (shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP) {
        // printf("got base timestamp\n");
        parseInputReport();  // This will update the rawAccelX, etc variables depending on which
                             // feature report is found
        return (true);
      }
    } else if (shtpHeader[2] == CHANNEL_CONTROL) {
      //   printf("got channel control\n");
      parseCommandReport();  // This will update responses to commands, calibrationStatus, etc.
      return (true);
    } else {
      printf("IMU got wack data\n");
    }
  }

  return (false);
}
float BNO055::getQuatI() {
  float quat = qToFloat(rawQuatI, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion J
float BNO055::getQuatJ() {
  float quat = qToFloat(rawQuatJ, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion K
float BNO055::getQuatK() {
  float quat = qToFloat(rawQuatK, rotationVector_Q1);
  return (quat);
}

// Return the rotation vector quaternion Real
float BNO055::getQuatReal() {
  float quat = qToFloat(rawQuatReal, rotationVector_Q1);
  return (quat);
}

// Return the acceleration component
float BNO055::getLinAccelX() {
  float accel = qToFloat(rawLinAccelX, linear_accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getLinAccelY() {
  float accel = qToFloat(rawLinAccelY, linear_accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getLinAccelZ() {
  float accel = qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
  return (accel);
}
// Return the acceleration component
float BNO055::getAccelX() {
  float accel = qToFloat(rawAccelX, accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getAccelY() {
  float accel = qToFloat(rawAccelY, accelerometer_Q1);
  return (accel);
}

// Return the acceleration component
float BNO055::getAccelZ() {
  float accel = qToFloat(rawAccelZ, accelerometer_Q1);
  return (accel);
}

// Return the gyro component
float BNO055::getGyroX() {
  float gyro = qToFloat(rawGyroX, gyro_Q1);
  return (gyro);
}

// Return the gyro component
float BNO055::getGyroY() {
  float gyro = qToFloat(rawGyroY, gyro_Q1);
  return (gyro);
}

// Return the gyro component
float BNO055::getGyroZ() {
  float gyro = qToFloat(rawGyroZ, gyro_Q1);
  return (gyro);
}

uint32_t BNO055::getTimestamp() {
  return timeStamp;
}

float BNO055::qToFloat(int16_t fixedPointValue, uint8_t qPoint) {
  float qFloat = fixedPointValue;
  qFloat *= pow(2, qPoint * -1);
  return (qFloat);
}
// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
// shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
// shtpData[5 + 1]: Sequence number (See 6.5.18.2)
// shtpData[5 + 2]: Status
// shtpData[3]: Delay
// shtpData[4:5]: i/accel x/gyro x/etc
// shtpData[6:7]: j/accel y/gyro y/etc
// shtpData[8:9]: k/accel z/gyro z/etc
// shtpData[10:11]: real/gyro temp/etc
// shtpData[12:13]: Accuracy estimate
void BNO055::parseInputReport(void) {
  std::vector<pupper_bno::Report> reports;
  if (!pupper_bno::parse_reports(std::vector<uint8_t>(shtpData, shtpData+payload_size_), reports)) return;
  // Preserve the old unverified base timestamp value for compatibility only.
  timeStamp = uint32_t(shtpData[1]) | uint32_t(shtpData[2]) << 8
              | uint32_t(shtpData[3]) << 16 | uint32_t(shtpData[4]) << 24;
  const auto received = std::chrono::steady_clock::now();
  for (const auto &report : reports) {
    // SH-2 v1.9 section7.2: sample = HINT + (signed reference + delay)*100us.
    // HINT precedes receipt, so a negative offset proves this minimum age.
    // Positive offsets prove no age bound; they must not fabricate future clocks.
    const int64_t relative_ticks = report.reference_delta_ticks + report.delay_ticks;
    const auto minimum_age = std::chrono::microseconds(relative_ticks < 0 ? -relative_ticks*100 : 0);
    if (report.id == SENSOR_REPORTID_GYROSCOPE && gyro_sequence_.accept(report.sequence)) {
      if (!report.timing_valid) { gyro_valid_ = false; continue; }
      gyro_minimum_age_at_receipt_ = minimum_age;
      rawGyroX = uint16_t(report.values[0]); rawGyroY = uint16_t(report.values[1]);
      rawGyroZ = uint16_t(report.values[2]); gyroAccuracy = report.accuracy;
      gyro_receipt_time_ = received; gyro_valid_ = true;
    } else if (report.id == SENSOR_REPORTID_ROTATION_VECTOR && quaternion_sequence_.accept(report.sequence)) {
      if (!report.timing_valid) { quaternion_valid_ = false; continue; }
      quaternion_minimum_age_at_receipt_ = minimum_age;
      rawQuatI = uint16_t(report.values[0]); rawQuatJ = uint16_t(report.values[1]);
      rawQuatK = uint16_t(report.values[2]); rawQuatReal = uint16_t(report.values[3]);
      rawQuatRadianAccuracy = report.rotation_accuracy; quatAccuracy = report.accuracy;
      quaternion_receipt_time_ = received;
      quaternion_valid_ = std::any_of(report.values.begin(), report.values.end(), [](int16_t v) { return v != 0; });
    }
    // Do not silently replace the selected RotationVector stream with GameRV.
    // Other complete known reports are parsed for boundaries, not adopted.
  }
}

// Unit responds with packet that contains the following:
// shtpHeader[0:3]: First, a 4 byte header
// shtpData[0]: The Report ID
// shtpData[1]: Sequence number (See 6.5.18.2)
// shtpData[2]: Command
// shtpData[3]: Command Sequence Number
// shtpData[4]: Response Sequence Number
// shtpData[5 + 0]: R0
// shtpData[5 + 1]: R1
// shtpData[5 + 2]: R2
// shtpData[5 + 3]: R3
// shtpData[5 + 4]: R4
// shtpData[5 + 5]: R5
// shtpData[5 + 6]: R6
// shtpData[5 + 7]: R7
// shtpData[5 + 8]: R8
void BNO055::parseCommandReport(void) {
  if (payload_size_ >= 16 && shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
    // The BNO080 responds with this report to command requests. It's up to use to remember which
    // command we issued.
    uint8_t command = shtpData[2];  // This is the Command byte of the response

    if (command == COMMAND_ME_CALIBRATE) {
      calibrationStatus = shtpData[5 + 0];  // R0 - Status (0 = success, non-zero = fail)
    }
  } else {
    // This sensor report ID is unhandled.
    // See reference manual to add additional feature reports as needed
  }

  // TODO additional feature reports may be strung together. Parse them all.
}

BNO055::Output BNO055::sample() {
  // Typically has two messages available in one call to sample()
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(25);
  for (unsigned packets=0; packets<32 && std::chrono::steady_clock::now()<deadline && dataAvailable(); ++packets) {
    latest_packet_timestamp = std::chrono::high_resolution_clock::now();
  }
  Output result;
  result.quat.x() = getQuatI();
  result.quat.y() = getQuatJ();
  result.quat.z() = getQuatK();
  result.quat.w() = getQuatReal();

  // Accelerometer currently not enabled
  //   result.acc(0) = getAccelX();
  //   result.acc(1) = getAccelY();
  //   result.acc(2) = getAccelZ();

  result.gyro(0) = getGyroX();
  result.gyro(1) = getGyroY();
  result.gyro(2) = getGyroZ();

  result.quaternion_valid = quaternion_valid_;
  result.gyro_valid = gyro_valid_;
  result.quaternion_accuracy = uint8_t(quatAccuracy);
  result.gyro_accuracy = uint8_t(gyroAccuracy);
  result.quaternion_receipt_time = quaternion_receipt_time_;
  result.gyro_receipt_time = gyro_receipt_time_;
  result.quaternion_minimum_age_at_receipt = quaternion_minimum_age_at_receipt_;
  result.gyro_minimum_age_at_receipt = gyro_minimum_age_at_receipt_;
  result.legacy_base_timestamp_raw = timeStamp;
  // acc is deterministic zero and acceleration_valid remains false.
  // The following fields retain the deprecated legacy calculation unchanged.
  // They are NOT used by the new independent host-receipt freshness checks.
  // Record time data read from IMU
  result.packet_timestamp = latest_packet_timestamp;
  result.measurement_timestamp = latest_packet_timestamp - std::chrono::microseconds(timeStamp);

  return result;
}
