#include <BMI160.h>
#include <BMI160Gen.h>
#include <CurieIMU.h>

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncUDP.h>
#include "esp32_settings.h"
#include <Wire.h>
#include "QuaternionCalc.h"
//#include "quat.h"

#include <iostream>     // std::cout
#include <tuple>        // std::tuple, std::get, std::tie, std::ignore

#define DEBUG true
#define DEBUG_PRINT if(DEBUG)Serial

#define ROTVEC 0
#define GAMEROTVEC 1

char header[128];
char networkSSID[15] = "George";
char networkPassword[15] = "039217752";
uint16_t serialLength;

IPAddress udpAddress(0, 0, 0, 0);
const uint16_t broadPort = 6969;
uint16_t serverPort = 0;
uint16_t driverPort = 0;

bool connected = false;

WiFiUDP udp;
AsyncUDP Audp;

#define EARTH_GRAVITY 9.80665
/* Typical sensitivity at 25C
   See p. 9 of https://www.mouser.com/datasheet/2/783/BST-BMI160-DS000-1509569.pdf
   65.6 LSB/deg/s = 500 deg/s
*/
#define TYPICAL_SENSITIVITY_LSB 65.6

// Scale conversion steps: LSB/°/s -> °/s -> step/°/s -> step/rad/s
constexpr float GSCALE = ((32768. / TYPICAL_SENSITIVITY_LSB) / 32768.) * (PI / 180.0);

#define ACCEL_SENSITIVITY_4G 8192.0f

// Accel scale conversion steps: LSB/G -> G -> m/s^2
constexpr float ASCALE_4G = ((32768. / ACCEL_SENSITIVITY_4G) / 32768.) * EARTH_GRAVITY;

/* LSB change per temperature step map.
   These values were calculated for 500 deg/s sensitivity
   Step quantization - 5 degrees per step
*/
const float LSB_COMP_PER_TEMP_X_MAP[13] = {
  0.77888f, 1.01376f, 0.83848f, 0.39416f,             // 15, 20, 25, 30
  -0.08792f, -0.01576f, -0.1018f, 0.22208f,           // 35, 40, 45, 50
  0.22208f, 0.22208f, 0.22208f, 0.2316f,              // 55, 60, 65, 70
  0.53416f                                            // 75
};
const float LSB_COMP_PER_TEMP_Y_MAP[13] = {
  0.10936f, 0.24392f, 0.28816f, 0.24096f,
  0.05376f, -0.1464f, -0.22664f, -0.23864f,
  -0.25064f, -0.26592f, -0.28064f, -0.30224f,
  -0.31608f
};
const float LSB_COMP_PER_TEMP_Z_MAP[13] = {
  0.15136f, 0.04472f, 0.02528f, -0.07056f,
  0.03184f, -0.002f, -0.03888f, -0.14f,
  -0.14488f, -0.14976f, -0.15656f, -0.16108f,
  -0.1656f
};

const int i2c_addr_default = 0x69;
const int i2c_addr_jumper = 0x68;

struct VyroImuData {

  float q[4] { 1.0f, 0.0f, 0.0f, 0.0f };
  uint32_t now = 0, last = 0;   //micros() timers
  float deltat = 0;             //loop time in seconds  
  Quaternion quaternion;

  Quaternion getQuaternion(BMI160GenClass imu) {
    now = micros();
    deltat = now - last; //seconds since last update
    last = now;
  
    int gx, gy, gz;      // raw gyro values
    int ax, ay, az;
  
    // read raw gyro measurements from device
    imu.readGyro(gx, gy, gz);
    imu.readAccelerometer(ax, ay, az);
  
    quaternion = mahonyQuaternionUpdate(q, ax, ay, az, gx * GSCALE, gy * GSCALE, gz * GSCALE, deltat * 1.0e-6f);
    //quat.set(-q[2], q[1], q[3], q[0]);
  
    return quaternion;
  }
};

class Multiplexer {
  public:

  int set(int channel) {
    DEBUG_PRINT.println("Setting multiplexer to channel " + String(channel) + " ...");

    Wire.beginTransmission(0x70);   // TCA9548A address is 0x70
    Wire.write(1 << channel);       // send byte to select bus
    int result = Wire.endTransmission();

    switch (result) {
      case 0: DEBUG_PRINT.println("Multiplexer successfully changed to channel " + String(channel) + "!"); break;
      case 1: DEBUG_PRINT.println("Multiplexer errored (data too long to fit in transmit buffer) while changing to channel " + String(channel) + "!"); break;
      case 2: DEBUG_PRINT.println("Multiplexer errored (received NACK on transmit of address) while changing to channel " + String(channel) + "!"); break;
      case 3: DEBUG_PRINT.println("Multiplexer errored (received NACK on transmit of data) while changing to channel " + String(channel) + "!"); break;
      case 4: DEBUG_PRINT.println("Multiplexer errored (other error) while changing to channel " + String(channel) + "!"); break;
      case 5: DEBUG_PRINT.println("Multiplexer errored (timeout) while changing to channel " + String(channel) + "!"); break;
      default: DEBUG_PRINT.println("Multiplexer errored (other error [on the default case]) while changing to channel " + String(channel) + "!"); break;
    }

    return result;
  }
};

Multiplexer mp;

class VyroIMU : public BMI160GenClass {
  public:

  VyroImuData IMUsData[8];
  VyroImuData currentData;

  BMI160GenClass imu;

    void operator[](int i) {
     if(i > 7) {
        DEBUG_PRINT.println("Channel index out of bounds");
      }

      currentData = IMUsData[i];
    }

    bool start(bool jumper) {
      return imu.begin(BMI160GenClass::I2C_MODE, jumper ? i2c_addr_jumper : i2c_addr_default);
    }

    bool start(int address) {
      return imu.begin(BMI160GenClass::I2C_MODE, address);
    }

    bool startAndApplySettings(bool jumper) {
      return startAndApplySettings(jumper ? i2c_addr_jumper : i2c_addr_default);
    }

    bool startAndApplySettings(int address) {
      if (start(address))
      {
        imu.setGyroRate(BMI160_GYRO_RATE_800HZ);
        delay(1);
        imu.setAccelRate(BMI160_ACCEL_RATE_800HZ);
        delay(1);
        imu.setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
        delay(1);
        imu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_4G);
        delay(1);
        imu.setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
        delay(1);
        imu.setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
        delay(1);
  
        imu.setGyroOffsetEnabled(true);
        imu.setAccelOffsetEnabled(true);
      }
    }

    void Calibrate() {
      imu.autoCalibrateGyroOffset();
      imu.autoCalibrateAccelerometerOffset(1, 0);
      imu.autoCalibrateAccelerometerOffset(2, 0);
      imu.autoCalibrateAccelerometerOffset(3, 1);
    }

    Quaternion getQuaternion() {
      return currentData.getQuaternion(imu);
    }
};

class IMUsManager {
  public:

  VyroIMU imu1;
  VyroIMU imu2;

  bool imu1Good = false;
  bool imu2Good = false;

  bool extends[8];
  bool extend = false;

  int currentIndex = -1;

  IMUsManager* operator[](int i) {    
    if(i > 7) {
      DEBUG_PRINT.println("Channel index '" + String(i) + "' out of bounds.");
    }
    
    if (currentIndex != i){
      currentIndex = i;
      mp.set(i);
      extend = extends[i];
      imu1[i];
      imu2[i];
    }
    return this;
  }

  std::tuple<bool, bool> initChannel(int channel) {

    DEBUG_PRINT.println("Initializing IMUs on multiplexer channel " + String(channel));

    byte error, address;

    mp.set(channel);

    Wire.beginTransmission(i2c_addr_default);
    error = Wire.endTransmission();
    if (error == 0) {
      if (imu1.start(i2c_addr_default)) {
        imu1Good = true;
        DEBUG_PRINT.println("imu1 good");
        imu1.Calibrate();
      } else {
        DEBUG_PRINT.println("Error in multiplexer channel " + String(channel) + String(error) + " at address " + String(address, HEX));
      }
    }

    Wire.beginTransmission(i2c_addr_jumper);
    error = Wire.endTransmission();
    if (error == 0) {
      if (imu2.start(i2c_addr_jumper)) {
        imu2Good = true;
        DEBUG_PRINT.println("imu2 good");
        imu2.Calibrate();
      } else {
        DEBUG_PRINT.println("Error in multiplexer channel " + String(channel) + String(error) + " at address " + String(address, HEX));
      }
    }

    extends[channel] = imu1Good && imu2Good;
    
    return std::tuple<bool, bool>(imu1Good, imu2Good);
  }

  void initAllChannels() {
    for (int i = 0; i < 8; i++) {
      try {
        initChannel(i);
      }
      catch (String error) {
        DEBUG_PRINT.println(error);
      }
    }
  }

};


/*struct Quat {
    public:
        float w;
        float x;
        float y;
        float z;
        
        Quat() {
            w = 1.0f;
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        
        Quat(float nw, float nx, float ny, float nz) {
            w = nw;
            x = nx;
            y = ny;
            z = nz;
        }

        Quat getProduct(Quaternion q) {
            // Quaternion multiplication is defined by:
            //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
            //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
            //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
            //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
            return Quaternion(
                w*q.w - x*q.x - y*q.y - z*q.z,  // new w
                w*q.x + x*q.w + y*q.z - z*q.y,  // new x
                w*q.y - x*q.z + y*q.w + z*q.x,  // new y
                w*q.z + x*q.y - y*q.x + z*q.w); // new z
        }

        Quat getConjugate() {
            return Quaternion(w, -x, -y, -z);
        }
        
        float getMagnitude() {
            return sqrt(w*w + x*x + y*y + z*z);
        }
        
        void normalize() {
            float m = getMagnitude();
            w /= m;
            x /= m;
            y /= m;
            z /= m;
        }
        
        Quaternion getNormalized() {
            Quaternion r(w, x, y, z);
            r.normalize();
            return r;
        }
};
*/

struct TrackerData {
  uint8_t id = 0;
  Quaternion quat;
};

class Vyro {
  public:
  IMUsManager trackers;
};

struct __attribute__((packed)) PingBroad {
  uint8_t header = (uint8_t)'I';
  uint8_t id = 77; // payload[0]
  uint8_t a; // MAC Address part a (payload[1:7])
  uint8_t b; // MAC Address part b (payload[1:7])
  uint8_t c; // MAC Address part c (payload[1:7])
  uint8_t d; // MAC Address part d (payload[1:7])
  uint8_t e; // MAC Address part e (payload[1:7])
  uint8_t f; // MAC Address part f (payload[1:7])
  bool extend = false; // payload[7]
  uint8_t footer = (uint8_t)'i';
} pingbroad;

struct __attribute__((packed)) Ping {
  uint8_t header = (uint8_t)'I';
  uint8_t id = 0;
  uint8_t id_ext = 0;
  uint8_t a;
  uint8_t b;
  uint8_t c;
  uint8_t d;
  uint8_t e;
  uint8_t f;
  float batt = 0;
  uint8_t mode;
  uint8_t mode_ext;
  uint8_t accuracy = 0;
  uint8_t accuracy_ext = 0;
  uint8_t wifi_power;
  uint8_t wifi_sleep;
  bool extend = false;
  uint8_t footer = (uint8_t)'i';
} ping;

struct __attribute__((packed)) Ack {
  uint8_t header;
  uint8_t reply;
  uint8_t id;
  uint8_t id_ext;
  uint8_t mode;
  uint8_t mode_ext;
  uint8_t wifi_power;
  uint8_t wifi_sleep;
  uint16_t serverPort;
  uint16_t driverPort;
  uint8_t footer;
} ack;

struct __attribute__((packed)) PayloadExt {
  uint8_t header = (uint8_t)'I';
  uint8_t id = 0;
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int16_t w = 32767;
  uint8_t id_ext = 0;
  int16_t x_ext = 0;
  int16_t y_ext = 0;
  int16_t z_ext = 0;
  int16_t w_ext = 32767;
  uint8_t footer = (uint8_t)'i';
} payloadext;

struct __attribute__((packed)) Payload {
  uint8_t header = (uint8_t)'I';
  uint8_t id = 0;
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  int16_t w = 32767;
  uint8_t footer = (uint8_t)'i';
} payload;

struct __attribute__((packed)) PayloadVyro {
  uint8_t header = (uint8_t)'I';
  TrackerData trackers[12]; // Trackers data
  uint8_t footer = (uint8_t)'i';
} payloadvyro;

uint32_t last_broadcast;
uint32_t last_ping;
uint32_t last_main_imu_check;
uint32_t last_extend_imu_check;
uint32_t last_udp_check;
uint32_t recv_watchdog;
uint32_t t_blink;
uint8_t led_state = 0;

/*float q[4] { 1.0f, 0.0f, 0.0f, 0.0f };
uint32_t now = 0, last = 0;   //micros() timers
float deltat = 0;             //loop time in seconds
Quat quaternion {};
*/

bool extended_imu_found = false;

uint8_t main_id = 0;
uint8_t extended_id = 0;

uint8_t imu_mode = 101;
uint8_t imu_mode_ext = 101;

uint8_t wifi_power = 78;
uint8_t wifi_sleep = 1;

bool streaming_udp = false;
bool imu_start = false;

bool first_read = false;
bool first_read_ext = false;

float battery_voltage() {
  return get_battery_voltage(analogReadMilliVolts(batt_monitor_pin));
}

void reset_imu() {
  digitalWrite(reset_pin, HIGH);
  delay(100);
  digitalWrite(reset_pin, LOW);
  delay(100);
  digitalWrite(reset_pin, HIGH);
}

void blink_led(uint16_t interval) {
  if (!led_state) {
    if (millis() - t_blink >= interval) {
      led_state = !led_state;
      digitalWrite(led_pin, led_state);
      t_blink = millis();
    }
  }
  else {
    if (millis() - t_blink >= 100) {
      led_state = !led_state;
      digitalWrite(led_pin, led_state);
      t_blink = millis();
    }
  }
}

int16_t map(float x, float in_min = -1, float in_max = 1, float out_min = -32767, float out_max = 32767) {
  return (int16_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void sendUDP(bool extend) {
  if ((driverPort != 0) && ((main_id != 0) || (extended_id != 0))) {
    udp.beginPacket(udpAddress, driverPort);
    if (extend) {
      udp.write((uint8_t*)&payloadext, sizeof(payloadext));
    }
    else {
      udp.write((uint8_t*)&payload, sizeof(payload));
    }

    if (udp.endPacket()) {
      last_udp_check = millis();
    }

    DEBUG_PRINT.print("main,");
    DEBUG_PRINT.print(payload.id);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payload.x);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payload.y);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payload.z);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payload.w);
    DEBUG_PRINT.println();

    DEBUG_PRINT.print("extended,");
    DEBUG_PRINT.print(payloadext.id_ext);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payloadext.x_ext);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payloadext.y_ext);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payloadext.z_ext);
    DEBUG_PRINT.print(",");
    DEBUG_PRINT.print(payloadext.w_ext);
    DEBUG_PRINT.println();
  }
}

void setIMUMode(uint8_t mode, uint8_t mode_ext) {
  if (imu_mode != mode) {
    imu_mode = mode;
    switch (imu_mode) {
      case ROTVEC:
        //      myIMU.enableARVRStabilizedRotationVector(10);
        //      myIMU.enableARVRStabilizedGameRotationVector(0);
        DEBUG_PRINT.println("AR VR Stabilized Rotation Vector enabled on main IMU");
        break;

      case GAMEROTVEC:
        //      myIMU.enableARVRStabilizedRotationVector(0);
        //      myIMU.enableARVRStabilizedGameRotationVector(10);
        DEBUG_PRINT.println("AR VR Stabilized Game Rotation Vector enabled on main IMU");
        break;
    }
    first_read = false;
  }
  
  if (extended_imu_found && (imu_mode_ext != mode_ext)) {
    imu_mode_ext = mode_ext;
    switch (imu_mode_ext) {
      case ROTVEC:
        //      myIMU2.enableARVRStabilizedRotationVector(10);
        //      myIMU2.enableARVRStabilizedGameRotationVector(0);
        DEBUG_PRINT.println("AR VR Stabilized Rotation Vector enabled on extended IMU");
        break;

      case GAMEROTVEC:
        //      myIMU2.enableARVRStabilizedRotationVector(0);
        //      myIMU2.enableARVRStabilizedGameRotationVector(10);
        DEBUG_PRINT.println("AR VR Stabilized Game Rotation Vector enabled on extended IMU");
        break;
    }
    first_read_ext = false;
  }
}

void setWiFiPower(uint8_t power) {
  if (wifi_power != power) {
    wifi_power = power;
    esp_wifi_set_max_tx_power(wifi_power);
    DEBUG_PRINT.print("WiFi TX Power: ");
    DEBUG_PRINT.println(wifi_power);
    int8_t power_check;
    esp_wifi_get_max_tx_power(&power_check);
    DEBUG_PRINT.print("WiFi TX Power check: ");
    DEBUG_PRINT.println(power_check);
  }
}

void setWiFiSleep(uint8_t sleep) {
  if (wifi_sleep != sleep) {
    wifi_sleep = sleep;
    WiFi.setSleep(wifi_sleep);
    DEBUG_PRINT.print("WiFi Sleep Mode: ");
    DEBUG_PRINT.println(wifi_sleep);
    DEBUG_PRINT.print("WiFi Sleep Mode check: ");
    DEBUG_PRINT.println(WiFi.getSleep());
  }
}

Vyro vyro;

void setup() {

  Serial.begin(115200);
  while (!Serial);    // wait for the serial port to open
  DEBUG_PRINT.println("Serial set up!");

  Wire.begin(32 /*SDA Pin*/, 33 /*SCL Pin*/, 400000 /*Speed*/);
  DEBUG_PRINT.println("Wire set up!");

  if (!brown_en) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  }

  setCpuFrequencyMhz(80);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  ping.a = mac[0];
  ping.b = mac[1];
  ping.c = mac[2];
  ping.d = mac[3];
  ping.e = mac[4];
  ping.f = mac[5];

  pingbroad.a = mac[0];
  pingbroad.b = mac[1];
  pingbroad.c = mac[2];
  pingbroad.d = mac[3];
  pingbroad.e = mac[4];
  pingbroad.f = mac[5];

  ping.extend = true;
  pingbroad.extend = true;
  DEBUG_PRINT.println("Extended IMU found");

  WiFi.onEvent(WiFiEvent);
  //Wire.begin(32, 33, 400000);

  esp_wifi_set_max_tx_power(wifi_power);
  WiFi.setSleep(wifi_sleep);

  connectToWiFi(networkSSID, networkPassword);

/*    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, HIGH);
    reset_imu();
    uint32_t imu_init_time = millis();
    while (!BMI160.begin(BMI160GenClass::SPI_MODE, 5)) {
      reset_imu();
      delay(500);
      if (millis() - imu_init_time >= 3000) {
        ESP.deepSleep(1000);
      }
    }
  
    BMI160.setGyroOffsetEnabled(true);
    BMI160.autoCalibrateGyroOffset();
  Wire.setClock(400000);
*/

/*  if (imu.begin(BMI160GenClass::I2C_MODE, 0x68)) {
    extended_imu_found = true;
    ping.extend = true;
    pingbroad.extend = true;
    DEBUG_PRINT.println("Extended IMU found");

    imu.setGyroRate(BMI160_GYRO_RATE_800HZ);
    delay(1);
    imu.setAccelRate(BMI160_ACCEL_RATE_800HZ);
    delay(1);
    imu.setFullScaleGyroRange(BMI160_GYRO_RANGE_500);
    delay(1);
    imu.setFullScaleAccelRange(BMI160_ACCEL_RANGE_4G);
    delay(1);
    imu.setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
    delay(1);
    imu.setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
    delay(1);

    imu.setGyroOffsetEnabled(true);
    imu.setAccelOffsetEnabled(true);
  }
  else {
    extended_imu_found = false;
    ping.extend = false;
    pingbroad.extend = false;
    DEBUG_PRINT.println("Extended IMU not found");
  }*/

  vyro = Vyro();
  vyro.trackers.initAllChannels();

  DEBUG_PRINT.println("All channels are supposed to be initialized!");

  extended_imu_found = vyro.trackers[6]->extend;

  pinMode(led_pin, OUTPUT);

  t_blink = millis();

  if (!brown_en) {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 1);
  }

  DEBUG_PRINT.println("Finished setup!");
}

void loop() {
  
  serialLength = Serial.available();
  if (serialLength) {
    Serial.readBytesUntil('\n', header, sizeof(header));
    if (String(header) == "111") {
      Serial.readBytesUntil('\n', networkSSID, sizeof(networkSSID));
      Serial.readBytesUntil('\n', networkPassword, sizeof(networkPassword));
      connectToWiFi(networkSSID, networkPassword);
      Serial.write(110);
    }
    else {
      try {
        char junk[serialLength];
        Serial.readBytes(junk, sizeof(junk));
      }
      catch (String error) {
        DEBUG_PRINT.println(error);
      }
    }
    memset(header, 0, sizeof(header));
    memset(networkSSID, 0, sizeof(networkSSID));
    memset(networkPassword, 0, sizeof(networkPassword));
  }

  if (connected) {
    //DEBUG_PRINT.println("Connected DW.");
    
    udp.parsePacket();
    if (udp.read((uint8_t*)&ack, sizeof(ack)) == sizeof(Ack)) {
      if (ack.reply == 200 && ack.header == (uint8_t)'I' && ack.footer == (uint8_t)'i') {
        recv_watchdog = millis();
        udpAddress = udp.remoteIP();
        main_id = ack.id;
        extended_id = ack.id_ext;
        setIMUMode(ack.mode, ack.mode_ext);
        setWiFiPower(ack.wifi_power);
        setWiFiSleep(ack.wifi_sleep);
        serverPort = ack.serverPort;
        driverPort = ack.driverPort;
        if ((driverPort != 0) && ((main_id != 0) || (extended_id != 0))) {
          if (!streaming_udp) {
            streaming_udp = true;
            last_udp_check = millis();
          }
        }
        else {
          streaming_udp = false;
        }
      }
    }

    if (udpAddress != IPAddress(0, 0, 0, 0)) {

      if (!imu_start) {
        imu_start = true;
        last_main_imu_check = millis();
        last_extend_imu_check = millis();
      }

      if (!first_read) {
        last_main_imu_check = millis();
        last_udp_check = millis();
      }

      if (extended_imu_found && !first_read_ext) {
        last_extend_imu_check = millis();
      }

      Quaternion quaternion_main = vyro.trackers[6]->imu1.getQuaternion();

      if (vyro.trackers[6]->imu1.currentData.q != NULL) {
        last_main_imu_check = millis();
        first_read = true;
        payload.id = main_id;
        payload.x = map(quaternion_main.x);
        payload.y = map(quaternion_main.y);
        payload.z = map(quaternion_main.z);
        payload.w = map(quaternion_main.w);

        payloadext.id = main_id;
        payloadext.x = map(quaternion_main.x);
        payloadext.y = map(quaternion_main.y);
        payloadext.z = map(quaternion_main.z);
        payloadext.w = map(quaternion_main.w);

        //ping.accuracy = myIMU.getQuatAccuracy();

        sendUDP(extended_imu_found);
      }

      Quaternion quaternion_extend = vyro.trackers[6]->imu2.getQuaternion();

      if (vyro.trackers[6]->imu2.currentData.q != NULL) {
        last_extend_imu_check = millis();
        first_read_ext = true;
        payloadext.id_ext = extended_id;
        payloadext.x_ext = map(quaternion_extend.x);
        payloadext.y_ext = map(quaternion_extend.y);
        payloadext.z_ext = map(quaternion_extend.z);
        payloadext.w_ext = map(quaternion_extend.w);

        //ping.accuracy_ext = imu2.getQuatAccuracy();
      }


      if ((serverPort != 0) && (millis() - last_ping >= 1000)) {
        last_ping = millis();
        ping.id = main_id;
        ping.id_ext = extended_id;
        ping.batt = battery_voltage();
        ping.mode = imu_mode;
        ping.mode_ext = imu_mode_ext;
        ping.wifi_power = wifi_power;
        ping.wifi_sleep = wifi_sleep;
        udp.beginPacket(udpAddress, serverPort);
        udp.write((uint8_t*)&ping, sizeof(ping));
        udp.endPacket();
        DEBUG_PRINT.println(ping.batt);
      }

      if (millis() - last_main_imu_check >= 1000) {
        ESP.deepSleep(1000);
      }

      if (extended_imu_found && (millis() - last_extend_imu_check >= 1000)) {
        ESP.deepSleep(1000);
      }

      if (streaming_udp) {
        if (millis() - last_udp_check >= 1000) {
          ESP.deepSleep(1000);
        }
      }

      blink_led(5000);
    }

    else {
      if (millis() - last_broadcast >= 1000) {
        last_broadcast = millis();
        Audp.broadcastTo((uint8_t*)&pingbroad, sizeof(pingbroad), broadPort);
        DEBUG_PRINT.println("Pinged");
      }
      blink_led(2000);
    }
  }
  else {
    //DEBUG_PRINT.println("Started loop, waiting for PC connection.");
    blink_led(1000);
  }
  if (millis() - recv_watchdog >= 5000) {
    udpAddress = IPAddress(0, 0, 0, 0);
    serverPort = 0;
    driverPort = 0;
    streaming_udp = false;
    imu_start = false;
  }
}

void connectToWiFi(const char * ssid, const char * pwd) {
  DEBUG_PRINT.println("Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, pwd);
  esp_wifi_set_max_tx_power(wifi_power);
  WiFi.setSleep(wifi_sleep);
  DEBUG_PRINT.println("Waiting for WIFI connection...");
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      DEBUG_PRINT.print("WiFi connected! IP address: ");
      DEBUG_PRINT.println(WiFi.localIP());
      udp.begin(WiFi.localIP(), broadPort);
      last_broadcast = millis();
      last_ping = millis();
      last_main_imu_check = millis();
      last_extend_imu_check = millis();
      last_udp_check = millis();
      recv_watchdog = millis();
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      DEBUG_PRINT.println("WiFi lost connection");
      connected = false;
      udpAddress = IPAddress(0, 0, 0, 0);
      serverPort = 0;
      driverPort = 0;
      streaming_udp = false;
      imu_start = false;
      WiFi.reconnect();
      break;
    default: break;
  }
}
