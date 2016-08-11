#ifndef _WIFI_FW_H
#define _WIFI_FW_H

#define DEFAULT_SSID_LENGTH 16

#define _MISO  12
#define _MOSI  13
#define _SCK   14
#define _SS    15

/*V2
  #define _MISO  14
  #define _MOSI  12
  #define _SCK   13
  #define _SS  15
*/

#define LED 2

#define ARM_CS_Enable() digitalWrite(_SS, 0)
#define ARM_CS_Disable() digitalWrite(_SS, 1)




uint8_t setPIDgain_ok = 1;

typedef struct {
  int8_t startByte;
  int8_t roll;
  int8_t pitch;
  int8_t throttle;
  int8_t yaw;
  int8_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} ControlData;

typedef struct {
  int8_t startByte;
  int8_t startByte2;
  int8_t yawPitchRoll;
  int16_t kp;
  int16_t ki;
  int16_t kd;
  int16_t checksum;
  char ssid[DEFAULT_SSID_LENGTH];
} TuningData;


WiFiUDP udp;
byte data[512] = {0};
unsigned int localPort = 12345;
TuningData tuningData[3] = {0};
String output = "";
String lastOutput = "";
String yawPitchRollText[3] = {"Yaw..:", "Pitch:", "Roll.:"};
char accessPointName[DEFAULT_SSID_LENGTH] = {'\0'};
char accessPointPassword[9] = {'\0'};
char defaultESPWiFiName[DEFAULT_SSID_LENGTH] = {'\0'};

int8_t Trim_value[3] = {0};
Ticker ticker_DEGUG;

static const int numberOfLines = 5;
String line[numberOfLines] = {""};
int currentLine = 0;
bool isUpdated = true;

IPAddress local_ip(192, 168, 5, 1);
IPAddress gateway(192, 168, 5, 1);
IPAddress subnet(255, 255, 255, 255);



void addLine(String string);
String floatToString(float value, int length, int decimalPalces);
String hexToString(byte value);
String intToString(int value, int length);
String ipToString(IPAddress ip);
bool isSSID(char* ssid);
void loadTuningData(void);
String readEEPROM(int index, int length);
void loadTrimData(void);
void saveTrimData(int8_t *tmp);
void saveTuningData(int i);
int writeEEPROM(int index, String text);

void blink(void);
uint8_t setPIDgain(void);
void sentControlcommand(int8_t roll_tmp, int8_t pitch_tmp, int8_t throttle_tmp, int8_t yaw_tmp);
void Read_udp(void);
void sentTrimcommand(void);
void sentFunctioncommand(int8_t FN);
int8_t limmit(int8_t value, int8_t low_b, int8_t high_b);

uint8_t spi_transfer(uint8_t b) ;
void spi_setup();


void init_drone(void);

void init_drone(void)
{
  delay(2000);

  pinMode(LED, OUTPUT);
  pinMode(_SS, OUTPUT);

  ARM_CS_Disable();

  Serial.begin(115200);


  WiFi.mode(WIFI_STA);
  delay(50);

  WiFi.disconnect();
  delay(200);


  byte mac[6] = {0};
  WiFi.macAddress(mac);

  sprintf(accessPointName, "BDrone-%lu", ESP.getChipId());
  sprintf(accessPointPassword, "%lu", ESP.getChipId());

  Serial.println("");
  Serial.println(accessPointPassword);

  if (accessPointName[13] == '\0')
  {
    accessPointName[13] =  '0';
    accessPointName[14] =  '0';
  }
  accessPointName[15] =  {'\0'};

  if (accessPointPassword[6] == '\0')
  {
    accessPointPassword[6] =  '0';
    accessPointPassword[7] =  '0';
  }

  accessPointPassword[8] = {'\0'};

  Serial.println(accessPointPassword);

  WiFi.softAP(accessPointName, accessPointPassword);
  delay(50);
  accessPointName[DEFAULT_SSID_LENGTH - 1] = {'\0'};

  WiFi.mode(WIFI_AP_STA);
  delay(50);

  udp.begin(localPort);
  delay(50);

  spi_setup();
  delay(50);

  EEPROM.begin(512);
  delay(50);

  loadTuningData();
  loadTrimData();

  delay(1000);

  setPIDgain();
  delay(50);
  setPIDgain();

  ticker_DEGUG.attach_ms(1000, blink);

  Serial.println();
  Serial.println(String("Name: ") + accessPointName);
  Serial.println(String("Pass: ") + accessPointPassword);
  Serial.println(String("Port: ") + localPort);
  Serial.println();



}

void Read_udp(void)
{
  int numberOfBytes = udp.parsePacket();

  if (numberOfBytes > 0)
  {
    udp.read(data, numberOfBytes);
    if (data[0] == 0XF0 && data[1] == 0XF0) // tuning data
    {
      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));

      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (tuningDataBuffer.yawPitchRoll >= 0X01 && tuningDataBuffer.yawPitchRoll <= 0X03 && tuningDataBuffer.checksum == checksum && isSSID(tuningDataBuffer.ssid))
      {
        int i = tuningDataBuffer.yawPitchRoll - 1;
        tuningData[i] = tuningDataBuffer;
        saveTuningData(i);

        if (tuningDataBuffer.yawPitchRoll == 0X03) setPIDgain();

        float kp = tuningDataBuffer.kp / 10.0f;
        float ki = tuningDataBuffer.ki / 10.0f;
        float kd = tuningDataBuffer.kd / 10.0f;

        String string = "";

        if (i == 0) {
          string += String("Receive tuning data from ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
          Serial.println(string);
          addLine("Receive tuning data from");
          addLine(ipToString(udp.remoteIP()) + " Port " + udp.remotePort());
        }
      }
    } else if (data[0] == 0XFC && data[1] == 0XFC) { // get tuning data

      TuningData tuningDataBuffer = {0};
      memcpy(&tuningDataBuffer, data, sizeof(TuningData));
      int16_t checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
      tuningDataBuffer.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (strlen(tuningDataBuffer.ssid) == 0) {
        memcpy(tuningDataBuffer.ssid, accessPointName, DEFAULT_SSID_LENGTH);
      }

      if (tuningDataBuffer.checksum == checksum && isSSID(tuningDataBuffer.ssid)) {
        // send tuning data to remote control
        for (tuningDataBuffer.yawPitchRoll = 0X01; tuningDataBuffer.yawPitchRoll <= 0X03; tuningDataBuffer.yawPitchRoll++) {
          int i = tuningDataBuffer.yawPitchRoll - 1;
          tuningDataBuffer.kp = tuningData[i].kp;
          tuningDataBuffer.ki = tuningData[i].ki;
          tuningDataBuffer.kd = tuningData[i].kd;
          tuningDataBuffer.checksum = tuningDataBuffer.yawPitchRoll + tuningDataBuffer.kp + tuningDataBuffer.ki + tuningDataBuffer.kd;
          memcpy(&data, &tuningDataBuffer, sizeof(TuningData));

          String string = "";

          if (i == 0) {
            string += String("Send tuning data to ") + ipToString(udp.remoteIP()) + " port " + udp.remotePort() + ":";
            Serial.println(string);
            addLine("Send tuning data to");
            addLine(ipToString(udp.remoteIP()) + " Port " + udp.remotePort());
          }

          string = hexToString(tuningDataBuffer.startByte) + ", ";
          string += hexToString(tuningDataBuffer.startByte2) + ", ";
          string += hexToString(tuningDataBuffer.yawPitchRoll) + ", ";
          string += intToString(tuningDataBuffer.kp, 4) + ", " + intToString(tuningDataBuffer.ki, 4) + ", " + intToString(tuningDataBuffer.kd, 4) + ", " + intToString(tuningDataBuffer.checksum, 4);
          Serial.println(string);

          String kpString = floatToString(tuningDataBuffer.kp / 10.0f, 5, 1);
          String kiString = floatToString(tuningDataBuffer.ki / 10.0f, 5, 1);
          String kdString = floatToString(tuningDataBuffer.kd / 10.0f, 5, 1);
          addLine(String(yawPitchRollText[i]) + kpString + "," + kiString + "," + kdString);

          udp.beginPacket(udp.remoteIP(), udp.remotePort());
          udp.write(data, sizeof(TuningData));
          udp.endPacket();
          delay(200); // don't forget to delay
        }
      }
    } else if (data[0] == 0XFE) { // trim and control

      static ControlData controlData_prev;
      ControlData controlData = {0};
      memcpy(&controlData, data, sizeof(ControlData));
      int8_t checksum = controlData.roll + controlData.pitch + controlData.throttle + controlData.yaw;
      controlData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';

      if (controlData.checksum == checksum && isSSID(controlData.ssid))
      {
        int8_t trimFlag = 0XFF;

        if (controlData.roll == trimFlag && controlData.pitch == trimFlag && controlData.throttle == trimFlag && controlData.yaw == trimFlag) {

          // Trim //
          int8_t trim_tmp_0[3] = {0};

          trim_tmp_0[0] = limmit(controlData_prev.roll,  -15, 15);
          trim_tmp_0[1] = limmit(controlData_prev.pitch, -15, 15);
          trim_tmp_0[2] = limmit(controlData_prev.yaw,   -15, 15);

          saveTrimData(Trim_value);
          memcpy(&Trim_value, &trim_tmp_0, 3);

          sentTrimcommand();
          Serial.println("Trim");

        } else {

          // control //
          sentControlcommand(controlData.roll + Trim_value[0], controlData.pitch + Trim_value[1] , controlData.throttle, controlData.yaw + Trim_value[2]);
          memcpy(&controlData_prev, data, sizeof(ControlData));
          blink();

          String rollString = intToString(controlData.roll, 4);
          String pitchString = intToString(controlData.pitch, 4);
          String throttleString = intToString(controlData.throttle, 3);
          String yawString = intToString(controlData.yaw, 4);
          output = String("Roll ") + rollString + ", Pitch " + pitchString + ", Throttle " + throttleString + ", Yaw " + yawString;



          if (output != lastOutput) {
            //   lastOutput = output;
            //   Serial.print(controlData.ssid); Serial.print(" ");
            Serial.println(output);
            //   addLine(String("|") + rollString + "|" + pitchString + "|" + throttleString + "|" + yawString + "|");
          }
        }
      }
    } 
    else if (data[0] == 0XF1) {
      ControlData FunctionData = {0};
      memcpy(&FunctionData, data, sizeof(ControlData));
      FunctionData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';
      if (isSSID(FunctionData.ssid)) {
        sentFunctioncommand(0XF1);
        Serial.println("F1");
      }

    } else if (data[0] == 0XF2) {
      ControlData FunctionData = {0};
      memcpy(&FunctionData, data, sizeof(ControlData));
      FunctionData.ssid[DEFAULT_SSID_LENGTH - 1] = '\0';
      if (isSSID(FunctionData.ssid)) {
        sentFunctioncommand(0XF2);
        Serial.println("F2");
      }
    }
  }
}

void addLine(String string)
{
  if (currentLine == numberOfLines - 1 && line[currentLine].length() > 0)
  {
    for (int i = 0; i < numberOfLines - 1; i++) {
      line[i] = line[i + 1];
    }
  }

  line[currentLine] = string;

  if (++currentLine >= numberOfLines)
  {
    currentLine = numberOfLines - 1;
  }

  isUpdated = true;
}

String floatToString(float value, int length, int decimalPalces)
{
  String stringValue = String(value, decimalPalces);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += " ";
  }

  return prefix + stringValue;
}

String hexToString(byte value)
{
  int length = 2;
  String stringValue = String(value, HEX);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += "0";
  }

  return "0x" + prefix + stringValue;
}

String intToString(int value, int length)
{
  String stringValue = String(value);
  String prefix = "";

  for (int i = 0; i < length - stringValue.length(); i++) {
    prefix += " ";
  }

  return prefix + stringValue;
}

String ipToString(IPAddress ip)
{
  return String(ip[0]) + "." + ip[1] + "." + ip[2] + "." + ip[3];
}

bool isSSID(char* ssid) {
  return (strcmp(ssid, accessPointName) == 0 || strcmp(ssid, defaultESPWiFiName) == 0 || strlen(ssid) == 0);
}

void loadTuningData()
{
  for (int i = 0; i < 3; i++) {
    int address = (i + 1) * 12;

    String str = readEEPROM(address, 4);
    tuningData[i].kp = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].ki = str.toInt();
    address += 4;

    str = readEEPROM(address, 4);
    tuningData[i].kd = str.toInt();
  }
}

void saveTuningData(int i)
{
  int address = (i + 1) * 12;

  String str = String(tuningData[i].kp);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].ki);
  writeEEPROM(address, str);
  address += 4;

  str = String(tuningData[i].kd);
  writeEEPROM(address, str);
}

void loadTrimData(void)
{
  for (int i = 0; i < 3; i++)
  {
    int address = (i) * 2;

    Trim_value[i] = EEPROM.read(address);
  }
}

void saveTrimData(int8_t *tmp)
{
  for (int i = 0; i < 3; i++)
  {
    int address = (i) * 2;
    EEPROM.write(address, *tmp + i );
  }
  EEPROM.commit();
}

String readEEPROM(int index, int length) {
  String text = "";
  char ch = 1;

  for (int i = index; (i < (index + length)) && ch; ++i) {
    if (ch = EEPROM.read(i)) {
      text.concat(ch);
    }
  }

  return text;
}

int writeEEPROM(int index, String text) {
  for (int i = index; i < text.length() + index; ++i) {
    EEPROM.write(i, text[i - index]);
  }

  EEPROM.write(index + text.length(), 0);
  EEPROM.commit();

  return text.length() + 1;
}


void blink(void)
{
  static int8_t led = 0;
  led = 1 - led;
  digitalWrite(LED, led);
}

uint8_t setPIDgain(void)
{
  int16_t temp ;

  //1st
  uint8_t command = 0XFE;

  int8_t data_buffer[20] = {0};

  data_buffer[0] = (uint8_t)(tuningData[0].kp >> 8);
  data_buffer[1] = (uint8_t)(tuningData[0].kp);
  data_buffer[2] = (uint8_t)(tuningData[0].ki >> 8);
  data_buffer[3] = (uint8_t)(tuningData[0].ki);
  data_buffer[4] = (uint8_t)(tuningData[0].kd >> 8);
  data_buffer[5] = (uint8_t)(tuningData[0].kd);
  data_buffer[6] = (uint8_t)(tuningData[1].kp >> 8);
  data_buffer[7] = (uint8_t)(tuningData[1].kp);
  data_buffer[8] = (uint8_t)(tuningData[1].ki >> 8);
  data_buffer[9] = (uint8_t)(tuningData[1].ki);
  data_buffer[10] = (uint8_t)(tuningData[1].kd >> 8);
  data_buffer[11] = (uint8_t)(tuningData[1].kd);
  data_buffer[12] = (uint8_t)(tuningData[2].kp >> 8);
  data_buffer[13] = (uint8_t)(tuningData[2].kp);
  data_buffer[14] = (uint8_t)(tuningData[2].ki >> 8);
  data_buffer[15] = (uint8_t)(tuningData[2].ki);
  data_buffer[16] = (uint8_t)(tuningData[2].kd >> 8);
  data_buffer[17] = (uint8_t)(tuningData[2].kd);
  int16_t checksum_buffer = tuningData[0].kp + tuningData[0].ki + tuningData[0].kd + tuningData[1].kp + tuningData[1].ki + tuningData[1].kd + tuningData[2].kp + tuningData[2].ki + tuningData[2].kd ;
  data_buffer[18] = (uint8_t)(checksum_buffer >> 8);
  data_buffer[19] = (uint8_t)(checksum_buffer);

  for (int index = 0; index < 20; index++)
  {
    spi_transfer(data_buffer[index]);
  }

  spi_transfer(command);
  spi_transfer(command);

  // 2nd

  for (int index = 0; index < 20; index++)
  {
    spi_transfer(data_buffer[index]);
  }

  spi_transfer(command);
  spi_transfer(command);

  return 0;
}

void sentFunctioncommand(int8_t _FN)
{
  spi_transfer(_FN);
  spi_transfer(_FN);
  spi_transfer(_FN);
  spi_transfer(_FN);
}

void sentControlcommand(int8_t roll_tmp, int8_t pitch_tmp, int8_t throttle_tmp, int8_t yaw_tmp)
{
  // set pid in firstime
  if (setPIDgain_ok == 1)
  {
    setPIDgain_ok = 0;
    loadTuningData();
    delay(1);
    setPIDgain();
    delay(1);
    setPIDgain();
  }
  static int16_t throttle_tmp_buffer;
  //1st
  // Serial.println(analogRead(A0));

  if (analogRead(A0) < 610) // 3.1v
  {
    if (throttle_tmp_buffer > 0) throttle_tmp_buffer-- ;
    if (throttle_tmp > throttle_tmp_buffer / 10) throttle_tmp = throttle_tmp_buffer / 10;

  } else {
    throttle_tmp_buffer = throttle_tmp * 10;
  }

  int8_t command = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

  spi_transfer(roll_tmp);
  spi_transfer(pitch_tmp);
  spi_transfer(throttle_tmp);
  spi_transfer(yaw_tmp);
  spi_transfer(command);

  command = 0xFD;
  spi_transfer(command);
  spi_transfer(command);

  // 2nd
  command = roll_tmp + pitch_tmp + throttle_tmp + yaw_tmp;

  spi_transfer(roll_tmp);
  spi_transfer(pitch_tmp);
  spi_transfer(throttle_tmp);
  spi_transfer(yaw_tmp);
  spi_transfer(command);

  command = 0xFD;
  spi_transfer(command);
  spi_transfer(command);
}

void sentTrimcommand(void)
{
  //1st
  uint8_t command = 0xFC;
  spi_transfer(command);
  spi_transfer(command);

  // 2nd
  command = 0xFC;
  spi_transfer(command);
  spi_transfer(command);
}

int8_t limmit(int8_t value, int8_t low_b, int8_t high_b)
{
  if (value < low_b) value = low_b;
  if (value > high_b) value = high_b;
  return value;
}

const unsigned char msk[] = {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01};
void spi_setup()
{
  pinMode(_SS, OUTPUT);
  pinMode(_MISO, INPUT);
  pinMode(_MOSI, OUTPUT);
  pinMode(_SCK, OUTPUT);
  //
  digitalWrite(_SS, HIGH);
  digitalWrite(_SCK, LOW);
}

// chip selection has been declared outside
inline void spi_select() {
  digitalWrite(_SS, LOW);
}
inline void spi_unselect() {
  digitalWrite(_SS, HIGH);
}

//mode 0: SCK idle low, phase: reading at middle of SCK HIGH pulse
//mode 1: SCK idle low, phase: reading at middle of SCK LOW pulse
//this big-bang should work for both  CPHA=1  and CPHA=0
uint8_t spi_transfer(uint8_t b)
{
  spi_select(); // should be called outside, may be required by one transition

  for (uint8_t _bit = 0; _bit < 8; _bit++)
  {
    digitalWrite(_MOSI, (b & msk[_bit]));
    digitalWrite(_SCK, LOW);  // data will change at falling edge
    digitalWrite(_SCK, HIGH);
    digitalWrite(_SCK, LOW);  // data will change at falling edge
  }
  spi_unselect();
  delayMicroseconds(50);
  return 1;
}


#endif
