#include <esp_now.h>
#include <math.h>
#include <SPI.h>
#include <VescUart.h>
#include <WiFi.h>

VescUart UART;
#define RXD2 16
#define TXD2 17
#define VESC_DEBUG 1

// definitons for single bits 0 - 7
#define BIT0 (1<<0)
#define BIT1 (1<<1)
#define BIT2 (1<<2)
#define BIT3 (1<<3)
#define BIT4 (1<<4)
#define BIT5 (1<<5)
#define BIT6 (1<<6)
#define BIT7 (1<<7)

#define FANPIN 25
#define LEDPIN 26
#define BUZZER 27
#define WHEEL_DIAMETER 83  // in mm
#define GEAR_RATIO 2.2

#define RED 14
#define GREEN 12
#define BLUE 13

// radio data declarations (data sent and received)
/* Remote Data
   ===========
   settings: will hold headlight, 3 sending types, other data
   data:     used for sending 4-byte int
   speed:    abstract value for sending and interpretation using settings

   Receiver Data
   =============
   mph:      holds calculated mph of board
   voltage:  holds read voltage of board
*/
int32_t speed = 0;

/*
settings byte
00000000
^  ^ ^^^   The following variables correspond to the bits (in order)
*/

// Radio hardware definitions
typedef struct sender_message
{
  uint16_t mph;
  uint16_t voltage;
}sender_message;

typedef struct receiver_message
{
  uint8_t settings;
  uint32_t speed;
}receiver_message;

// DEVKIT V1 Remote    7C:9E:BD:F3:50:DC
// Standalone Remote   F0:08:D1:D1:6F:18

// Testing
// uint8_t broadcastAddress[] = {0x7C, 0x9E, 0xBD, 0xF3, 0x50, 0xDC};
uint8_t broadcastAddress[] = {0xF0, 0x08, 0xD1, 0xD1, 0x6F, 0x18};

sender_message senderData;
receiver_message receiverData;

uint8_t headlightFlag = 0, dutyFlag = 0, rpmFlag = 0, currentFlag = 0;

int16_t mphInt = 0;
float rpmToMphCoeff = 0;
int16_t skateboardVoltInt = 0;
float skateboardVoltFloat = 0;  // multiply by 100, cast to int then send as int

// Skateboard values
long motorRpm, tachometer;
float batVoltage, batPercent;

uint8_t statusCounter = 0;
uint8_t settingEnable = 0;

// Trigger an event when we send data
void OnDataSent(const uint8_t * mac, esp_now_send_status_t status)
{

}

// Process received data
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&receiverData, incomingData, sizeof(receiverData));
  getSettings(receiverData.settings);

  sendRadioData();
}

// Send the radio data (for readability)
void sendRadioData(void)
{
  senderData.mph = mphInt;
  senderData.voltage = skateboardVoltInt;
  esp_now_send(broadcastAddress, (uint8_t *) &senderData, sizeof(senderData));
}

void printBatteryData()
{
  Serial.print(batVoltage);
  Serial.print("  ");
  Serial.print(batPercent);
  Serial.println("");
}

// getters and setters for bitwise operations
uint8_t getBit(uint8_t byte, uint8_t bit)
{
  // return value of a bit in byte
  return ( (byte & bit) ? 1 : 0 );
}

void setBit(uint8_t *byte, uint8_t bit, uint8_t value)
{
  // set the bit of the byte to the respective value (1 or 0)
  if (value == 0)
  {
    // clear the specified bit to 0
    *byte &= ~(bit);
  }
  else
  {
    // set the specified bit to 1
    *byte |= bit;
  }
}

// get the settings and store them into the corresponding vars
/*
   Settings Byte:

   01234567
   ^  ^ ^^

  lsb
      0   currentFlag
      1   rpmFlag
      2   dutyFlag
      3
      4   headlightFlag
      5
      6
      7
  msb
*/
void getSettings(uint8_t settings)
{
  currentFlag = getBit(settings, BIT0);
  rpmFlag = getBit(settings, BIT1);
  dutyFlag = getBit(settings, BIT2);

  headlightFlag = getBit(settings, BIT4);
}

// Print general settings for debugging
void printRadioData()
{
  // print the settings values in the order they occur from the byte

  Serial.print(currentFlag);
  Serial.print("  ");
  Serial.print(rpmFlag);
  Serial.print("  ");
  Serial.print(dutyFlag);
  Serial.print("  ");
  Serial.print(headlightFlag);
  Serial.print("  ");
  Serial.print(speed);
  Serial.print("  ");

  Serial.println("  ");
}

void blinkStatusLeds()
{
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
  delay(250);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
  delay(350);
  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, HIGH);
  digitalWrite(BLUE, HIGH);
  delay(250);
  digitalWrite(RED, LOW);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);
}

// Get all of the necessary vesc values
void getVescData()
{
  if ( UART.getVescValues() )
  {
    if (VESC_DEBUG == 1)
      Serial.println("Fetching VESC data");
    // we have more values that we can get,
    // but we only need these for now ...
    motorRpm = UART.data.rpm;
    tachometer = UART.data.tachometer;
    batVoltage = UART.data.inpVoltage;

    // wheel speed in mph based on motor rpm
    mphInt = (int)(motorRpm * rpmToMphCoeff);
  }
  else
  {
    if (VESC_DEBUG == 1)
      Serial.println("Unable to get VESC data");
  }
}

void printVescData()
{
  Serial.print(motorRpm);
  Serial.print("  ");
  Serial.print(tachometer);
  Serial.print("  ");
  Serial.print(batVoltage);
  Serial.println("  ");
}

void setup()
{
  // for debugging with computer
  Serial.begin(115200);
  Serial.setTimeout(10);
  WiFi.mode(WIFI_STA);

  // for Vesc communication
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(10);

  UART.setSerialPort(&Serial2);

  // Print some newlines to split up from the ESP32 data
  Serial.print("\n\n");

  // throw an error message if init error, keep initializing until it works
  while (esp_now_init() != ESP_OK)
  {
    Serial.print("Radio init FAILED: ");
    delay(500);
  }

  Serial.println("Radio init successful");
  delay(500);

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  while (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    delay(500);
  }

  Serial.println("Add peer successful");
  delay(500);

  pinMode(FANPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  // blink the leds to indicate setup is complete
  blinkStatusLeds();

  rpmToMphCoeff = (M_PI * WHEEL_DIAMETER * 60) / (304.8 * 5280 * GEAR_RATIO);

  Serial.println("Ready");
  delay(1000);
}

void loop()
{
  getVescData();
  // Testing
  mphInt = 10;
  skateboardVoltInt = 12;

  // printRadioData();
  // printBatteryData();
  printVescData();

  settingEnable = (currentFlag == 0  &&  rpmFlag == 0  &&  dutyFlag == 0) ? 0 : 1;

  if (settingEnable == 1)  // motor is enabled
  {
    digitalWrite(GREEN, HIGH);
    // we'll have this indicate when the motor is okay to use
    if (currentFlag == 1)
    {
      if (speed > 0)
      {
        UART.setCurrent(speed);
      }
      // We need to make sure to consider when to brake
      else if (speed < 0)
      {
        UART.setBrakeCurrent(speed);
      }
    }
    else if (rpmFlag == 1)
    {
      UART.setRPM(speed);
    }
    else if (dutyFlag == 1)
    {
      UART.setDuty((float)speed / 100);
    }
  }
  else  // motor is not enabled
  {
    digitalWrite(GREEN, LOW);
    speed = 0;
    UART.setCurrent(0);
  }

  // write LEDPIN & RED whatever the flag is (logic wise)
  digitalWrite(LEDPIN, (headlightFlag == 1) ? HIGH : LOW );
  digitalWrite(RED, (headlightFlag == 1) ? HIGH : LOW );


  // This is extremely important for preventing watchdog timeouts
  delay(5);
}
