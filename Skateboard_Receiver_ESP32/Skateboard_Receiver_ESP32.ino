#include <esp_now.h>
#include <math.h>
#include <SPI.h>
#include <VescUart.h>
#include <WiFi.h>
#include <elapsedMillis.h>

VescUart UART;
#define RXD2 16
#define TXD2 17
#define VESC_DEBUG 1

// Definitons for single bits 0 - 7
#define BIT0 (1<<0)
#define BIT1 (1<<1)
#define BIT2 (1<<2)
#define BIT3 (1<<3)
#define BIT4 (1<<4)
#define BIT5 (1<<5)
#define BIT6 (1<<6)
#define BIT7 (1<<7)

elapsedMillis sendTimer;
#define SEND_INTERVAL 1000
#define FANPIN 14
#define LEDPIN 12
#define BUZZER 26
#define WHEEL_DIAMETER 83  // In mm
#define GEAR_RATIO 2.2

#define RED 32
#define GREEN 33
#define BLUE 25

// Battery sensor declarations
// Number of readings to average
#define NUM_READINGS 100
#define BAT_PRECISION 1
int8_t batteryCounter = 0, initialAverageFlag = 0;
int8_t batFlag = 0;
float adjust = -.03;
float total = 0, avgBatVoltage = 0;
/* some definitions
   batVoltage (supposed voltage)
   avgBatVoltage (calculated)
*/

// Radio data declarations (data sent and received)
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

// Radio hardware definitions
typedef struct senderMessage
{
  uint16_t mph;
  long rpm;
  float voltage;
}senderMessage;

typedef struct receiverMessage
{
  uint8_t settings;
  uint32_t speed;
}receiverMessage;

uint8_t broadcastAddress[] = {0x9C, 0x9C, 0x1F, 0xEA, 0x19, 0xF0};
// This device - 9C:9C:1F:C7:EA:C8

senderMessage senderData;
receiverMessage receiverData;

uint8_t headlightFlag = 0, dutyFlag = 0, rpmFlag = 0, currentFlag = 0;

int16_t mphInt = 0;
float rpmToMphCoeff = 0;
float skateboardVoltFloat = 0;  // Multiply by 100, cast to int then send as int

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
  speed = receiverData.speed;
}

void sendRadioData(void)
{
  senderData.mph = mphInt;
  senderData.rpm = motorRpm;
  senderData.voltage = avgBatVoltage;
  esp_now_send(broadcastAddress, (uint8_t *) &senderData, sizeof(senderData));
}

void printBatteryData()
{
  Serial.print(batVoltage);
  Serial.print("  ");
  Serial.print(batPercent);
  Serial.println("");
}


uint8_t getBit(uint8_t byte, uint8_t bit)
{
  return ( (byte & bit) ? 1 : 0 );
}

void setBit(uint8_t *byte, uint8_t bit, uint8_t value)
{
  if (value == 0)
  {
    // Clear the specified bit to 0
    *byte &= ~(bit);
  }
  else
  {
    // Set the specified bit to 1
    *byte |= bit;
  }
}

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

void printRadioData()
{
  // Printed in the order they occur from the byte
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
    batFlag = 1;

    if (VESC_DEBUG == 1)
    {
      Serial.println("Fetching VESC data");
    }       
      
    motorRpm = UART.data.rpm;
    tachometer = UART.data.tachometer;
    batVoltage = UART.data.inpVoltage;

    // Wheel speed in mph based on motor rpm
    mphInt = (int)(motorRpm * rpmToMphCoeff);
  }
  else
  {
    batFlag = 0;

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

void getBattery(void)
{
  if (batFlag == 1)
  {
    // NOTE: using a 12 bit reading resolution
    if (batVoltage <= 0)
    {
      batteryCounter = 0;
      avgBatVoltage = 0;
    }

    if (batteryCounter <= NUM_READINGS)
    {
      total += batVoltage;
      batteryCounter++;
    }
    else if (batteryCounter > NUM_READINGS)
    {
      avgBatVoltage = (total / NUM_READINGS) + adjust;
      total = 0;
      batteryCounter = 0;
    }

    if (avgBatVoltage < 0)
    {
      // Never let the average of positive values be negative
      avgBatVoltage = 0;
    }
  }
}

void setup()
{
  // For computer debugging
  Serial.begin(115200);
  Serial.setTimeout(10);
  WiFi.mode(WIFI_STA);

  // For Vesc communication
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(10);

  UART.setSerialPort(&Serial2);

  // Print some newlines to split up from the ESP32 data
  Serial.print("\n\n");

  // Throw an error message if init error, keep initializing until it works
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

  // Blink the leds to indicate setup is complete
  blinkStatusLeds();

  rpmToMphCoeff = (60 * M_PI * WHEEL_DIAMETER) / (1000 * 1610 * GEAR_RATIO);

  Serial.println("Ready");
  delay(1000);
  sendTimer = 0;
}

void loop()
{
  getVescData();
  getBattery();
  // Testing
  // mphInt = 10;
  // batVoltage = 12;

  // printRadioData();
  // printBatteryData();
  // printVescData();
  settingEnable = (currentFlag == 0  &&  rpmFlag == 0  &&  dutyFlag == 0) ? 0 : 1;

  // Motor is allowed to be on
  if (settingEnable == 1)
  {
    digitalWrite(GREEN, HIGH);
    // We'll have this indicate when we can use the motor
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
  else  // Motor is not enabled
  {
    digitalWrite(GREEN, LOW);
    speed = 0;
    UART.setCurrent(0);
  }

  if (sendTimer > SEND_INTERVAL)
  {
    sendRadioData();
    sendTimer = 0;
  }

  digitalWrite(LEDPIN, (headlightFlag == 1) ? HIGH : LOW );
  digitalWrite(RED, (headlightFlag == 1) ? HIGH : LOW );

  // This is extremely important for preventing watchdog timeouts
  delay(5);
}
