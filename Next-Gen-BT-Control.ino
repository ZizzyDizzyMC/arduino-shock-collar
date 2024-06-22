#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <SPI.h>


// Adding heltec stuff for OLED display.
#include "heltec.h"
#include <analogWrite.h>

// Bluetooth Portion.

BLEServer* pServer = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
String bleAddress = ""; // CONFIGURATION: < Use the real device BLE address here.
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
uint32_t max_power = 128;
uint32_t max_vibe = 200;
int vibration;
int vibration_one;
int vibration_two;

const int pin_motor_a = 12;
const int pin_motor_b = 14;

#define SERVICE_UUID           ""
#define CHARACTERISTIC_RX_UUID ""
#define CHARACTERISTIC_TX_UUID ""

// End of bluetooth portion.

// Collar Portion
byte buttonPin = 0;

//=================================================== START OF COLLAR SETUP CODE ======================================================================

//const int shock_min = 0; // Minimum of power a command will be executed at
const int shock_delay = 10; // Maximum rate at which the shock function can be used at
const int cmd_max = 1500; // Maximum of milliseconds which a command can be executed at

// Constant variables
const int pin_led = LED_BUILTIN; // Pin for indication LED
const int pin_rtx =  27; // Pin to transmit over
const String key = "01000110010101100"; // Key of the transmitter, dont touch if you dont know how it works  //00101100101001010 old key
//const int pin_vext = 21; // VEXT pin power.
// Variables which do change
int collar_chan = 0; // Can be channel 0 or 1
int collar_duration = 1000; // Duration of the command in milliseconds
int collar_power = 10; // Strength of the command, can be 0-100, but will be limited by shock_min and shock_max
int collar_vibe_zero = 0;
int collar_vibe_one = 0;
int motor_vibe_a = 0;
int motor_vibe_b = 0;

int vibration_collar_zero;
int vibration_collar_one;
int last_vibe_one = 0;
int last_vibe_two = 0;
int vibe_one = 0;
int vibe_two = 0;
// Define values for easier recognition
#define COLLAR_LED 1
#define COLLAR_BEEP 2
#define COLLAR_VIB 3
#define COLLAR_ZAP 4

// Strings used for building up the command sequence
String sequence, power, channelnorm, channelinv, modenorm, modeinv, incomingString, command;

// Store the last time anything was transmitted to the collar
unsigned long transmit_last = 0;
unsigned long shock_last = 0;
unsigned long lastTrans = 0;
unsigned long transmit_start = 0;
unsigned long transmit_start_zero = 0;
unsigned long transmit_start_one = 0;
unsigned long lastUpdate = 0;

// Store the last command sent to motors.

unsigned long motor_start_a = 0;
unsigned long motor_start_b = 0;
int cArray[6];
char testing[50];

struct transmitParams {
  int c, m, p, d;
};

void transmitCommand(int c, int m, int p, int d)
{
  transmit_last = millis();
  switch (c) // Check the channel
  {
    case 1: // Channel 1
      channelnorm = "111";
      channelinv = "000";
      break;
    default: // Channel 0
      channelnorm = "000";
      channelinv = "111";
      break;
  }

  switch (m) // Check the mode
  {
    case 1: // Light
      modenorm = "1000";
      modeinv = "1110";
      break;
    case 2: // Beep
      modenorm = "0100";
      modeinv = "1101";
      break;
    case 4: // Shock
      modenorm = "0001";
      modeinv = "0111";
      shock_last = millis();
      break;
    default: // Vibrate
      modenorm = "0010";
      modeinv = "1011";
//      p = 10; // Set strengh to 10 for the command to be executed properly
      break;
  }

  // Convert power to binary
  int zeros = String(p, BIN).length();

  String power;
  for (int i = 0; i < 7 - zeros; i++)
  {
    power = power + "0";
  }
  power = power + String(p, BIN);

  String sequence = "1" + channelnorm + modenorm + key + power + modeinv + channelinv + "00";

  //digitalWrite(pin_led, HIGH);
  d = d * 200;
  d = constrain(d, 200, cmd_max); // Clamp duration of the command
  unsigned long cmd_start = millis();
  while (millis() - cmd_start < d)  {
    // start bit
    digitalWrite(pin_rtx, HIGH);
    delayMicroseconds(800); // chnged to new protocol Z+ - was 400
    digitalWrite(pin_rtx, LOW);
    delayMicroseconds(750);// wait 750 uS

    for (int n = 0; n < 41 ; n++)
    {
      if (sequence.charAt(n) == '1') // Transmit a one
      {
        digitalWrite(pin_rtx, HIGH);
        delayMicroseconds(200); // chnged to new protocol
        digitalWrite(pin_rtx, LOW);
        delayMicroseconds(1500); // chnged to new protocol
      }
      else // Transmit a zero
      {
        digitalWrite(pin_rtx, HIGH);
        delayMicroseconds(200); // chnged to new protocol
        digitalWrite(pin_rtx, LOW);
        delayMicroseconds(750); // chnged to new protocol
      }
    }
    //delayMicroseconds(7000); // chnged to new protocol
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
  //digitalWrite(pin_led, LOW);
  vTaskDelete(NULL);
}

void collar_keepalive()
{
  if (millis() - transmit_last >= 120000) // Send command to the collar at least every 2 minutes to make it stay on
  {
    Serial.println("Keep-alive:\tCollar");
    transmit_command(collar_chan, COLLAR_LED, 1, 1);
    //transmit_command(collar_chan, int * COLLAR_LED, int * 1, int * 1);
  }
}

void run_command(String a)
{
  //a = a + "," + 0;
  a.toCharArray(testing, 50);
  //char testing[50] = {a};
  command = strtok(testing, ",");
  int i = 0;
  while (command != 0)
    {
      Serial.println(command);
      cArray[i] = command.toInt();
      i++;
      command = strtok(0, ",");
    }
  transmit_command(cArray[0], cArray[1], cArray[2], cArray[3]);
}


// Bluetooth Stuff

void UpdateRF(void) {
  Serial.print("vibration");
  Serial.println(vibration);
  lastUpdate = millis();
  int power = map(vibration, 0 , 20 , 0, max_power);
  int vibe = map(vibration, 0, 20, 0, max_vibe);
  int vibe_one = map(vibration_one, 0, 20, 0, max_vibe);
  int vibe_two = map(vibration_two, 0, 20, 0, max_vibe);
  int vibe_collar_zero = map(vibration_collar_zero, 0, 20, 0, 50);
  int vibe_collar_one = map(vibration_collar_one, 0, 20, 0, 50);
  power = constrain(power, 0, max_power);
   if (vibe_one > 32) {
    
    analogWrite(pin_motor_a, vibe_one);
    } else {
    analogWrite(pin_motor_a, 0);
  }
  if (vibe_two > 32) {
    analogWrite(pin_motor_b, vibe_two);
  } else {
    analogWrite(pin_motor_b, 0);
  }
  
  // Collar Vibe Persistence.
  if(vibe_collar_zero > 0) {
    transmit_command(0, COLLAR_VIB, vibe_collar_zero, 2);
  }
  if(vibe_collar_one > 0) {
    transmit_command(1, COLLAR_VIB, vibe_collar_one, 2);
  }
}

void transmitter(void* userParams) {
  transmitParams* params = (transmitParams*)userParams;
  transmitCommand(params->c, params->m, params->p, params->d);
  free((void*)params);
  vTaskDelete(NULL);
}

void transmit_command(int c, int m, int p, int d){
  transmitParams* params = (transmitParams*)(malloc(sizeof(transmitParams)));
    params->c = c;
    params->m = m;
    params->p = p;
    params->d = d;
    xTaskCreate(transmitter,"transmit data",10000,(void*)params,0, NULL);
}


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      Serial.println("Oh god we disconnected");
      analogWrite(pin_motor_a, 0);
      analogWrite(pin_motor_b, 0);
      deviceConnected = false;
    }
};



class MySerialCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      static uint8_t messageBuf[64];
      assert(pCharacteristic == pRxCharacteristic);
      std::string rxValue = pRxCharacteristic->getValue();
      
      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
      if (rxValue == "DeviceType;") {
        Serial.println("$Responding to Device Enquiry");
        memmove(messageBuf, "P:204:f4366e5239a5;", 19); //changed to E instead of P, E is for Ear, P is for Edge
        // CONFIGURATION:               ^ Use a BLE address of the Lovense device you're cloning.
        pTxCharacteristic->setValue(messageBuf, 19);
        pTxCharacteristic->notify();
      } else if (rxValue == "Battery;") {
        memmove(messageBuf, "69;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
      } else if (rxValue == "PowerOff;") {
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
      } else if (rxValue == "RotateChange;") {
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
      } else if (rxValue.rfind("GetLight;", 0) == 0) {
        Serial.println("Got Light Request, responding with Light:1;");
        memmove(messageBuf, "Light:1;", 8);
        pTxCharacteristic->setValue(messageBuf, 8);
        pTxCharacteristic->notify();
      } else if (rxValue.rfind("GetAS;", 0) == 0) {
        Serial.println("Got AS.");
        memmove(messageBuf, "AutoSwitch:0:1;", 15);
        pTxCharacteristic->setValue(messageBuf, 15);
        pTxCharacteristic->notify();
      } else if (rxValue.rfind("AI;", 0) == 0) {
        Serial.println("Got AI request, responding with nothing.");
        //memmove(messageBuf, "ERR;", 4);
        //pTxCharacteristic->setValue(messageBuf, 4);
        //pTxCharacteristic->notify();
      } else if (rxValue.rfind("Status:", 0) == 0) {
        memmove(messageBuf, "2;", 2);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
      } else if (rxValue.rfind("Vibrate:", 0) == 0) {
        vibration = std::atoi(rxValue.substr(8).c_str());
        Serial.println("Vibe");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate1:", 0) == 0) {
        //vibration_one = std::atoi(rxValue.substr(9).c_str());
        //vibration_collar_zero = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Ear 1");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate2:", 0) == 0) {
        //vibration_two = std::atoi(rxValue.substr(9).c_str());
        //vibration_collar_one = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Ear 2");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate3:", 0) == 0) {
        vibration_collar_zero = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Vibe Collar Zero");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate4:", 0) == 0) {
        vibration_collar_one = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Vibe Collar One");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate5:", 0) == 0) {
        vibration_collar_one = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Shock Collar Zero");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else if (rxValue.rfind("Vibrate6:", 0) == 0) {
        vibration_collar_one = std::atoi(rxValue.substr(9).c_str());
        Serial.println("Shock Collar One");
        memmove(messageBuf, "OK;", 3);
        pTxCharacteristic->setValue(messageBuf, 3);
        pTxCharacteristic->notify();
        UpdateRF();
      } else {
        Serial.println("$Unknown request");        
        memmove(messageBuf, "ERR;", 4);
        pTxCharacteristic->setValue(messageBuf, 4);
        pTxCharacteristic->notify();
      }
    }
};

void setup() {
  Serial.begin(115200);

  SPI.begin();
  // Create the BLE Device
  BLEDevice::init("LVS-204"); // CONFIGURATION: The name doesn't actually matter, The app identifies it by the reported id.

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristics
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_TX_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_RX_UUID,
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );
  pRxCharacteristic->setCallbacks(new MySerialCallbacks());

  // Start the service
  pService->start();
  //Serial.print("Setup: Executing on core ");
  //Serial.println(xPortGetCoreID());
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
  pinMode(pin_rtx, OUTPUT); // Set transmitter pin as output
  pinMode(pin_led, OUTPUT); // Set LED pin as output
  pinMode(buttonPin, INPUT_PULLUP);
  analogWriteFrequency(pin_motor_a, 45000);
  analogWriteFrequency(pin_motor_b, 45000);
  analogWriteResolution(pin_motor_a, 8);
  analogWriteResolution(pin_motor_b, 8);
  Serial.println("Should be setup and ready to go.");
  Heltec.VextON();
  lastUpdate = millis();
}



void loop() {
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(100); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    //collar_keepalive();
    if (digitalRead(buttonPin) == LOW) {
      //transmit_command(collar_chan, COLLAR_LED, 100);
    }
    if (Serial.available() > 0) {
      incomingString = Serial.readString();
      run_command(incomingString);
    }
    if (millis() - lastTrans >= 2500) {
      Serial.println("Still runs");
      lastTrans = millis();
      collar_keepalive();
    }
    if (millis() - lastUpdate >= 5000){

      if (last_vibe_one == vibe_one) {
        vibe_one = 0;
        analogWrite(pin_motor_a, vibe_one);
      }
      if (last_vibe_two == vibe_two) {
        vibe_two = 0;
        analogWrite(pin_motor_b, vibe_two);
      }
      last_vibe_one = vibe_one;
      last_vibe_two = vibe_two;
    }
}
