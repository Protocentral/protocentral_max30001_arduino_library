//////////////////////////////////////////////////////////////////////////////////////////
//
//    Example for MAX30001 + QT Py ES32-C3 + OpenView BLE
//
//    This example plots the ECG through serial UART on openview processing GUI.
//    GUI URL: https://github.com/Protocentral/protocentral_openview.git
//
//    Arduino connections:
//
//  |MAX30001 pin label| Pin Function         |Arduino Connection|
//  |----------------- |:--------------------:|-----------------:|
//  | MISO             | Slave Out            |  D12             |
//  | MOSI             | Slave In             |  D11             |
//  | SCLK             | Serial Clock         |  D13             |
//  | CS               | Chip Select          |  D7              |
//  | VCC              | Digital VDD          |  +5V             |
//  | GND              | Digital Gnd          |  Gnd             |
//  | FCLK             | 32K CLOCK            |  -               |
//  | INT1             | Interrupt1           |  02              |
//  | INT2             | Interrupt2           |  -               |
//
//    This software is licensed under the MIT License(http://opensource.org/licenses/MIT).
//
//   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
//   NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
//   IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
//   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//   For information on how to use, visit https://github.com/Protocentral/protocentral_max30001
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "protocentral_max30001.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define Heartrate_SERVICE_UUID (uint16_t(0x180D))
#define Heartrate_CHARACTERISTIC_UUID (uint16_t(0x2A37))

#define DATASTREAM_SERVICE_UUID (uint16_t(0x1122))
#define DATASTREAM_CHARACTERISTIC_UUID (uint16_t(0x1424))


#define MAX30001_CS_PIN 20
#define MAX30001_DELAY_SAMPLES 8 // Time between consecutive samples

#define CES_CMDIF_PKT_START_1 0x0A
#define CES_CMDIF_PKT_START_2 0xFA
#define CES_CMDIF_TYPE_DATA 0x02
#define CES_CMDIF_PKT_STOP 0x0B
#define DATA_LEN 0x0C
#define ZERO 0

volatile char DataPacket[DATA_LEN];
const char DataPacketFooter[2] = {ZERO, CES_CMDIF_PKT_STOP};
const char DataPacketHeader[5] = {CES_CMDIF_PKT_START_1, CES_CMDIF_PKT_START_2, DATA_LEN, ZERO, CES_CMDIF_TYPE_DATA};

uint8_t data_len = 0x0C;
bool BioZSkipSample = false;

MAX30001 max30001(MAX30001_CS_PIN);

signed long ecg_data;
signed long bioz_data;

uint8_t ecg_data_buff[40];
uint8_t resp_data_buff[40];
uint8_t ppg_data_buff[20];
int sp02;
uint32_t ecg_stream_cnt = 0;
uint32_t resp_stream_cnt = 0;
uint16_t ppg_stream_cnt = 0;
float temp;


bool deviceConnected = false;
bool oldDeviceConnected = false;
bool temp_data_ready = false;
bool spo2_calc_done = false;
bool ecg_buf_ready = false;
bool resp_buf_ready = false;
bool ppg_buf_ready = false;
String strValue = "";

BLEServer *pServer = NULL;

BLECharacteristic *Heartrate_Characteristic = NULL;
BLECharacteristic *datastream_Characteristic = NULL;

void sendData(signed long ecg_sample, signed long bioz_sample, bool _bioZSkipSample)
{

  DataPacket[0] = ecg_sample;
  DataPacket[1] = ecg_sample >> 8;
  DataPacket[2] = ecg_sample >> 16;
  DataPacket[3] = ecg_sample >> 24;

  DataPacket[4] = bioz_sample;
  DataPacket[5] = bioz_sample >> 8;
  DataPacket[6] = bioz_sample >> 16;
  DataPacket[7] = bioz_sample >> 24;

  if (_bioZSkipSample == false)
  {
    DataPacket[8] = 0x00;
  }
  else
  {
    DataPacket[8] = 0xFF;
  }

  DataPacket[9] = 0x00; // max30001.heartRate >> 8;
  DataPacket[10] = 0x00;
  DataPacket[11] = 0x00;

  // Send packet header (in ProtoCentral OpenView format)
  for (int i = 0; i < 5; i++)
  {
    Serial.write(DataPacketHeader[i]);
  }

  // Send the data payload
  for (int i = 0; i < DATA_LEN; i++) // transmit the data
  {
    Serial.write(DataPacket[i]);
  }

  // Send packet footer (in ProtoCentral OpenView format)
  for (int i = 0; i < 2; i++)
  {
    Serial.write(DataPacketFooter[i]);
  }
}

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
    Serial.println("connected");
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    Serial.println("disconnected");
  }
};

class MyCallbackHandler : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *datastream_Characteristic)
  {
    std::string value = datastream_Characteristic->getValue();
    // int len = value.length();
    strValue = "0";

    if (value.length() > 0)
    {
      Serial.print("New value: ");

      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(String(value[i]));
        strValue += value[i];
      }

      Serial.println();
    }
  }
};

void HealthyPi5_BLE_Init()
{
  BLEDevice::init("Healthypi 5");      // Create the BLE Device
  pServer = BLEDevice::createServer(); // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *HeartrateService = pServer->createService(Heartrate_SERVICE_UUID); // Create the BLE Service
  BLEService *datastreamService = pServer->createService(DATASTREAM_SERVICE_UUID);

  Heartrate_Characteristic = HeartrateService->createCharacteristic(
      Heartrate_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  datastream_Characteristic = datastreamService->createCharacteristic(
      DATASTREAM_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

  Heartrate_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->addDescriptor(new BLE2902());
  datastream_Characteristic->setCallbacks(new MyCallbackHandler());

  // Start the service
  HeartrateService->start();
  datastreamService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(Heartrate_SERVICE_UUID);
  
  pAdvertising->addServiceUUID(DATASTREAM_SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void setup()
{
  Serial.begin(57600); // Serial begin
  //SPI.begin();
   

  SPI.begin(10,8,7,20);

  //SPI.

  bool ret = max30001.max30001ReadInfo();
  if (ret)
  {
    Serial.println("MAX 30001 read ID Success");
  }
  else
  {
    while (!ret)
    {
      // stay here untill the issue is fixed.
      ret = max30001.max30001ReadInfo();
      Serial.println("Failed to read ID, please make sure all the pins are connected");
      delay(5000);
    }
  }

  Serial.println("Initialising the chip ...");
  max30001.BeginECGBioZ(); // initialize MAX30001
                           // max30001.Begin();
}

void handle_ble_stack()
{

  if (ecg_buf_ready)
  {
    ecg_buf_ready = false;
    datastream_Characteristic->setValue(ecg_data_buff, 38);
    datastream_Characteristic->notify();
  }

  

  if (1) //(spo2_calc_done)
  {
    

    uint8_t hr_att_ble[5];
    hr_att_ble[0] = 0x00;
    hr_att_ble[1] = (uint8_t)sp02;
    hr_att_ble[2] = (uint8_t)(sp02 >> 8);
    hr_att_ble[3] = 0;
    hr_att_ble[4] = 0;

    Heartrate_Characteristic->setValue(hr_att_ble, 5);
    Heartrate_Characteristic->notify();

    
  }

  

  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
  }

  if (!deviceConnected && oldDeviceConnected)
  {
    delay(500);                  // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
}


void loop()
{
  ecg_data = max30001.getECGSamples();
  // max30001.getHRandRR();
  if (BioZSkipSample == false)
  {
    bioz_data = max30001.getBioZSamples();
    sendData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = true;
  }
  else
  {
    bioz_data = 0x00;
    sendData(ecg_data, bioz_data, BioZSkipSample);
    BioZSkipSample = false;
  }
  delay(8);
}