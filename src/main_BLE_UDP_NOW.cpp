#include <Arduino.h>
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <AsyncUDP.h>
#include <Ticker.h>
#include <ESP32_NOW.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

Preferences preferences;
String unitID = "HD 4123";

const char * ssid = "RML-NA";
const char * password = "integrity";

AsyncUDP udp;
uint16_t PLMport = 62104;

uint8_t macTerpilih[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct struct_message {
  char kodeHD[10];
  float TONASE;
} struct_message;
struct_message dataOut;

uint16_t berat2byte(uint8_t high, uint8_t low){
	uint16_t masa;
	masa = (uint16_t)(((low << 8) | high) & 0x3FF);
	return masa;
}

Ticker broadcastUDP;

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

char dataSerial[1000];
char *pSerial = dataSerial;
int lenSerial = 0;

// var Serial1 untuk di kirim
char dataHEX[601];                                  // penampung data dalam tek HEX + 1 untuk Null atau end point
char* pHEX = dataHEX;                                           // pointer ke dataHex
int HexLen = 0;

uint8_t txData[200];      // buff Serial1
uint8_t *pData = txData;  // pointer posisi pengisian data
size_t dataLen = 0;       // untuk menyimpan panjang data
uint32_t timer;           // timer Serial1

// aturan HD785
#define STX 0x02 // 2
#define ETX 0x03 // 3
#define DLE 0x10 // 16

bool readStat = true;
bool stxStat = false;
bool dleStat = false;
bool etxStat = false;
bool bccStat = false;
bool krmData = false;

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  // Constructor of the class using the broadcast address
  ESP_NOW_Broadcast_Peer(uint8_t channel, wifi_interface_t iface, const uint8_t *lmk) : ESP_NOW_Peer(macTerpilih, channel, iface, lmk) {}

  // Destructor of the class
  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  // Function to properly initialize the ESP-NOW and register the broadcast peer
  bool begin() {
    if (!ESP_NOW.begin()) {
      Serial.println("Failed to initialize ESP-NOW");
      return false;
    }
    return true;
  }

  bool add_peer() {
    if (!add()) {
      Serial.printf("Failed to add peer [" MACSTR "]\n", MAC2STR(addr()));
      return false;
    }
    return true;
  }

  // Function to send a message to all devices within the network
  bool send_message(const uint8_t *data, size_t len) {
    if (!send(data, len)) {
      Serial.println("Failed to broadcast message");
      return false;
    }
    return true;
  }
};
ESP_NOW_Broadcast_Peer broadcast_peer(0, WIFI_IF_STA, nullptr);

void infoUDP() {
	udp.broadcastTo((uint8_t *)unitID.c_str(), unitID.length(), PLMport);
	Serial1.write(0x02); Serial1.write(0x4D); Serial1.write(0x32); Serial1.write(0x03); Serial1.write(0x7E);
}

// ubah status koneksi
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

//calback data masuk print ke Serial1
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    if(pCharacteristic->getValue().startsWith("CHANGE: ")){
      unitID = pCharacteristic->getValue().substring(8);

      preferences.begin("unit", false);
      unitID = preferences.putString("kodeUnit", unitID);
      preferences.end();
    }

    uint8_t *data = pCharacteristic->getData();
    size_t len = pCharacteristic->getLength();
    Serial1.write(data, len);
    Serial.write(data, len);
  }
};

float tonase(uint8_t depan, uint8_t belakang){
  float hasil = 0.1f;

  uint16_t gabung = (belakang << 8) | depan;

  hasil = gabung / 10.0f;

  return hasil;
}

// olah data Serial1
void olahData(){
  // olah data jika Serial1 tidak ada data dan timer sudah lebih dari 200ms dan datalen > 0
  if(millis() - timer > 200 && dataLen > 0){
    readStat = false; // matikan dulu pembacaan Serial1
    if(stxStat){  // olah data PLM
      strcpy(dataOut.kodeHD, unitID.c_str());
      // auto(A-C) L(data Dumping), MMS(B-C) M4(data perbucket) P4(final Load) M2(realtime data by request)

      // auto(A-C) header L data dumping
      if(txData[0] == 'L'){ // 76(//0x4C)
        float payload = tonase(txData[9], txData[10]);
        int len = sprintf(pHEX, "\n\nL Automode data DUMPING: [%.1f]\n", payload);    // Header Untuk L data dumping
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX

        //udp
        dataOut.TONASE = payload;
        String dataOutUDP = String(dataOut.kodeHD) + ": " + String(dataOut.TONASE, 1) + " Ton";
	      udp.broadcastTo((uint8_t *)dataOutUDP.c_str(), dataOutUDP.length(), PLMport);
        //now
        broadcast_peer.send_message((uint8_t *)&dataOut, sizeof(dataOut));
      }
      // MMS(B-C) M4 data per Bucket 
      else if(txData[0] == 'M' && txData[1] == '4'){ //77(0x4D), 52(0x34)
        float payload = tonase(txData[3], txData[4]);
        float estimasi = tonase(txData[5], txData[6]);
        int len = sprintf(pHEX, "\n\nM4 mmsMode perBucket[%.1f] estimasi[%.1f]\n", payload, estimasi);    // Header Untuk data M4
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX

        //udp
        dataOut.TONASE = payload;
        String dataOutUDP = String(dataOut.kodeHD) + ": " + String(dataOut.TONASE, 1) + " Ton";
	      udp.broadcastTo((uint8_t *)dataOutUDP.c_str(), dataOutUDP.length(), PLMport);
        //now
        broadcast_peer.send_message((uint8_t *)&dataOut, sizeof(dataOut));
      }
      // MMS(B-C) P4 data Final Load
      else if(txData[0] == 'P' && txData[1] == '4'){ //80(0x50), 52(0x34)
        float payload = tonase(txData[4], txData[4]);
        int len = sprintf(pHEX, "\n\nP4 mmsMode Final Load: [%.1f]\n", payload);    // Header Untuk data P4
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX

        //udp
        dataOut.TONASE = payload;
        String dataOutUDP = String(dataOut.kodeHD) + ": " + String(dataOut.TONASE, 1) + " Ton";
	      udp.broadcastTo((uint8_t *)dataOutUDP.c_str(), dataOutUDP.length(), PLMport);
        //now
        broadcast_peer.send_message((uint8_t *)&dataOut, sizeof(dataOut));
      }
      //MMS(B-C) realTime Data
      else if(txData[0] == 'M' && txData[1] == '2'){ //77(0x4D), 50(0x32)
        float payload = tonase(txData[13], txData[14]);
        int len = sprintf(pHEX, "\n\nM2 mmsMode realTime [%.1f]\n", payload);    // Header Untuk data M2
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX

        //udp
        dataOut.TONASE = payload;
        String dataOutUDP = String(dataOut.kodeHD) + ": " + String(dataOut.TONASE, 1) + " Ton";
	      udp.broadcastTo((uint8_t *)dataOutUDP.c_str(), dataOutUDP.length(), PLMport);
        //now
        broadcast_peer.send_message((uint8_t *)&dataOut, sizeof(dataOut));
      }

      // ubah data ke HEX
      for(int i=0; i < dataLen; i++){
        sprintf(pHEX, "%02X ", txData[i]);                            // ubah 1 byte data ke 3 byte Tek 2 huruf Hex 1 spasi
        pHEX += 3;                                                    // mengeser pointer sesuai 2hex 1 spasi
        HexLen += 3;                                                  // menentukan panjang array HEX
      }
      krmData = true;                    // ubah flag kirim data agar di proses BLE

    } else {  // olah kirim data RAW
      int len = sprintf(pHEX, "\n\nDATA RAW\n");    // Header Untuk data RAW
      pHEX += len;                                  // mengeser pointer sesuai Header
      HexLen += len;                                // menentukan panjang array HEX

      // ubah data ke HEX
      for(int i=0; i < dataLen; i++){
        sprintf(pHEX, "%02X ", txData[i]);                            // ubah 1 byte data ke 3 byte Tek 2 huruf Hex 1 spasi
        pHEX += 3;                                                    // mengeser pointer sesuai 2hex 1 spasi
        HexLen += 3;                                                  // menentukan panjang array HEX
      }
      krmData = true;                    // ubah flag kirim data agar di proses BLE
    }
  }
}

// memproses data Serial1 rx
void processSerial1Data() {
	// 1. poses jika Serial1 buf ada data
	while (Serial1.available() > 0) {
		timer = millis();                         //reset timer Serial1
    uint8_t data = Serial1.read();

    // 2. jika stxStat=true byte berikutnya adalah data PLM
    if(bccStat){
      Serial1.write(0x02); Serial1.write(0x06); Serial1.write(0x31); Serial1.write(0x03); Serial1.write(0x36);
      olahData();
    }
    else if(stxStat){
      // cek apakah byte sama dengan DLE atau ETX dan kondisi flag
      if(data == DLE && !dleStat){
        dleStat = true;
      } else if(data == ETX  && !etxStat && !dleStat){
        etxStat = true;
      } else if(etxStat && !dleStat){
        // data berikutnya adalah bcc
        bccStat = true;
      } else {
        // byte adalah data salin byte ke buff txData
        if(dleStat){ dleStat = false; }               // untuk ubah flag jika byte berikutnya kode dle
        
				*pData = data;                                // isi memori dengan pointer
        pData++;                                      // mengeser posisi pointer
        dataLen++;                                    // menambah panjang data
      }
    } else {
      // cek apakah dataLen 0 jika iya ubah flag stxStat jika bukan masukan data ke buff
      if(data == STX && !stxStat && dataLen == 0){
        stxStat = true;             // ubah stx true untuk byte berikutnya sebagai data
        pData = txData;                               // mengembalikan pointer ke posisi awal
      } else {
        // data Serial1 lain2
        *pData = data;                                // isi memori dengan pointer
        pData++;                                      // mengeser posisi pointer
        dataLen++;                                    // menambah panjang data
      }
    }
  }
}

// memproses data Serial1 rx
void processSerialData() {
	// 1. poses jika Serial1 buf ada data
	while (Serial.available() > 0) {
		timer = millis();                         //reset timer Serial1
    uint8_t data = Serial.read();

    // 2. jika stxStat=true byte berikutnya adalah data PLM
    if(bccStat){
      // Serial1.write(0x02); Serial1.write(0x06); Serial1.write(0x31); Serial1.write(0x03); Serial1.write(0x36);
      olahData();
    }
    else if(stxStat){
      // cek apakah byte sama dengan DLE atau ETX dan kondisi flag
      if(data == DLE && !dleStat){
        dleStat = true;
      } else if(data == ETX  && !etxStat && !dleStat){
        etxStat = true;
      } else if(etxStat && !dleStat){
        // data berikutnya adalah bcc
        bccStat = true;
      } else {
        // byte adalah data salin byte ke buff txData
        if(dleStat){ dleStat = false; }               // untuk ubah flag jika byte berikutnya kode dle
        
				*pData = data;                                // isi memori dengan pointer
        pData++;                                      // mengeser posisi pointer
        dataLen++;                                    // menambah panjang data
      }
    } else {
      // cek apakah dataLen 0 jika iya ubah flag stxStat jika bukan masukan data ke buff
      if(data == STX && !stxStat && dataLen == 0){
        stxStat = true;             // ubah stx true untuk byte berikutnya sebagai data
        pData = txData;                               // mengembalikan pointer ke posisi awal
      } else {
        // data Serial1 lain2
        *pData = data;                                // isi memori dengan pointer
        pData++;                                      // mengeser posisi pointer
        dataLen++;                                    // menambah panjang data
      }
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, 19, 21);

  preferences.begin("unit", false);
  unitID = preferences.getString("kodeUnit", "Houler");
  preferences.end();

  // Create the BLE Device
  BLEDevice::init("TEST RS232");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	WiFi.setAutoReconnect(true);

  if(udp.listen(PLMport)) {
    udp.onPacket([](AsyncUDPPacket packet) {
			if(!packet.isMulticast() && !packet.isBroadcast()){
				String dataIn = String((char *)packet.data()).substring(0, packet.length());
				if(dataIn.startsWith("CHANGE: ")){
					unitID = dataIn.substring(8);

          preferences.begin("unit", false);
          unitID = preferences.putString("kodeUnit", unitID);
          preferences.end();
				}
			}
		});
	}

  broadcast_peer.begin(); broadcast_peer.add_peer();

  broadcastUDP.attach(1, infoUDP);
}

void loop() {

  olahData();
  // cek apakah ada data Serial1
  if(readStat){ processSerial1Data(); processSerialData(); }

  // cek flag kirim data
  if(krmData){
    // jika terhubung BLE kirim data jika tidak buang data
    if(deviceConnected){
      pTxCharacteristic->setValue((uint8_t*)dataHEX, HexLen); // panggil data untuk di kirim
      pTxCharacteristic->notify();                                     // kirim data
    }

    // untuk akhiri proses kirim data dan kembali baca Serial1
    dataLen = 0;
    pData = txData;
    HexLen = 0;
    pHEX = dataHEX;
    readStat = true;
    krmData = false;
    stxStat = false;
    dleStat = false;
    etxStat = false;
  }
  delay(10);
}

// L = 4C, P4 = 50 34, M4 = 4D 34, M2 = 4D 32
// req 02 4D 32 03 7C
// stp 02 51 32 03 60
