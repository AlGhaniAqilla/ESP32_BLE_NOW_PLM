#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

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
    if(pCharacteristic->getValue().startsWith("UNIT: ")){
      unitID = pCharacteristic->getValue().substring(6);

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
      // auto(A-C) L(data Dumping), MMS(B-C) M4(data perbucket) P4(final Load) M2(realtime data by request)

      // auto(A-C) header L data dumping
      if(txData[0] == 'L'){ // 76(//0x4C)
        float payload = tonase(txData[9], txData[10]);
        int len = sprintf(pHEX, "\n\nL Automode data DUMPING: [%.1f]\n", payload);    // Header Untuk L data dumping
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX
      }
      // MMS(B-C) M4 data per Bucket 
      else if(txData[0] == 'M' && txData[1] == '4'){ //77(0x4D), 52(0x34)
        float payload = tonase(txData[3], txData[4]);
        float estimasi = tonase(txData[5], txData[6]);
        int len = sprintf(pHEX, "\n\nM4 mmsMode perBucket[%.1f] estimasi[%.1f]\n", payload, estimasi);    // Header Untuk data M4
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX
      }
      // MMS(B-C) P4 data Final Load
      else if(txData[0] == 'P' && txData[1] == '4'){ //80(0x50), 52(0x34)
        float payload = tonase(txData[4], txData[4]);
        int len = sprintf(pHEX, "\n\nP4 mmsMode Final Load: [%.1f]\n", payload);    // Header Untuk data P4
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX
      }
      //MMS(B-C) realTime Data
      else if(txData[0] == 'M' && txData[1] == '2'){ //77(0x4D), 50(0x32)
        float payload = tonase(txData[13], txData[14]);
        int len = sprintf(pHEX, "\n\nM2 mmsMode realTime [%.1f]\n", payload);    // Header Untuk data M2
        pHEX += len;                       // mengeser pointer sesuai Header
        HexLen += len;                     // menentukan panjang array HEX
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
  Serial1.println("Waiting a client connection to notify...");
}

void loop() {

  olahData();
  // cek apakah ada data Serial1
  if(readStat){ processSerial1Data(); processSerialData(); }

  // cek flag kirim data
  if(krmData){
    // jika terhubung kirim data jika tidak buang data
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
