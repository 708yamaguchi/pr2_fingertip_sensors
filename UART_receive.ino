// https://shizenkarasuzon.hatenablog.com/entry/2020/06/23/120550
// https://qiita.com/ma2shita/items/37d403fb7a79814d4d4c

#include <M5Stack.h>
#include <SoftwareSerial.h>
#define MESSAGE_SIZE 1000

// SoftwareSerial to use UART via Grove connector
// SoftwareSerial GroveA(21, 22);
SoftwareSerial GroveA(22, 21);

void setup() {
  M5.begin();
  Serial.begin(115200);
  GroveA.begin(57600);
}

void loop() {
  char received_data[MESSAGE_SIZE];
  int received_byte_size = GroveA.available();

  if (received_byte_size == 0) {
    // Serial.println("No data is received.");
    delay(10);
  }
  else {
    for (int i=0;i<received_byte_size; i++) {
      received_data[i] = (char)GroveA.read();
    }
    received_data[received_byte_size] = 0; // 終端文字
    Serial.print(received_data);
  }
}
