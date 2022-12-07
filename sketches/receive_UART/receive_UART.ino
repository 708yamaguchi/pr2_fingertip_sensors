// https://shizenkarasuzon.hatenablog.com/entry/2020/06/23/120550
// https://qiita.com/ma2shita/items/37d403fb7a79814d4d4c

#include <M5Stack.h>
#include <SoftwareSerial.h>
#define PACKET_BYTES 44

// SoftwareSerial to use UART via Grove connector
// SoftwareSerial GroveA(21, 22);
SoftwareSerial GroveA(22, 21);

void setup() {
  M5.begin();
  Serial.begin(115200);
  GroveA.begin(57600);
}

// Receive data until \n is detected.
int receive_data (char* received_data) {
  int index = 0;

  while (true) {
    int received_byte_size = GroveA.available();
    // Wait until new data come
    if (received_byte_size == 0) {
      continue;
      // delay(1);
    }
    // Read data
    else {
      char data_ = (char)GroveA.read();
      // Check delimiter
      if (data_ == '\r') {
        continue;
      }
      else if (data_ == '\n') {
        break;
      }
      // Else, store data
      else {
        received_data[index] = data_;
        index++;
      }
    }
  }
  // The byte size of received data
  return index;
}

// packet
// https://docs.google.com/presentation/d/1VxRJWDqeDk_ryu-x1Vhj3_6BDu3gscwvNpngHKwfR4M/edit#slide=id.g1585f6b098c_0_0
// Args
// proximities: 12 length array
// forces: 12 length array
// imu: 3 length array
// Return
// packet_type: 0 or 1
bool parse (const char* packet, uint16_t* proximities, uint16_t* forces, int16_t* imu,
            int* board_select, int* packet_type) {
  // Force and proximity value
  for(int i=0; i<12; i++) {
    uint16_t prox = 0;
    prox += packet[i*3] << 4;
    prox += packet[i*3 + 1] >> 4;
    proximities[i] = prox;
    uint16_t force = 0;
    force += (packet[i*3 + 1] & 0b00001111) << 8;
    force += packet[i*3 + 2];
    forces[i] = force;
  }
  // IMU value
  for(int i=0; i<3; i++) {
    int16_t imu_val = 0;
    imu_val += (int16_t)packet[12*24 + i*2];
    imu[i] = imu_val;
  }
  // board select
  *board_select = packet[42] >> 4;
  // packet type
  *packet_type = packet[42] & 0b00001111;
  // check sum
  uint8_t check_sum = packet[43];
  uint8_t packet_sum = 0;
  for(int i=1; i<PACKET_BYTES-2; i=i+2) {
    packet_sum += packet[i];
  }
  return check_sum == packet_sum;
}

void loop() {
  char received_data[PACKET_BYTES];
  int data_size = receive_data(received_data);

  if (data_size == PACKET_BYTES) {
    uint16_t proximities[12], forces[12];
    int16_t imu[3];
    int board_select, packet_type;
    bool check_sum;
    check_sum = parse(received_data, proximities, forces, imu, &board_select, &packet_type);
    //
    Serial.println("proximities");
    for(int i=0; i<12; i++) {
      Serial.print(proximities[i]);
      Serial.print(", ");
    }
    Serial.println();
    //
    Serial.println("forces");
    for(int i=0; i<12; i++) {
      Serial.print(forces[i]);
      Serial.print(", ");
    }
    Serial.println();
    //
    Serial.println("imu");
    for(int i=0; i<3; i++) {
      Serial.print(imu[i]);
      Serial.print(", ");
    }
    Serial.println();
    // TODO: Get imu value as int
    Serial.println("packet_type: ");
    Serial.print(packet_type);
    Serial.println();
    //
    if (check_sum) {
      Serial.println("check_sum is correct.");
    }
    else {
      Serial.println("check_sum is NOT correct.");
    }
    //
    Serial.println();
    Serial.println();
  }
  else {
    /*
    Serial.print("[ERROR] packet byte size is expected ");
    Serial.print(PACKET_BYTES);
    Serial.print(", but ");
    Serial.print(data_size);
    Serial.println(" bytes come.");
    */
  }
}
