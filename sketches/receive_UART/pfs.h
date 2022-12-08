// https://shizenkarasuzon.hatenablog.com/entry/2020/06/23/120550
// https://qiita.com/ma2shita/items/37d403fb7a79814d4d4c

#include <SoftwareSerial.h>

#define PACKET_BYTES 44

struct pfs_packet {
  uint16_t proximities[12];
  uint16_t forces[12];
  int16_t imu[3];
  int board_select;
  int packet_type;
  bool check_sum;
};

struct pfs_sensors {
  uint16_t proximities[24];
  uint16_t forces[24];
  int16_t acc[3];
  int16_t gyro[3];
};

// Receive data until \n is detected.
int receive_data (SoftwareSerial* ss, char* received_data) {
  int index = 0;

  while (true) {
    int received_byte_size = ss->available();
    // Wait until new data come
    if (received_byte_size == 0) {
      continue;
      // delay(1);
    }
    // Read data
    else {
      char data_ = (char)ss->read();
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
// packet: packet array received from serial(UART)
// Return
// packet_obj: struct pfs_packet object, which contains parsed packet infomation.
struct pfs_packet parse (const char* packet) {
  struct pfs_packet packet_obj;
  // Force and proximity value
  for(int i=0; i<12; i++) {
    uint16_t prox = 0;
    prox += packet[i*3] << 4;
    prox += packet[i*3 + 1] >> 4;
    packet_obj.proximities[i] = prox;
    uint16_t force = 0;
    force += (packet[i*3 + 1] & 0b00001111) << 8;
    force += packet[i*3 + 2];
    packet_obj.forces[i] = force;
  }
  // IMU value
  for(int i=0; i<3; i++) {
    int16_t imu_val = 0;
    imu_val += packet[36 + i*2] << 8;
    imu_val += packet[36 + i*2 + 1];
    packet_obj.imu[i] = imu_val;
  }
  // board select
  packet_obj.board_select = packet[42] >> 4;
  // packet type
  packet_obj.packet_type = packet[42] & 0b00001111;
  // check sum
  uint8_t check_sum = packet[43];
  uint8_t packet_sum = 0;
  for(int i=1; i<PACKET_BYTES-2; i=i+2) {
    packet_sum += packet[i];
  }
  packet_obj.check_sum = (check_sum == packet_sum);
  if (!check_sum) {
    // Serial.println("check_sum is NOT correct.");
  }
  return packet_obj;
}

// data_array is proximity or force sensor data array (24 length) 
void order_data(const uint16_t* data_array, uint16_t* data_ordered) {
  for(int i=0; i<8; i++) {
    data_ordered[i] = data_array[i];
  }
  for(int i=0; i<4; i++) {
    data_ordered[8+i] = data_array[16+i];
    data_ordered[12+i] = data_array[20+i];
    data_ordered[16+i] = data_array[8+i];
    data_ordered[20+i] = data_array[12+i];
  }
}

void read_sensors (SoftwareSerial* ss, struct pfs_sensors* sensors) {
  uint16_t forces[24], proximities[24];
  uint8_t packet_exist[2] = {0, 0}; // if packet type X comes, packet_exist[X] = 1;
  // Wait for two type packets to come and append them
  while (!(packet_exist[0]==1 && packet_exist[1]==1)) {
    // Receive packet
    char received_data[PACKET_BYTES];
    int data_size = receive_data(ss, received_data);
    // Parse packet
    struct pfs_packet packet;
    if (data_size == PACKET_BYTES) {
      packet = parse(received_data);
    }
    else {
      // Serial.print("[ERROR] packet byte size is expected ");
      // Serial.print(PACKET_BYTES);
      // Serial.print(", but ");
      // Serial.print(data_size);
      // Serial.println(" bytes come.");
    }
    // Store packet
    if (packet.packet_type == 0) {
      for(int i=0; i<12; i++) {
        proximities[i] = packet.proximities[i];
        forces[i] = packet.forces[i];
      }
      for(int i=0; i<3; i++) {
        sensors->gyro[i] = packet.imu[i];
      }
    }
    else if (packet.packet_type == 1) {
      for(int i=0; i<12; i++) {
        proximities[i+12] = packet.proximities[i];
        forces[i+12] = packet.forces[i];
      }
      for(int i=0; i<3; i++) {
        sensors->acc[i] = packet.imu[i];
      }
    }
    packet_exist[packet.packet_type] = 1;
  }
  // order proximity and force data
  order_data(proximities, sensors->proximities);
  order_data(forces, sensors->forces);
}

void print_sensors (const struct pfs_sensors* sensors) {
  Serial.println("proximities");
  for(int i=0; i<24; i++) {
    Serial.print(sensors->proximities[i]);
    Serial.print(", ");
  }
  Serial.println();
  //
  Serial.println("forces");
  for(int i=0; i<24; i++) {
    Serial.print(sensors->forces[i]);
    Serial.print(", ");
  }
  Serial.println();
  //
  Serial.println("acc");
  for(int i=0; i<3; i++) {
    Serial.print(sensors->acc[i]);
    Serial.print(", ");
  }
  Serial.println();
  //
  Serial.println("gyro");
  for(int i=0; i<3; i++) {
    Serial.print(sensors->gyro[i]);
    Serial.print(", ");
  }
  Serial.println();
  Serial.println();
}
