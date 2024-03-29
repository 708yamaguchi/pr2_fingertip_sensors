// https://shizenkarasuzon.hatenablog.com/entry/2020/06/23/120550
// https://qiita.com/ma2shita/items/37d403fb7a79814d4d4c

// By default, define SOFTWARE_SERIAL. You can override this.
#if (!defined I2C_MASTER) && (!defined HARDWARE_SERIAL) && (!defined SOFTWARE_SERIAL)
  #define I2C_MASTER
#endif

// By default, PFS slave address is 0x01. You can override this.
// 7bit slave address
#if (!defined PFS_ADDRESSES)
  #define PFS_ADDRESSES {0x01}
#endif
const uint8_t pfs_addresses[] = PFS_ADDRESSES;

#ifdef I2C_MASTER
  #include <Wire.h>
#endif

#ifdef SOFTWARE_SERIAL
  #include <SoftwareSerial.h>
  // SoftwareSerial to use UART via Grove connector
  // The following pin assignments (22, 21) allow I2C
  // to be used without removing and inserting connectors.
  SoftwareSerial GroveA(22, 21);
#endif

#define PACKET_BYTES 44
#define NUM_SENSORS 24

struct pfs_packet {
  int16_t proximities[12];
  int16_t forces[12];
  int16_t imu[3];
  int board_select;
  int packet_type;
  bool check_sum;
};

struct pfs_sensors {
  int16_t proximities[NUM_SENSORS];
  int16_t forces[NUM_SENSORS];
  int16_t acc[3];
  int16_t gyro[3];
};

void begin_pfs() {
  #ifdef SOFTWARE_SERIAL
    GroveA.begin(57600);
    Serial.println("Start SOFTWARE_SERIAL mode");
  #endif
  #ifdef HARDWARE_SERIAL
    // Connect RX and TX to M5Stack's pin2 and pin5
    Serial1.begin(57600, SERIAL_8N1,2, 5);
    Serial.println("Start HARDWARE_SERIAL mode");
  #endif
  #ifdef I2C_MASTER
    Wire.begin();
    Serial.println("Start I2C_MASTER mode");
  #endif
}

void end_pfs_serial() {
  #ifdef SOFTWARE_SERIAL
    GroveA.end();
  #endif
  #ifdef HARDWARE_SERIAL
    // Connect RX and TX to M5Stack's pin2 and pin5
    Serial1.end();
  #endif
}


// Receive data until \n is detected.
template <typename SerialClass>
int receive_data_with_serial (SerialClass* ss, char* received_data) {
  int index = 0;

  unsigned long start_time = millis();
  int timeout = 500; // [ms]
  while (millis() - start_time < timeout) {
    int received_byte_size = ss->available();
    // Wait until new data come
    if (received_byte_size == 0) {
      continue;
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

int receive_data_with_i2c (char* received_data, uint8_t pfs_address) {
  // Send command to PFS
  Wire.beginTransmission(pfs_address);
  uint8_t req = 0x12;
  Wire.write(req);
  byte error = Wire.endTransmission();
  if (error != 0) {
    Serial.println("Error is detected in Wire.endTransmission()");
  }
  // Receive data
  Wire.requestFrom((int)pfs_address, PACKET_BYTES);
  int received_bytes = Wire.available(); // Expect to get PACKET_BYTES bytes
  for(int i=0; i<received_bytes; i++){
    received_data[i] = Wire.read();
  }
  // The byte size of received data
  return received_bytes;
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
  // if (packet_obj.check_sum) {
  //   Serial.println("check_sum is correct.");
  //   Serial.print("Received check sum is ");
  //   Serial.println(check_sum);
  //   Serial.print("Calculated check sum is ");
  //   Serial.println(packet_sum);
  // }
  // else {
  //   Serial.println("check_sum is NOT correct.");
  // }

  return packet_obj;
}

// data_array is proximity or force sensor data array (NUM_SENSORS length) 
void order_data(const int16_t* data_array, int16_t* data_ordered) {
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

// Before calling this function, you need to call setup_pfs_serial()
void read_sensors(struct pfs_sensors* sensors, uint8_t pfs_address = 0x01) {
  int16_t forces[NUM_SENSORS], proximities[NUM_SENSORS];
  uint8_t packet_exist[2] = {0, 0}; // if packet type X comes, packet_exist[X] = 1;
  unsigned long start_time = millis();
  int timeout = 3000; // [ms]
  // Wait for two type packets to come and append them
  // If timeout, do not change sensors object
  while (!(packet_exist[0]==1 && packet_exist[1]==1) &&
         (millis() - start_time < timeout)) {
    // Receive packet
    char received_data[PACKET_BYTES];
    int data_size;
    #ifdef SOFTWARE_SERIAL
      data_size = receive_data_with_serial<SoftwareSerial>(&GroveA, received_data);
    #endif
    #ifdef HARDWARE_SERIAL
      data_size = receive_data_with_serial<HardwareSerial>(&Serial1, received_data);
    #endif
    #ifdef I2C_MASTER
      data_size = receive_data_with_i2c(received_data, pfs_address);
    #endif
    if (data_size != PACKET_BYTES) {
      // Serial.print("[ERROR] packet byte size is expected ");
      // Serial.print(PACKET_BYTES);
      // Serial.print(", but ");
      // Serial.print(data_size);
      // Serial.println(" bytes come.");
      delay(10);
      continue;
    }
    // Parse packet
    struct pfs_packet packet;
    packet = parse(received_data);
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
  for(int i=0; i<NUM_SENSORS; i++) {
    Serial.print(sensors->proximities[i]);
    Serial.print(", ");
  }
  Serial.println();
  //
  Serial.println("forces");
  for(int i=0; i<NUM_SENSORS; i++) {
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

void receive_pfs() {
  struct pfs_sensors sensors;
  #if (defined SOFTWARE_SERIAL) || (defined HARDWARE_SERIAL)
    read_sensors(&sensors);
    print_sensors(&sensors); // For debug
  #endif
  #if (defined I2C_MASTER)
    for(int i=0; i<sizeof(pfs_addresses)/sizeof(uint8_t); i++) {
      read_sensors(&sensors, pfs_addresses[i]);
      Serial.print("\nPFS Slave Address: 0x");
      Serial.println(pfs_addresses[i], HEX);
      print_sensors(&sensors); // For debug
    }
  #endif
}
