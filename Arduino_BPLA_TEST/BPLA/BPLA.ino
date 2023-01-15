#include <SPI.h>
#include "printf.h"
#include "RF24.h"
#include <Servo.h>

#define LIFT 4
#define MOTOR 5
#define RUDDER 6
#define ELERON1 9
#define ELERON2 10

Servo eleron1,eleron2,lift,rudder,motor;

RF24 radio(7, 8);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 0;
byte reload_counter = 0; // increases each time radio fails to connect 

byte base_channel = 0x60;
byte best_channel = 0x60;

byte system_default_command = 0; // number corresponding to system command "do nothing"
byte system_change_channel_command = 10; // number corresponding to system command "change channel"
byte channel_default_value = 255; // default channel (alias to base channel)
byte check_default_value = 232; // default value for verifying non-empty transmition

byte system_cell_number = 0; // number in array of cell for system commands
byte channel_cell_number = 1; // number in array of cell for channel
byte sent_counter_cell_number = 2; // number in array of cell for sent counter that is increased each time communication happens
byte check_cell_number = 3; // number in array of cell for check value

int motor_default_value = 850; // default motor power value
int rudder_default_value = 90; // default rudder angle value
int lift_default_value = 90; // default lift angle value
int eleron_default_value = 90; // default eleron angle value

byte motor_cell_number = 4; // number in array of cell for motor power
byte rudder_cell_number = 5; // number in array of cell for rudder angle
byte lift_cell_number = 6; // number in array of cell for lift angle
byte eleron_cell_number = 7; // number in array of cell for eleron angle

int message_counter = 1; // counts number of successfull communications

// array for sending payload
int send_payload[8]={system_default_command, channel_default_value, message_counter, check_default_value,
                     0,                      0,                     0,               0};

// array for receiving payload
int recieve_payload[8]={system_default_command, channel_default_value, message_counter,    check_default_value,
                        motor_default_value,    rudder_default_value,  lift_default_value, eleron_default_value};
                 
unsigned long recieving_timer;

void start_radio(){
  unsigned long t1 = millis();
  while (millis()-t1 < 60000) {
    if (!radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
    }
    break;
  }

  radio.setChannel(best_channel);

  pipeIt();
  
  radio.startListening(); // put radio in RX mode

  radio.writeAckPayload(1, &send_payload, sizeof(send_payload));

  if(reload_counter % 14 == 13) { // fall to base channel if cannot connect
    radio.setChannel(base_channel);
    ++reload_counter;
  }
  ++reload_counter;
}

void setup_motor(){
  motor.writeMicroseconds(2300);
  delay(2000);
  motor.writeMicroseconds(850);
  delay(5000);
}

void setup_servos(){
  eleron1.attach(ELERON1);
  eleron2.attach(ELERON2);
  lift.attach(LIFT);
  rudder.attach(RUDDER);
  motor.attach(MOTOR);
  setup_motor();
}

void setup() {
  delay(10000);
  Serial.begin(57600);
//  portOne.begin(9600);
  
  setup_servos();
  start_radio();
}

void loop() {
  // This device is a RX node

  uint8_t pipe;
  if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize(); // get the size of the payload
    int len = bytes/sizeof(int);
    int cpayload[len];
    for(int i = 0; i < len; i++) {
      cpayload[i] = recieve_payload[i];
    }
    radio.read(&recieve_payload, bytes);            // fetch payload from FIFO

    if(recieve_payload[check_cell_number] != check_default_value) {
      for(int i = 0; i < len; i++) {
        recieve_payload[i] = cpayload[i];
      }
    }
    
    if(recieve_payload[system_cell_number] == system_change_channel_command) {
      best_channel = recieve_payload[channel_cell_number];
      recieve_payload[system_cell_number] = system_default_command;
      recieve_payload[channel_cell_number] = channel_default_value;
      radio.setChannel(best_channel);
    }

    // setup electronics
    eleron1.write(recieve_payload[eleron_cell_number]);
    eleron2.write(recieve_payload[eleron_cell_number]);
    lift.write(recieve_payload[lift_cell_number]);
    rudder.write(recieve_payload[rudder_cell_number]);
    motor.writeMicroseconds(recieve_payload[motor_cell_number]);

    message_counter = recieve_payload[sent_counter_cell_number];
    message_counter++;

    send_payload[sent_counter_cell_number] = message_counter;

    // send data
    radio.writeAckPayload(1, &send_payload, sizeof(send_payload));
    
    recieving_timer = millis();
  } else {
    unsigned long curent_t = millis();
    if(curent_t - recieving_timer > 5000) {
      recieving_timer = curent_t;
      start_radio();
    }
  }
}

void pipeIt() {
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);
}
