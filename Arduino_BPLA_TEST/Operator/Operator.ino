#include <SPI.h>
#include <RF24.h>

// define sticks and buttons
#define MOTOR A0
#define RUDDER A1
#define LIFT A2
#define ELERON A3

#define MOTOR_BUTTON 4
#define ELERON_BUTTON 5

// define radio
RF24 radio(7, 8);

uint8_t address[][6] = {"1Node", "2Node"}; // radio addresses
bool radioNumber = 1; // sets radio address to "2Node"
byte reload_counter = 0; // increases each time radio fails to connect 
// if problem persists, first tries to change channel to better one, 
// then falls down to fixed base channel

bool set_channel = true; // indicates whether to try 
// to switch to better channel and send info about better channel to other radio

const uint8_t num_channels = 128; // total number of channels

byte base_channel = 0x60;
byte best_channel = 0x60;
byte future_best_channel = 0x60;

int stick_range[4][2], stick_center[4]; // needed for stick calibration
float stick_trim[4]; // stores trim values for sticks

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

int message_counter = 0; // counts number of successfull communications

// array for sending payload
int send_payload[8]={system_default_command, channel_default_value, message_counter,    check_default_value,
                     motor_default_value,    rudder_default_value,  lift_default_value, eleron_default_value};

// array for receiving payload
int recieve_payload[8]={system_default_command, channel_default_value, message_counter, check_default_value,
                        0,                      0,                     0,               0};

unsigned long sending_timer; // timer to cause radio reload in case of long lasting disconnection

unsigned long roundtrip_time; // time it take to send and receive data

unsigned long printing_timer; // timer to print data in serial

int trim_state = 0; // which control to trim now

// variables to switch on/off engine
bool engine_on = false;
bool engine_was_on = false;

void start_radio(){ // setups radio
  unsigned long t1 = millis();
  while (millis()-t1 < 60000) {
    if (!radio.begin()) {
      Serial.println(F("radio hardware is not responding!!"));
    }
    break;
  }

  radio.setChannel(best_channel);

  pipeIt();

  radio.stopListening();  // put radio in TX mode

  if(reload_counter % 8 == 0) { // find new best channel if old is polluted
    set_channel = true;
    findBestChannel();
    ++reload_counter;
  }

  if(reload_counter % 14 == 13) { // fall to base channel if cannot connect
    set_channel = true;
    findBestChannel();
    radio.setChannel(base_channel);
    ++reload_counter;
  }
  ++reload_counter;
}

void setup_sticks(){ // stick calibration
  for (int i=0; i<4; i++){
    stick_range[i][0] = 1024;
    stick_range[i][1] = 0;
    stick_trim[4] = 0;
  }
  
  while(digitalRead(ELERON_BUTTON)){ // press button on eleron stick to exit calibration mode
    int el = analogRead(ELERON);
    int lf = analogRead(LIFT);
    int mo = analogRead(MOTOR);
    int ru = analogRead(RUDDER);
    stick_range[0][0] = el < stick_range[0][0] ? el : stick_range[0][0];
    stick_range[0][1] = el > stick_range[0][1] ? el : stick_range[0][1];
    stick_range[1][0] = lf < stick_range[0][0] ? lf : stick_range[0][0];
    stick_range[1][1] = lf > stick_range[0][1] ? lf : stick_range[0][1];
    stick_range[2][0] = mo < stick_range[0][0] ? mo : stick_range[0][0];
    stick_range[2][1] = mo > stick_range[0][1] ? mo : stick_range[0][1];
    stick_range[3][0] = ru < stick_range[0][0] ? ru : stick_range[0][0];
    stick_range[3][1] = ru > stick_range[0][1] ? ru : stick_range[0][1];
  }

  delay(1000); // put all sticks to center position

  int el = analogRead(ELERON);
  int lf = analogRead(LIFT);
  int mo = analogRead(MOTOR);
  int ru = analogRead(RUDDER);

  stick_center[0] = el;
  stick_center[1] = lf;
  stick_center[2] = mo;
  stick_center[3] = ru;

  for(int i = 0; i < 4; i++){ // crop sticks range to make it simmetrical for center
    int range = (stick_range[i][0] + stick_range[i][1])/2 > stick_center[i] ? stick_center[i] - stick_range[i][0] : stick_range[i][1] - stick_center[i];
    stick_range[i][0] = stick_center[i] - range;
    stick_range[i][1] = stick_center[i] + range;
  }
}

void setup() {
  // general setup
  pinMode(MOTOR_BUTTON, INPUT_PULLUP);
  pinMode(ELERON_BUTTON, INPUT_PULLUP);
  
  Serial.begin(57600);

  // calibrate sticks
  setup_sticks();

  // setup nrf
  start_radio();

  // setup timers
  printing_timer = millis();
  sending_timer = millis();
  
  roundtrip_time = 0;
}

void loop() {
  // engine force shutdown logic
  bool is_on = analogRead(MOTOR) > 2 * (stick_range[2][0]+stick_range[2][1])/3;
  if (!is_on && engine_was_on) {
    engine_on = !engine_on;
    engine_was_on = false;
  }

  if (is_on) {
    engine_was_on = true;
  }

  // printing data
  if (millis() - printing_timer > 500) {
    // print trimming data
    switch (trim_state) {
      case 0: {
        Serial.println("ELERON: " + String(stick_trim[trim_state]));
        break;
      }
      case 1: {
        Serial.println("LIFT: " + String(stick_trim[trim_state]));
        break;
      }
      case 3: {
        Serial.println("RUDDER: " + String(stick_trim[trim_state]));
        break;
      }
    }
    
    // print engine data
    Serial.println("ENGINE: " + String(engine_on));

    // print received data
    Serial.println("TIME: " + String(roundtrip_time));
    Serial.println("COUNT: " + String(message_counter));
    
    printing_timer = millis();
  }

  // trimming logic
  if (!digitalRead(MOTOR_BUTTON) && !digitalRead(ELERON_BUTTON)) {
    trim_state = trim_state == 1 ? 3 : (trim_state+1) % 4;
  } else if (!digitalRead(MOTOR_BUTTON)) {
    stick_trim[trim_state] = max(stick_trim[trim_state]-0.1, -180);
  } else if (!digitalRead(ELERON_BUTTON)) {
    stick_trim[trim_state] = min(stick_trim[trim_state]+0.1, 180);
  }

  // nrf job
  // This device is a TX node

  // read sticks
  setup_payload();

  // set payload system command data
  if(set_channel) {
    send_payload[system_cell_number] = system_change_channel_command;
    send_payload[channel_cell_number] = future_best_channel;
  } else {
    send_payload[system_cell_number] = system_default_command;
    send_payload[channel_cell_number] = channel_default_value;
  }
  
  unsigned long start_timer = micros();                    // start the timer
  bool report = radio.write(&send_payload, sizeof(send_payload));      // transmit & save the report

  if (report) { // read response
    uint8_t pipe;
    if (radio.available(&pipe)) {             // is there a payload? get the pipe number that recieved it
      sending_timer = millis();
      
      uint8_t bytes = radio.getPayloadSize(); // get the size of the payload
      int len = bytes/sizeof(int);
      int cpayload[len]; // create temprorary array to store previously recieved data
      for(int i = 0; i < len; i++) {
        cpayload[i] = recieve_payload[i];
      }
      radio.read(&recieve_payload, bytes);            // fetch payload from FIFO
       
      if(recieve_payload[check_cell_number] != check_default_value) { // check if data valid
        //revert to previous state
        for(int i = 0; i < len; i++) {
          recieve_payload[i] = cpayload[i];
        }
      } else {
        // update channel info
        if(set_channel) {
          set_channel = !set_channel;
          best_channel = future_best_channel;
          radio.setChannel(best_channel);
        }
        message_counter = recieve_payload[sent_counter_cell_number];
        message_counter++;

        unsigned long end_timer = micros();                      // end the timer
        roundtrip_time = start_timer - end_timer;
      }
    }
  } else {
//      Serial.println(F("Transmission failed or timed out")); // payload was not delivered
    unsigned long curent_t = millis();
    if(curent_t - sending_timer > 5000) {
      sending_timer = curent_t;
      start_radio();
    }
  }
}

void setup_payload(){
  // set eleron and lift
  send_payload[eleron_cell_number] = constrain(int(stick_trim[0]) + 180 - constrain(map(analogRead(ELERON), stick_range[0][0], stick_range[0][1], 0, 180), 0, 180), 0, 180);
  send_payload[lift_cell_number] = constrain(int(stick_trim[1]) + 180 - constrain(map(analogRead(LIFT), stick_range[1][0], stick_range[1][1], 0, 180), 0, 180), 0, 180);

  // set engine
  if (engine_on) {
    send_payload[motor_cell_number] = constrain(3150 - constrain(map(analogRead(MOTOR), stick_range[2][0], (stick_range[2][0]+stick_range[2][1])/2, 850, 2300), 850, 2300), 850, 2300);
  } else {
    send_payload[motor_cell_number] = 850;
  }
  
  // set rudder
  send_payload[rudder_cell_number] = constrain(int(stick_trim[3]) + constrain(map(analogRead(RUDDER), stick_range[3][0], stick_range[3][1], 0, 180), 0, 180), 0, 180);

  // set counter
  send_payload[sent_counter_cell_number] = message_counter;
}

void pipeIt() {
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  
  radio.openWritingPipe(address[radioNumber]);
  radio.openReadingPipe(1, address[!radioNumber]);
}

void findBestChannel() {
  uint8_t values[num_channels];
  
  memset(values,0,sizeof(values));
  int rep_counter = 100;
  while (rep_counter--) {
    int i = num_channels;
    while (i--) {
      radio.setChannel(i);
      radio.startListening();
      delayMicroseconds(128);
      radio.stopListening();
      if ( radio.testCarrier() )
        ++values[i];
    }
  }

  byte first_void_channel;
  byte last_void_channel;
  byte best_first_void_channel = 0;
  byte best_last_void_channel = 0;
  
  int i = 0;
  Serial.println("channel data");
  while (i < num_channels) {
    if(values[i] == 0 && (values[max(i-1, 0)] != 0 || i == 0)) {
      if (best_last_void_channel - best_first_void_channel < last_void_channel - first_void_channel) {
        best_first_void_channel = first_void_channel;
        best_last_void_channel = last_void_channel;
      }
      first_void_channel = i;
    } else if((values[i] != 0 && values[max(i-1, 0)] == 0) || i == (num_channels - 1)) {
      last_void_channel = i;
    }
    Serial.print(String(values[i]) + " ");
    ++i;
  }
  Serial.println("");
  Serial.println(last_void_channel);
  future_best_channel = best_first_void_channel + ((int) (best_last_void_channel - best_first_void_channel) / 2);

  radio.setChannel(best_channel);
}
