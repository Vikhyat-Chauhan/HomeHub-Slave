#include <EEPROM.h>
#include <ArduinoJson.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <DHT.h> 

#define DHTTYPE DHT11   // DHT 22  (AM2302)

//Slave Input data port
#define IN_DATA_PORT Serial
#define IN_DATA_PORT_BAUD 115200
//Slave output data port
#define OUT_DATA_PORT Serial
#define OUT_DATA_PORT_BAUD 115200

//Devices Count
#define ALL_RELAY_NUMBER 4
#define ALL_FAN_NUMBER 2
#define ALL_SENSOR_NUMBER 1
#define ALL_BUTTON_NUMBER 4

//Memory start pages continues till device count
#define ALL_RELAY_MEMSPACE 10

//Button special features
#define BUTTON_LONGPRESSDELAY 5000
#define BUTTON_MULTIPRESSTIME 3000
#define BUTTON_MULTIPRESSCOUNT 7
#define BUTTON_TYPE true              //True represents the Tap button type, while False activates the switch button type

//hardware pin number arrays
const int relay_pin[ALL_RELAY_NUMBER] = {8,9,10,11};
const int button_pin[ALL_BUTTON_NUMBER] = {4,5,6,7};
const int sensor_pin[ALL_SENSOR_NUMBER] = {14};
const char * sensor_type[ALL_SENSOR_NUMBER] = {"TEMP"};

const int capacity = 200; 

//Slave Handling Variables
unsigned int IN_DATA_PORT_counter = 0;
bool _receiving_json = false;
bool _received_json = false;
String IN_DATA_PORT_command = "";
String in_command_buffer = "";

typedef struct{
  bool longpress = false;
  bool handshake = false;
  bool sync = false;
}FLAG;

typedef struct{
    bool current_state;
    bool previous_state;
    unsigned long current_value;
    unsigned long previous_value;
    boolean sleep = false;
    int memspace;
    int pin;
    bool change = false;
    bool lastcommand = false;
}RELAY;

typedef struct{
    bool current_state;
    bool previous_state;
    unsigned long current_value;
    unsigned long previous_value;
    int memspace;
    bool change = false;
    bool lastcommand = false;
}FAN;

typedef struct{
    const char* type;
    int pin;
    int current_value;
    unsigned long previous_value;
    bool change = false;
    bool lastcommand = false;
    bool error = false;
    bool error_code = 0;
    unsigned long millis_time_stamp = 30000;  //Keep it equal boot_time_delay
    unsigned long read_time_delay = 2000; 
    unsigned long boot_time_delay = 30000;
}SENSOR;

typedef struct{
    int pin;
    int press_counter;
    bool previous_pressed;
    bool previous_state;
    bool longpress;
    unsigned long pressed_millis;
}BUTTON;

typedef struct{
    const char* NAME;
    bool change = false;
    unsigned int RELAY_NUMBER = ALL_RELAY_NUMBER;
    unsigned int FAN_NUMBER = ALL_FAN_NUMBER;
    unsigned int SENSOR_NUMBER = ALL_SENSOR_NUMBER;
    unsigned int BUTTON_NUMBER = ALL_BUTTON_NUMBER;
    FLAG flag;
    RELAY relay[ALL_RELAY_NUMBER];
    FAN fan[ALL_FAN_NUMBER];
    SENSOR sensor[ALL_SENSOR_NUMBER];
    BUTTON button[ALL_BUTTON_NUMBER];
}SLAVE;

DHT dht(sensor_pin[0], DHTTYPE);
SLAVE slave;

void setup() {
  Serial.begin(115200);         // start serial for debug
  Serial.flush();
  //Timer for bluetooth data
  structure_initilizer(); //Should be above all slave structure using functions : read_saved_values(),
  lastsaveddata_initilizer();
  device_initilizer();
  for(int i=0;i<slave.BUTTON_NUMBER;i++){
    pinMode(slave.button[i].pin,INPUT_PULLUP);
  }
  interrupt_initilizer();
}

void loop(){
  async_tasks();
}

void async_tasks(){
  master_input_handler();
  device_handler();
  master_output_handler();
  //reset button longpress booleans after sending their change details.
  for(int i=0;i<slave.BUTTON_NUMBER;i++){
    slave.button[i].longpress = false;
  }
}

void master_output_handler(){
  if(slave.flag.handshake == true){ 
    const size_t capacity = 400;
    DynamicJsonDocument doc(capacity);
    doc["NAME"] = "4RELAY"; 
    doc["ROLE"] = "SLAVE";
    
    JsonObject device = doc.createNestedObject("DEVICE"); 
    
    //All Relay Devices here
    JsonArray relay = device.createNestedArray("RELAY");
    for(int i=0;i<slave.RELAY_NUMBER;i++){
      JsonObject relay_x = relay.createNestedObject();
      relay_x["STATE"] = slave.relay[i].current_state;
      relay_x["VALUE"] = slave.relay[i].current_value;
    }
    //All FAN Devices here
    JsonArray fan = device.createNestedArray("FAN");
    for(int i=0;i<slave.FAN_NUMBER;i++){
      JsonObject fan_x = fan.createNestedObject();
      fan_x["STATE"] = slave.fan[i].current_state;
      fan_x["VALUE"] = slave.fan[i].current_value;
    }
    //All Sensor Devices here
    JsonArray sensor = device.createNestedArray("SENSOR");
    for(int i=0;i<slave.SENSOR_NUMBER;i++){
      JsonObject sensor_x = sensor.createNestedObject();
      sensor_x["TYPE"] = slave.sensor[i].type;
      sensor_x["VALUE"] = slave.sensor[i].current_value;
    }
    //All Button Devices here only in handshake, because we dont use them after handshake decleration
    JsonArray button = device.createNestedArray("BUTTON");
    for(int i=0;i<slave.BUTTON_NUMBER;i++){
      JsonObject button_x = button.createNestedObject();
      button_x["TYPE"] = BUTTON_TYPE;
      button_x["STATE"] = false;
    }
    //Command
    String command = push_command();
    if(command != "NULL"){
      doc["COMMAND"] = command;
    }
    serializeJson(doc, OUT_DATA_PORT);
    slave.change = false;
    slave.flag.handshake = false;
    slave.flag.sync = true;
  }
  //Handless normal sync after initial handshakes
  if(slave.flag.sync == true){
  if(slave.change == true){ 
    //All summing up counters
    int relay_change_counter = 0;
    int relay_lastcommand_counter = 0;
    int fan_change_counter = 0;
    int fan_lastcommand_counter = 0;
    int sensor_change_counter = 0;
    int sensor_lastcommand_counter = 0;
    //Counting the sum of all device changes to decide to add device array to output json or not
    for(int i=0;i<slave.RELAY_NUMBER;i++){
      if(slave.relay[i].change == true){
        relay_change_counter++;
      }
      if(slave.relay[i].lastcommand == false){
        relay_lastcommand_counter++;
      }
    }
    for(int i=0;i<slave.FAN_NUMBER;i++){
      if(slave.fan[i].change == true){
        fan_change_counter++;
      }
      if(slave.fan[i].lastcommand == false){
        fan_lastcommand_counter++;
      }
    }
    for(int i=0;i<slave.SENSOR_NUMBER;i++){
      if(slave.sensor[i].change == true){
        sensor_change_counter++;
      }
      if(slave.sensor[i].lastcommand == false){
        sensor_lastcommand_counter++;
      }
    }
    const size_t capacity = 400;
    DynamicJsonDocument doc(capacity);
    
    doc["NAME"] = "4RELAY"; 
    doc["ROLE"] = "SLAVE";
    
    JsonObject device = doc.createNestedObject("DEVICE"); 
    
    //All Relay Devices here
    if(relay_change_counter !=0){
      if(relay_lastcommand_counter != 0){
        JsonArray relay = device.createNestedArray("RELAY");
        for(int i=0;i<slave.RELAY_NUMBER;i++){
          JsonObject relay_x = relay.createNestedObject();
          if(slave.relay[i].change == true){
            if(slave.relay[i].lastcommand != true){
              relay_x["STATE"] = slave.relay[i].current_state;
              relay_x["VALUE"] = slave.relay[i].current_value;
            }
          }
        }
      }
    }
    //All FAN Devices here
    if(fan_change_counter !=0){
      if(fan_lastcommand_counter != 0){
        JsonArray fan = device.createNestedArray("FAN");
        for(int i=0;i<slave.FAN_NUMBER;i++){
          JsonObject fan_x = fan.createNestedObject();
          if(slave.fan[i].change == true){
            if(slave.relay[i].lastcommand != true){
              fan_x["STATE"] = slave.fan[i].current_state;
              fan_x["VALUE"] = slave.fan[i].current_value;
            } 
          }
        }
      }
    }
    //All Sensor Devices here
    if(sensor_change_counter != 0){
      //if(sensor_lastcommand_counter != 0){
        JsonArray sensor = device.createNestedArray("SENSOR");
        for(int i=0;i<slave.SENSOR_NUMBER;i++){
          JsonObject sensor_x = sensor.createNestedObject();
          if(slave.sensor[i].change == true){
            //if(slave.sensor[i].lastcommand != true){
              sensor_x["TYPE"] = slave.sensor[i].type;
              sensor_x["VALUE"] = slave.sensor[i].current_value;
            //}
          }
        }
      //}
    }
    //Command
    String command = push_command();
    if(command != "NULL"){
      doc["COMMAND"] = command;
    }
    serializeJson(doc, OUT_DATA_PORT);
    slave.change = false;
  }
  }
}

String push_command(){
  if(slave.flag.longpress == true){
    slave.flag.longpress = false;
    String output_command = "ALL_LONGPRESS";
    return output_command;
  }
  else{
    String output_command = "NULL";
    return output_command;
  }
}

void serial_json_input_capture(){
  //Read incomming bit per cycle and decode the incomming commands
    if(IN_DATA_PORT.available()) {
        char c = IN_DATA_PORT.read();
        if (c == '{'){
            if(IN_DATA_PORT_counter==0){
                _receiving_json = true;
                IN_DATA_PORT_command = "";
            }
            IN_DATA_PORT_counter+=1;
            IN_DATA_PORT_command += c;
        }
        else if (c == '}'){
            IN_DATA_PORT_counter+=-1;
            IN_DATA_PORT_command += c;
        }
        else{
            IN_DATA_PORT_command += c;
        }
        if(IN_DATA_PORT_counter == 0 && _receiving_json == true){
            in_command_buffer = IN_DATA_PORT_command;               
            IN_DATA_PORT_command = "";
            _receiving_json = false;
            _received_json = true;
        }
    }
}

void master_input_handler(){
    //resetting relay last command flags to false and change depending on the input json command below
    for(int i=0;i<slave.RELAY_NUMBER;i++){
          slave.relay[i].lastcommand = false;
        }
    if(_received_json == true){ 
        const size_t capacity = 400;
        DynamicJsonDocument doc(capacity);
        deserializeJson(doc, in_command_buffer);
        JsonObject obj = doc.as<JsonObject>();
        JsonArray relays = doc["DEVICE"]["RELAY"];
        JsonArray fans = doc["DEVICE"]["FAN"];
        JsonArray sensors = doc["DEVICE"]["SENSOR"];
        int index = 0;
        for (JsonObject repo : relays){
            if(repo.containsKey("STATE")) { //Check if Key is actually present in it, or it will insert default values in places of int
               slave.relay[index].current_state = repo["STATE"].as<bool>();
               slave.relay[index].lastcommand = true;
            }
            if(repo.containsKey("VALUE")) {
               slave.relay[index].current_value = repo["VALUE"].as<int>();
               slave.relay[index].lastcommand = true;
            }
            index++;
        }
        index = 0;
        for (JsonObject repo : fans){
            if(repo.containsKey("STATE")) { //Check if Key is actually present in it, or it will insert default values in places of int
               slave.fan[index].current_state = repo["STATE"].as<bool>();
               slave.fan[index].lastcommand = true;
            }
            if(repo.containsKey("VALUE")) {
               slave.fan[index].current_value = repo["VALUE"].as<int>();
               slave.fan[index].lastcommand = true;
            }
            index++;
        }
        index = 0;
        for (JsonObject repo : sensors){
          if(repo.containsKey("TYPE")) { //Check if Key is actually present in it, or it will insert default values in places of int
             slave.sensor[index].type = repo["TYPE"].as<char *>();
             slave.sensor[index].lastcommand = true;
          }
          if(repo.containsKey("VALUE")){
            slave.sensor[index].current_value = repo["VALUE"].as<int>();
            slave.sensor[index].lastcommand = true;
          }
            index++;
        }
        String command = doc["COMMAND"].as<char *>();
        receive_command(command);
        _received_json = false;
        }
}

String receive_command(String command){
  if(command == "HANDSHAKE"){
    slave.flag.handshake = true;
  }
  else if(command == "LIGHT_TOGGLE"){
    
  }
  else if(command == "SLEEP_TOGGLE"){
    
  }
}

void device_handler(){
  //Handles hardware writing of any relay changes and monitors any relay changes to document them
  for(int i =0;i<slave.RELAY_NUMBER;i++){
    if(slave.relay[i].sleep == false){
    //Change has occured in the state
      if(slave.relay[i].current_state != slave.relay[i].previous_state){
        slave.relay[i].change = true; //Enter change variable for relay
        slave.change = true; //Enter change variable for overall slave change
        EEPROM.write(slave.relay[i].memspace,slave.relay[i].current_state); delay(5);
        slave.relay[i].previous_state = slave.relay[i].current_state;
      }
      else{ //No Change has been encountered
        slave.relay[i].change = false;
        slave.relay[i].previous_state = slave.relay[i].current_state;
      }
    digitalWrite(slave.relay[i].pin,!slave.relay[i].current_state);
    }
  }
  //Temperature Sensor handler 
  temperaturesensor_handler();  //temperature sensor sensor is at array place 1
}

void button_handler(){ 
  for(int i=0;i<slave.BUTTON_NUMBER;i++){
    int button_state = digitalRead(slave.button[i].pin); 
    if(BUTTON_TYPE){
      if(button_state == 0){ //button is pressed 
       if(slave.button[i].previous_pressed){
         //we will skip it till it's pressed, can change device state here
       }
       else{
         slave.button[i].previous_pressed= true;
         slave.button[i].pressed_millis = millis();
       }
      }
     else{
       if(slave.button[i].previous_pressed){
         if((millis() - slave.button[i].pressed_millis)> BUTTON_LONGPRESSDELAY){
           slave.button[i].longpress = true;
           slave.flag.longpress = true;
           slave.change = true;
         }
         else{
           slave.relay[i].current_state = !slave.relay[i].current_state;                //button his now left after pressing, can change device state here
           digitalWrite(slave.relay[i].pin,slave.relay[i].current_state);
         }
         slave.button[i].previous_pressed= false;
      }
      else{
        slave.button[i].previous_pressed= false;
      //no button pressed for a long time
      }
    }
    }
    else{
      if(button_state != slave.button[i].previous_state){       //Button state changed
        if(slave.button[0].press_counter == 0){                    //initiate multiple press counter
          slave.button[i].pressed_millis = millis();
          slave.button[i].press_counter += 1;
        }
        else if((slave.button[i].press_counter >0) && (slave.button[i].press_counter <BUTTON_MULTIPRESSCOUNT)){//Add counter 
          slave.button[i].press_counter+=1;
        }
        else if(slave.button[0].press_counter >= BUTTON_MULTIPRESSCOUNT){                 //Button press 7 times
          if((millis() - slave.button[i].pressed_millis) < BUTTON_MULTIPRESSTIME){    //check time it took press 7 buttons, 3 sec here
            slave.button[i].longpress = true;
            slave.flag.longpress = true;
            slave.change = true;
          }
          slave.button[0].press_counter = 0;
        }
        slave.relay[i].current_state = !slave.relay[i].current_state;                //button his now left after pressing, can change device state here
        digitalWrite(slave.relay[i].pin,slave.relay[i].current_state);
        slave.button[i].previous_state = button_state;
      }
   }
  }  
}

void lastsaveddata_initilizer(){
  //For Relay Sensor Struct
  for(int i=0;i<slave.RELAY_NUMBER;i++){
    slave.relay[i].current_state = EEPROM.read(slave.relay[i].memspace);
    slave.relay[i].previous_state = slave.relay[i].current_state;
  }
}

void temperaturesensor_handler(){
  int sensor_number;  
  for(int i=0;i<slave.SENSOR_NUMBER;i++){
    if(slave.sensor[i].type == "TEMP"){      //Find the array position of temperature sensor in sensor structure 
      sensor_number = i;
    }
  }
  unsigned long Millis = millis();
  if(Millis > (slave.sensor[sensor_number].boot_time_delay)){
    if((Millis - slave.sensor[sensor_number].millis_time_stamp) >  slave.sensor[sensor_number].read_time_delay){
      slave.sensor[sensor_number].millis_time_stamp = Millis;
    //Read data and store it to variables temp
    int temp = dht.readTemperature();
    //Temperature stability function
    if(20<temp && temp<120){ 
      slave.sensor[sensor_number].error = 0;
      if(temp != slave.sensor[sensor_number].current_value){
        slave.sensor[sensor_number].current_value = temp;
        slave.sensor[sensor_number].change = true;
        slave.change = true;
      }
      else{
        slave.sensor[sensor_number].change = false;
      }
      slave.sensor[sensor_number].current_value = temp; 
    }
    else{
      slave.sensor[sensor_number].current_value = 0;
        if(temp<20){
          slave.sensor[sensor_number].error = 1;
        }
        else if(temp>120){
          slave.sensor[sensor_number].error = 2;
        }
      }
    }
  }
}

void structure_initilizer(){
  //Initilizing Relay structures inside slave structure
  for(int i=0;i<ALL_RELAY_NUMBER;i++){
    //put relay pins in relay structures for use throught the program
    slave.relay[i].pin = relay_pin[i];
    //put relay memorys spaces in relay structures for use through the program
    slave.relay[i].memspace = ALL_RELAY_MEMSPACE + i;
    }
  //Initilizing BUTTON structures inside slave structure
  for(int i=0;i<ALL_BUTTON_NUMBER;i++){
    //put button pins in button structures for use throught the program
    slave.button[i].pin = button_pin[i];
    }
  //Initilizing SENSOR structures inside slave structure
  //Temperature SENSOR number 1
  for(int i=0;i<ALL_SENSOR_NUMBER;i++){
    //put sensor pins in sensor structures for use throught the program
    slave.sensor[i].pin = sensor_pin[i];
    slave.sensor[i].type = sensor_type[i];
    }
}

void device_initilizer(){
  //Initilizing relays from the presaved values after booting 
    for(int i=0;i<slave.RELAY_NUMBER;i++){
        pinMode(slave.relay[i].pin,OUTPUT);
        //Serial.print(slave.relay[i].pin);Serial.print("  ");Serial.println(slave.relay[i].current_state);
        digitalWrite(slave.relay[i].pin,!slave.relay[i].current_state);
    }
    //Initilizing sensor from the presaved values after booting 
    /*for(int i=0;i<ALL_SENSOR_NUMBER;i++){
    //put sensor pins in sensor structures for use throught the program
      pinMode(slave.sensor[i].pin,INPUT);
    }*/
    dht.begin();
}

void interrupt_initilizer(){
  Timer1.initialize(20);                     //100 - 1000
  Timer1.attachInterrupt(serial_json_input_capture);
  MsTimer2::set(100, button_handler); // 500ms period
  MsTimer2::start();
}
