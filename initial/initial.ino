//#include <EEPROM.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <MsTimer2.h>
#include <DS3231.h>
#include <DHT.h> 
#include <EEPROM.h>

#define DATA_PORT Serial
#define DATA_PORT_BAUD 9600
#define DHTTYPE DHT11   // DHT 22  (AM2302)
#define rom_address 0x57

#define ALL_RELAY_NUMBER 4
#define ALL_FAN_NUMBER 0
#define ALL_SENSOR_NUMBER 2
#define ALL_RELAY_MEMSPACE 10

int relay_pin[ALL_RELAY_NUMBER] = {8,9,10,11};

#define serialport Serial
SoftwareSerial mySerial(3, 2); // RX, TX
    
typedef struct{
    bool state;
    unsigned long value;
    boolean sleep = false;
    int memspace;
    int pin;
    byte current_state;
    byte previous_state;
    boolean change= false;
}RELAY;

typedef struct{
    bool state;
    unsigned long value;
    boolean change= false;
}FAN;

typedef struct{
    const char* type;
    unsigned long value;
    boolean change= false;
}SENSOR;

typedef struct{
    const char* NAME;
    unsigned int RELAY_NUMBER = ALL_RELAY_NUMBER;
    unsigned int FAN_NUMBER = ALL_FAN_NUMBER;
    unsigned int SENSOR_NUMBER = ALL_SENSOR_NUMBER;
    RELAY relay[ALL_RELAY_NUMBER];
    FAN fan[ALL_FAN_NUMBER];
    SENSOR sensor[ALL_SENSOR_NUMBER];
}SLAVE;

SLAVE slave;

String databuffer = "";
unsigned int DATA_PORT_counter = 0;
bool _receiving_json = false;
bool _received_json = false;
bool _initiate_AP = false;
String DATA_PORT_command = "";
String command_buffer = "";
        
//Flags
boolean initiateAP_flag = false;
boolean sync_flag = false;
boolean auto_save_flag = false;
boolean automatic_response_flag = true;

// Variable for external buttons long press
int button_type;
const int button_number = 4;
const int button_pin[button_number+1] = {NULL,4,5,6,7};
int button_press_counter = 0;
bool button_previous_pressed[button_number+1] = {NULL,false,false,false,false};
int button_previous_state[button_number+1] = {NULL,0,0,0,0};
unsigned long button_pressed_millis = 0;

//Temperature Sensor related Struct
byte tsensor_pin = 14;
int tsensor_temp,tsensor_humi,tsensor_error_code;
boolean tsensor_change = true;
boolean tsensor_error;
byte tsensor_memspace[5] = {NULL,250,251,252,253};

DS3231 Clock;
DHT dht(tsensor_pin, DHTTYPE);

void setup() {
  //Read the saved device items
  // Start the I2C interface
  Wire.begin();
  dht.begin();
  Serial.begin(9600);         // start serial for debug
  //Timer for bluetooth data
  initilise_interrupts();
  initilise_structures();
  read_saved_values();
  for(int i=0;i<slave.RELAY_NUMBER;i++){
    pinMode(relay_pin[i],OUTPUT);
  }
  for(int i=1;i<=button_number;i++){
    pinMode(button_pin[i],INPUT_PULLUP);
    button_previous_state[i] = digitalRead(button_pin[i]);
  }
}

void loop(){
  //handle_command();delay(1000);
  async_tasks();
}

void async_tasks(){
  /*auto_save();
  temp_sensor_handler();
  time_sensor_handler();
  relay_handler();
  automatic_response();
  send_packet();*/
}

void initilise_structures(){
  //Initilizing relay structures inside slave structure
  for(int i=0;i<ALL_RELAY_NUMBER;i++){
    //put relay pins in relay structures for use throught the program
    slave.relay[i].pin = relay_pin[i];
    //put relay memorys spaces in relay structures for use through the program
    slave.relay[i].memspace = ALL_RELAY_MEMSPACE + i;
    }
}

void read_saved_values(){
  //Button type setting
  button_type = EEPROM.read(301);
  //For Relay Sensor Struct
  for(int i=0;i<slave.RELAY_NUMBER;i++){
    slave.relay[i].current_state = EEPROM.read(slave.relay[i].memspace);
    slave.relay[i].previous_state = slave.relay[i].current_state;
  }
  //For Temperature Sensor
  /*tsensor_temp = EEPROM.read(tsensor_memspace[1]); 
  tsensor_humi = EEPROM.read(tsensor_memspace[2]);
  tsensor_error = EEPROM.read(tsensor_memspace[3]);
  tsensor_error_code = EEPROM.read(tsensor_memspace[4]);*/
}

void initilise_interrupts(){
  Timer1.initialize(100);                     //100 - 1000
  Timer1.attachInterrupt(master_handler);
  MsTimer2::set(100, button_handler); // 500ms period
  MsTimer2::start();
}

void auto_save(){
  if(auto_save_flag == true){
    rom_write(301,button_type); 
    auto_save_flag = false;
  }
}

void automatic_response(){
  //Place Semi Essential responses here, which can be stopped via flag for a single cycle
  if(automatic_response_flag == true){
    for(int i=0;i<slave.RELAY_NUMBER;i++){
      if(slave.relay[i].change == true){
        String data = "<DE";
        data = data + String(i) + ">";
        data = data + String(slave.relay[i].current_state);
        data = data + "X";
        send_data(data);
      }
    }
  }
  else{
    automatic_response_flag = true;
  }
  //Place Essential responses here
  if(initiateAP_flag == true){
    send_data("<WSP>0X");
    initiateAP_flag = false;
  }
  if(sync_flag == true){
    String command = "";
    //Sending Device 1 Status
    for(int i=0;i<=slave.RELAY_NUMBER;i++){
      String command = "<DE";
      command = command + String(i) + ">";
      command = command + String(slave.relay[i].current_state);
      command = command + "X";
      send_data(command); 
    }
    //Sending Device 2 Status 
    command = "<TEM>"+String(tsensor_temp)+"X";
    send_data(command);
    //Sending Device 2 Status 
    command = "<HUM>"+String(tsensor_temp)+"X";
    send_data(command);
    //Sending Device Status 
    sync_flag = false;
  }
  if(tsensor_change == 0){
    String data = "<TEM>"+String(tsensor_temp)+"X";
    send_data(data);        
  }
}

void relay_handler(){
  for(int i =0;i<slave.RELAY_NUMBER;i++){
    if(slave.relay[i].sleep == false){
    //Change has occured in the state
      if(slave.relay[i].current_state != slave.relay[i].previous_state){
        slave.relay[i].change = true;
        EEPROM.write(slave.relay[i].memspace,slave.relay[i].current_state);
        slave.relay[i].previous_state = slave.relay[i].current_state;
      }
      else{ //No Change has been encountered
        slave.relay[i].change = false;
        slave.relay[i].previous_state = slave.relay[i].current_state;
      }
    digitalWrite(slave.relay[i].pin, slave.relay[i].current_state);
    }
  }
}

void button_handler(){ 
  for(int i=1;i<=button_number;i++){
    int button_state = digitalRead(button_pin[i]);
    if(button_type == 0){
      if(button_state == 0){ //button is pressed 
       if(button_previous_pressed[i]){
         //we will skip it till it's pressed, can change device state here
       }
       else{
         button_previous_pressed[i]= true;
         button_pressed_millis = millis();
       }
      }
     else{
       if(button_previous_pressed[i]){
         if((millis() - button_pressed_millis)> 5000){
           initiateAP_flag = true;  
         }
         else{
           slave.relay[i-1].current_state = !slave.relay[i-1].current_state;                //button his now left after pressing, can change device state here
           digitalWrite(slave.relay[i].pin,slave.relay[i].current_state);
         }
         button_previous_pressed[i]= false;
      }
      else{
        button_previous_pressed[i]= false;
      //no button pressed for a long time
      }
    }
    }
    else{
      if(button_state != button_previous_state[i]){       //Button state changed
        if(button_press_counter == 0){                    //initiate multiple press counter
          button_pressed_millis = millis();
          button_press_counter += 1;
        }
        else if((button_press_counter >0) && (button_press_counter <7)){//Add counter 
          button_press_counter+=1;
        }
        else if(button_press_counter >= 7){                 //Button press 7 times
          if((millis() - button_pressed_millis) < 3000){    //check time it took press 7 buttons, 3 sec here
            initiateAP_flag = true;
          }
          button_press_counter = 0;
        }
        slave.relay[i-1].current_state = !slave.relay[i-1].current_state;                //button his now left after pressing, can change device state here
        digitalWrite(slave.relay[i-1].pin,slave.relay[i-1].current_state);
        button_previous_state[i] = button_state;
      }
   }
  }  
}

void master_handler(){
  //Read incomming bit per cycle and decode the incomming commands
    if(DATA_PORT.available()) {
        char c = DATA_PORT.read();
        if (c == '{'){
            if(DATA_PORT_counter==0){
                _receiving_json = true;
                DATA_PORT_command = "";
            }
            DATA_PORT_counter+=1;
            DATA_PORT_command += c;
        }
        else if (c == '}'){
            DATA_PORT_counter+=-1;
            DATA_PORT_command += c;
        }
        else{
            DATA_PORT_command += c;
        }
        if(DATA_PORT_counter == 0 && _receiving_json == true){
            //DATA_PORT.println(DATA_PORT_command);
            command_buffer = DATA_PORT_command;
            DATA_PORT_command = "";
            _receiving_json = false;
            _received_json = true;
        }
    }
}

void master_sync_handler(){/*
    if(_received_json == true){
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, command_buffer);
        JsonObject obj = doc.as<JsonObject>();
        JsonArray relays = doc["DEVICE"]["RELAY"];
        JsonArray fans = doc["DEVICE"]["FAN"];
        JsonArray sensors = doc["DEVICE"]["SENSOR"];
        slave.NAME = doc["NAME"];
        slave.RELAY_NUMBER = relays.size();
        slave.FAN_NUMBER = fans.size();
        slave.SENSOR_NUMBER = sensors.size();
        int index = 0;
        for (JsonObject repo : relays){
            slave.relay[index].state = repo["STATE"].as<bool>();
            slave.relay[index].value = repo["VALUE"].as<int>();
            index++;
        }
        index = 0;
        for (JsonObject repo : fans){
            slave.fan[index].state = repo["STATE"].as<bool>();
            slave.fan[index].value = repo["VALUE"].as<int>();
            index++;
        }
        index = 0;
        for (JsonObject repo : sensors){
            slave.sensor[index].type = repo["TYPE"].as<char *>();
            slave.sensor[index].value = repo["VALUE"].as<int>();
            index++;
        }
    _received_json = false;
    const char* command = doc["COMMAND"];
    if(command != nullptr){
        slave_command_processor(command);
    }
  }*/
}

void master_command_processor(const char* command){/*
    char Command[20];
    strlcpy(Command, command,20);
    if((strncmp(Command,"WIFI_SETUP_START",20) == 0)){
        HomeHub_DEBUG_PRINT("Starting Wifi Setup AP");
        initiate_wifi_setup();
    }
    else if((strncmp(Command,"WIFI_SETUP_STOP",20) == 0)){
        HomeHub_DEBUG_PRINT("Stopping Wifi Setup AP");
        end_wifi_setup();
    }
    else if((strncmp(Command,"WIFI_SAVED_CONNECT",20) == 0)){
        HomeHub_DEBUG_PRINT("Connecting to presaved Wifi");
        saved_wifi_connect();
    }
    else if((strncmp(Command,"UPDATE_DEVICE",20) == 0)){
        HomeHub_DEBUG_PRINT("Connecting to presaved Wifi");
        update_device();
    }
    else{
        HomeHub_DEBUG_PRINT("Unsupported Command");
    }*/
}

void handle_command(){/*
  if(received_json == true){Serial.println("Processing json");
  
    String input =
      "{\"ID\":\"ESP8266\",\"ROLE\":\"MASTER\",\"REQUEST\":\"HANDSHAKE\"}";

  
    Serial.println(input);
    received_json = false;
  }
  
  result = 'x';
  if(DATA_PORT_command == "<SYN>"){
    sync_flag = true; //will give 0 for connection and 1 for disconnection
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<DE1>"){
    automatic_response_flag = false;
    relay_current_state[1] = value.toInt();
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<DE2>"){
    automatic_response_flag = false;
    relay_current_state[2] = value.toInt();
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<DE3>"){
    automatic_response_flag = false;
    relay_current_state[3] = value.toInt();
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<DE4>"){
    automatic_response_flag = false;
    relay_current_state[4] = value.toInt();
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<BUT>"){
    button_type = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<WSP>"){
    initiateAP_flag = true; 
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<dse>"){
    DEVICESELECT = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<MOO>"){
    ONMOTIONFEATURE = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<MOF>"){
    OFFMOTIONFEATURE = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<TEO>"){
    ONTEMPFEATURE[DEVICESELECT] = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<TEF>"){
    OFFTEMPFEATURE[DEVICESELECT] = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<TIO>"){
    ONTIMEFEATURE[DEVICESELECT] = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<TIF>"){
    OFFTIMEFEATURE[DEVICESELECT] = value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  //data Values
  else if(DATA_PORT_command == "<teo>"){
    OnDeviceTempValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<tef>"){
    OffDeviceTempValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  //data Values
  else if(DATA_PORT_command == "<tho>"){
    OnDeviceHourValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<tmo>"){
    OnDeviceMinuValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<thf>"){
    OffDeviceHourValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  }
  else if(DATA_PORT_command == "<tmf>"){
    OffDeviceMinuValue[DEVICESELECT]= value.toInt();
    auto_save_flag = true;
    result = 's'; // transmission successfully Read
  } 
  else{
    //Device Recieved data but not matched
  }
  serialport.print(result);*/
}

void tsensor(){
  //Read data and store it to variables temp
  int temp = dht.readTemperature();
  int humi = dht.readHumidity();
  bool changed_above = false;
  //Temperature stability function
  if(20<temp && temp<120){ 
    tsensor_error = 1;
    if(temp != tsensor_temp){
      tsensor_change = 0;
      changed_above = true;
      rom_write(tsensor_memspace[1],temp);//EEPROM.write(tsensor_memspace[1],temp);
    }
    else{
      tsensor_change = 1;
    }
   tsensor_temp = temp; 
  }
  else{
    tsensor_temp = 0;
    if(temp<20){
      tsensor_error_code = 00;
    }
    else if(temp>120){
      tsensor_error_code = 01;
    }
    rom_write(tsensor_memspace[3],tsensor_error);//EEPROM.write(tsensor_memspace[3],tsensor_error);
    rom_write(tsensor_memspace[4],tsensor_error_code);//EEPROM.write(tsensor_memspace[4],tsensor_error_code);
  }
  //Humidity stability function
  if(0<=humi && humi<=100){ 
    tsensor_error = 1;
    if(humi != tsensor_humi){
      tsensor_change = 0;
      rom_write(tsensor_memspace[2],humi);//EEPROM.write(tsensor_memspace[2],humi);
    }
    else{
      if(changed_above == false){
      tsensor_change = 1;
      }
    }
   tsensor_humi = humi; 
  }
  else{
    tsensor_humi = 0;
    if(humi<0){
      tsensor_error_code = 10;
    }
    else if(humi>100){
      tsensor_error_code = 11;
    }
    rom_write(tsensor_memspace[3],tsensor_error);//EEPROM.write(tsensor_memspace[3],tsensor_error);
    rom_write(tsensor_memspace[4],tsensor_error_code);//EEPROM.write(tsensor_memspace[4],tsensor_error_code);
  }
  
  //Send data
  if(tsensor_error == 1){    //No Error
    if(tsensor_change == 0){ //Change observed in sensor
      String data = "<TEM>" + String(tsensor_temp) + "X";//Sending data
      //data = "<HUM>" + String(Th_sensor.Humidity) + "X";//Sending data
      //dataPush(data);
    }
  }
  else{                        // Error observed in data
    //Sending Error code
  }
}

void send_data(String command){
    databuffer += command;
}

void send_packet(){
  int x = databuffer.indexOf('X');
  if(x>0){
  String command = databuffer.substring(0,x+1);
  serialport.print(command);
  serialport.flush();
  databuffer = databuffer.substring(x+1,databuffer.length());
  }
}

//Custom EEPROM replacement functions
void rom_write(unsigned int eeaddress, byte data){
  int rdata = data;
  Wire.beginTransmission(rom_address);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission(); 
  delay(10); //Delay introduction solved the data save unreliability
}

//Custom EEPROM replacement functions
byte rom_read(unsigned int eeaddress){
  byte rdata = 0xFF;
  Wire.beginTransmission(rom_address);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(rom_address,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}
