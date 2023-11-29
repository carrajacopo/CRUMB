/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Sensor Package CRUMB (Compact Radio Unit for Moon data Broadcasting) Code
//  LANE - Lunear Advanced NEtwork
//  Version LANE.57
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Developer History:
// 06.2023 - 11.2023: Jacopo Carra
// 05.2019 - 12.2019: Jan-Henrik Zünkler
// 11.2019 -  : Justin Dalrymple
// 11.2019 -  : Flavie Rometsch
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Archievements:
//
// Acceleremeter
// Gravimetric Measurements
// Grvity Processing
// Serial Display
// OLED Display
// Lara messaging
// Package forewarding
// Stamp Messages
// Multiple times forewarding fixed
// Working Message interpreter
// Processing Paths from Messages
// Storing possible paths
// Timeout for very old paths
// Finding the best path
// updated to and over stamps in transmit radings
// Finding to and over stamps in messages
// Working with to and over stamps
// Mesh Mode - Finding the fastest path
// Updating the Path analyser so that also part-parts are recognised as paths
// Ping - !!!!!!!!!!!!!!!!!!!!!!!!!! works, but not tested sufficiently !!!!!!!!!!!!!!!!!!!!!!!
// Updated Serial Print for node red connection
// Updated the Pathfinder Function, so that a path is only stored in one direction, and not 2 for each direction
// implementing serial console
// Interpreting commands
// only send when there is no network traffic - parsepacket - reduced netework travel by waiting for no network travel
// chat
// store to transmission storage issue
// commands
// different sensors - sensor fuctions unifying
// raw readings
// calibration
// reset
// sending commands
// Handling of received messages
// Interpreting Commands
// SOS
// different sensors at the same time enabled
// Wireless Status and Calibration
// Master Ping - A Master now gets obsolete
// 
// 13/08/2019
// Updated the path storage, so that path won't be sored in two directions anymore
// Updated the pathfinder algorithm so that it adaps to the previous changes
// Updated Ping Function - every device now transmits randomly about the pats it has access to. Pings are not forewarded
// EEprom from ID's
// Memory for caibration values according to the ID
// 
// 14/08/2019
// Only one output String - Incomming String are forewarded to serial
// JSON Printing when it comes to readings sended to the client
// Reengineeried the message stamps for forewarding and receiving, so that now only the raw numbers are stampend, and not additional identifiers
// JSON Printing received reedings
// secured transmission trough sending messages multiple times. They will also be forewarded multiple times, but one the message was forewarded, the same message won't be forewarded again
// double received messages are only display once; Only if a message was forewarded, and therefore it has a different path, the message will be displayed again
// implemented asking for readings command
// issue with misinterpreting ot the to stamp fixes
// 
// 15.08.2019 - 22.08.2019
// Acknowledging received transmissions
// different transmission security typse like in LoRa Wan
// JSON fully working
// LoRa Protocoll reengineered
// Reengineered the Timeout Function for the path storage - only working once in an hour now
// Readings are now in transmission mode send in intervalls and in non transmission mode, on request
//
// 23.08.2019
// confirmation messages Ack
// sending hello message with sensor readings 
// Commands can be inserted as one line or via console
// Reassesd the serial mode with view on IoT
// Reassesed the Acknowledgement Messages, that messages are not not only send to the Origin SP, but if the origin SP is not the Sending SP, to the sending SP
// Reassesed the interpreter for Acknowledgement messages, so that an Acknowlegement is now only seen as one, it is directed to this SP, and not if the direction is another SP
// Reassesed the Ping Messages for more interpretion power: /h to /C /h
//
// 19.09.2016
// Advanced the Reference Value function
// Changed the Ping-Interval - new variable for the ping intervals, so that readings are send every 1 to 10min, and pings every 15 to 30 min
//
// 09.10.2018
// removed look_for_incoming_transmissions() because of an impplemented while loop in get_transmissions()
//
// 30.10.2018
// LoRa_String() funtion: Wait Funtion waits now the same amout of time that the transmission took
// clear_path() function: removes al path with a specific ID from the transmission memory
// LoRa_String() funtion: When no Ack was received after 5 attemps, the over ID is automatically cleared from the transmission memory, and a new Over ID is looked up with the same process the wirst over ID was found
// If there is no possibe Over ID anymore, no over ID will be added to the string, and any SP receiving the transmission will forward it
// Active and passive acknowldgements
//
// more secure transmission at security factor 2: while transmitting one message, all incoming messages are stored in a temporary memory and not forewarded
// after and passive or active acknowledgement was received, these messages will be forewarded
// Improved debug messages
// replaced the function that transmits transmissions received during transmission
// package_arrived has to be set to 1 one startup
//
// 14.11.2019
// Status Command: Implementing the complete command. Status messages will now work like Ping Messages that are transmitted by the Mesh. Status KEY of Statur Messages is "/S"
// Updating the JSON Printer: First ID in the Path Section is now the Origin ID, so that the Path array now always has one member more than the Millis, RSSI and Snr Value - This will make computing easier
// 
// 10.11.2023
// Now the code works for ESP32-WROOM-32
// The ADXL345 library was out of date, so now the commands have been adapted to a newer library
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// In Progress:
// Battry Status
// Automatet WiFi Scanning and Master Setting - Multiple Masters possible, masters directly send MQTT over FlexHab WiFi
// multiple device technologies - Problems with the libraries
// dual core - one for network and one for operations
// more than one sensor operation
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "Arduino.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#include <ADXL345.h>  // Old library, does not work
#include <EEPROM.h>
#include <ArduinoJson.h>

#include "esp_system.h"

#include <LoRa.h>                                // This library doesn't work anymore

#define BAND      868E6                          // you can set band here directly,e.g. 868E6,915E6
#define node_num  100                            // Number of nodes this node will communicate with in the Network

#define EEPROM_size 1                            // Size off the EEPROM Memory, there Master and Node ID are stored

// SX1276 LoRa module pins definitions, and colors of the respective soldered jumper cables
#define ss 13   // Gray
#define rst 4   // White
#define dio0 2  // Orange (pale)
#define SCK 18  // Purple
#define MISO 19 // Yellow
#define MOSI 23 // Orange (bright)
#define led 27

// Assign a unique ID to the sensor
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


int node_ID;                                     // Unique ID of this Device - if both are 0, this device is a master
int master_device;                               // ID of the Master

// Transmission Memory
String transmission_storage[node_num];           // recent transmissions
String network_paths[node_num * 2];              // storage for possible network paths
String last_transmission;                        // stored the last messages that was transmitted

// Temporary incoming transmittion storage
String temp_incoming_transmissions_message[100];
int temp_incoming_transmissions_over[100];
int temp_incoming_transmissions_to[100];

// Path Memory -- multiplying by 4 means that the code is designed to handle up to 4 paths for each node
int path_from[node_num * 4];                     // Start ID of a path
int path_over[node_num * 4];                     // ID of the Next ID in a path
int path_to[node_num * 4];                       // End ID of a path
int path_rssi[node_num * 4];                     // Mean of all RSSI Values of a path
int path_snr[node_num * 4];                      // Mean of all Snr Values in a path
int path_millis[node_num * 4];                   // Time when the path was updated last

// Setting the Interval Preverences
int interval_min      = 60000;                   // Data shall be send min. every 1 minute (5 seconds)
int interval_max      = 600000;                  // Data shall be send max. every 5 minutes (15 seconds)
int interval_ping_min = 60000 * 15;              // Ping be send min. every 15 Minutes (10 seconds)
int interval_ping_max = 60000 * 30;              // Ping be send max. every 30 Minutes(20 seconds)
int interval          = 0;                       // Initialising the Interval Variable
int interval_serial   = 0;                       // Interval for displaying the sensor readings on the serial monitor
int interval_ping     = 0;                       // Interval for PINGs
int lastsend          = 0;                       // Time when the last message was send
int lastsend_ping     = 0;                       // Time when the last PING was send
int lastprinted       = 0;                       // Time when the last message was serial printed
int package_arrived   = 1;                       // Used to track in Transmission mode 2 if a package arrived - Set to 0 when a transmission is in progress, otherwhise set to another value
int init_sp           = 0;                       // Used to send initial readings to the Master as a Hello Message

// Variables for measurement values and measurements
//accelerometry/gravity
double raw_readings[3];                          // Array for accelleration Values
double processed_readings[4] = {0, 0, 0, 0};     // Value of accelleration in the point
double num_measurements                = 0;      // Numer of measurements
int    extraordinary_event             = 0;      // Indicates if a moonquake was detected
double extraordinary_event_sensitivity = 0.25;   // Value for the sensitivyty of moonquake detection

// Sensor Calibration Variables
int    sensor_range  = 2;                         // Measuring Range of the Sensor
                                                  //  +- 2g for max accuracy
double reference_value = 1;                       // Local gravityity Level
                                                  // at the point of calibration - Here:1g
double ref_max[3];                                // Max sensor readings
double ref_min[3];                                // Min sensor readings
double offset[3];                                 // Offset values


// Package Settings
int serial_mode                  = 1;             // Seriel Mode: enabled(1) / disabled (0) - Shall sensor readings be displayed?
int debug_mode                   = 0;             // Debug Mode: enabled(1) / disabled (0)
int constant_transmission_mode   = 0;             // Shall sensor readins be transmitted in intervals? If not, they will be transmitted only on request
int display_mode                 = 1;             // Activate the display
String measurement_mode          = " A;";         // Do you want to get the accelleration value (A) or someting else?
int processing_mode              = 1;             // Shall the raw readings be processed?

//// Com Setting
//int LoRa_Mesh_Enable             = 1;
//int WiFi_Enable                  = 0;

// Reset Function for resetting the arduino wirelessly without the button
//void(* resetFunc) (void) = 0; //THIS DOESN'T WORK FOR ESP32! BECAUSE WHEN THE ESP32 RESETS IT DOESN'T JUMP
                              //TO THE RESET VECTOR DIRECTLY. INSTEAD, IT EXECUTE THE BOOTLOADER (position 0), WHICH
                              //THEN LOADS YOUR APPLICATION AND JUMPS TO ITS RESET VECTOR. INSTEAD WE CAN USE:
void resetFunc() {
  esp_restart();              //NOW IT'S MORE RELIABLE, SINCE IT WILL NOT RUN THE BOOTLOADER AGAIN, JUST RESET THE ESP
}

////////////////////////////////////////////////////////////////////////////////////
//  SetUp
////////////////////////////////////////////////////////////////////////////////////

void setup() {

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);                       // Initializing led as off

  Wire.begin(15, 14);
  Serial.begin(115200);                         // Initialize Serial Cummunication with a computer

  EEPROM.begin(EEPROM_size);                    // Initialize the EEPROM to get the Master and Node ID
  node_ID       = EEPROM.read(0);               // Unique ID of this Sensor Package
  master_device = EEPROM.read(1);               // ID of the Master

  
  init_calibration_value();                     // Initializing the Calibration_values

  
  if(!accel.begin())                            // There was a problem detecting the ADXL345... check your connections!
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  accel.setRange(ADXL345_RANGE_2_G); //To add: sensor_range dependency

  // Serial Monitor Initialization Message
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
  if (node_ID != master_device) {
    Serial.println("Gravimetric Sensor - Empowered");
  }
  else {
    Serial.println("Gravimetric Sensor Mesh - Gateway Empowered");
  }
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
  Serial.print("Node-ID: ");
  Serial.println(node_ID);
  Serial.print("Range: +-");
  Serial.print(sensor_range);
  Serial.println("g;");
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
  Serial.println("LANE - Lunar Advance Network");
  Serial.println("Version LANE.57"); //maybe change name?
  Serial.println("Developed by Jacopo Carra for lunar exploration (previous version by Jan-Henrik Zuenkler)");
  Serial.println("Spaceship EAC - European Astronaut Centre EAC - European Space Agency ESA");
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");

  print_settings();

  // Initializing and clearing the path arrays
  for (int i = 0; i < (node_num * 4); i++) {
    path_from[i]   = -1;
    path_over[i]   = -1;
    path_to[i]     = -1;
    path_rssi[i]   = -1;
    path_snr[i]    = -1;
    path_millis[i] = -1;
  }
  
  // Initializing and clearing the temporary transmission arrays
  for (int i = 0; i < 100; i++) {
    temp_incoming_transmissions_message[i] = "";
    temp_incoming_transmissions_over[i] = -1;
    temp_incoming_transmissions_to[i] = -1;
  }

  // Initializing SX1276 LoRa module
  SPI.begin(SCK, MISO, MOSI, ss);
  LoRa.setPins(ss, rst, dio0);
  LoRa.setSPIFrequency(1E6);
  if (!LoRa.begin(868E6 /*433E6 915E6*/))      // Frequency for LoRa in Europe is 868 MHz
  {
    Serial.println("Starting LoRa failed!");
    // while (1);
  }
}

////////////////////////////////////////////////////////////////////////////////////
//  Printing
////////////////////////////////////////////////////////////////////////////////////

// Function that prints out the current sensor readings to the serial monitor
void serial_print_readings() {

  if (serial_mode == 1) {

    if (measurement_mode != "") {
      if (node_ID == master_device) { 
        // Generationg the JSON Doument with the Data to be send
        StaticJsonDocument<512> print_object;
      
        print_object["ID"]   = node_ID;        // This devices unique Node-ID
        print_object["Time"] = millis();       // The Value of the Millis function in ms
  
        JsonArray Path   = print_object["Path"].createNestedArray("Path");
        JsonArray Millis = print_object["Path"].createNestedArray("Millis");
        JsonArray Rssi   = print_object["Path"].createNestedArray("Rssi");
        JsonArray Snr    = print_object["Path"].createNestedArray("Snr");
  
        if (measurement_mode.indexOf(" A;") != -1) {
  
          if (processing_mode == 1) {
            // Gravity Measurements
            print_object["Readings"]["G"]["Actual"]  = processed_readings[0];  // Actual Value
            print_object["Readings"]["G"]["Min"]     = processed_readings[1];  // Min Value
            print_object["Readings"]["G"]["Mean"]    = processed_readings[2];  // Mean Value
            print_object["Readings"]["G"]["Max"]     = processed_readings[3];  // Max Value
          }
  
          // Acceleration measurements
          print_object["Readings"]["A"]["X"]         = raw_readings[0];        // X Value
          print_object["Readings"]["A"]["Y"]         = raw_readings[1];        // Y Value
          print_object["Readings"]["A"]["Z"]         = raw_readings[2];        // Z Value
        }
  
        // Transform the JSON Document to a String
        String print_string;
        serializeJson(print_object, print_string);

        Serial.println(print_string);

        // clearing variables
        // normally in Sensor Packages this variables are cleared after the transmission
        // In the case that the Device is a Master, it does not transmit it's readings, but it does print it's readings over Serial in JSON here, to the Variables have to be cleared here!
        num_measurements      = 0;
        raw_readings[0]       = 0;
        raw_readings[1]       = 0;
        raw_readings[2]       = 0;
        processed_readings[0] = 0;
        processed_readings[1] = 0;
        processed_readings[2] = 0;
        processed_readings[3] = 0;
      }
      else {
        Serial.print(String(node_ID) + " " + String(millis()));
  
        if (processing_mode == 1) {
          Serial.print(" /G " + String(processed_readings[0]) + " " + String(processed_readings[1]) + " " + String(processed_readings[2]) + " " + String(processed_readings[3]));
        }
  
        Serial.println(" /A " + String(raw_readings[0]) + " " + String(raw_readings[1]) + " " + String(raw_readings[2]));
      }
    }
  }
}

void print_as_JSON(String transmission_incomming) {
  
  // Generationg the JSON Doument with the Data to be send
  StaticJsonDocument<512> print_object;

  // ID and Millis of the Message
  print_object["ID"]   = get_part(transmission_incomming, 1, 2).toInt(); // This device's unique Node-ID
  print_object["Time"] = get_part(transmission_incomming, 2, 3).toInt(); // The Value of the Millis function, in ms

  // Path of the Message
  JsonArray Path       = print_object["Path"].createNestedArray("Path");
  JsonArray Millis     = print_object["Path"].createNestedArray("Millis");
  JsonArray Rssi       = print_object["Path"].createNestedArray("Rssi");
  JsonArray Snr        = print_object["Path"].createNestedArray("Snr");

  // Processing the Path
  String temp_string = get_substring(transmission_incomming, "ID", " Time");
  
  // Remove "ID "
  temp_string.remove(0,3);

  // First add the origin ID and millis
  print_object["Path"]["Path"].add(get_part(transmission_incomming, 1, 2).toInt());
  print_object["Path"]["Millis"].add(get_part(transmission_incomming, 2, 3).toInt());
  
  int i = 0;
  
  while (get_part(temp_string, i, i + 1) != "") {
    print_object["Path"]["Path"].add(get_part(temp_string, i, i + 1).toInt());
    i = i + 1;
  }

  // Processing the Millis Values
  temp_string = get_substring(transmission_incomming, "Time", " Rssi");
  
  // Remove "Time "
  temp_string.remove(0,5);
  
  i = 0;
  
  while (get_part(temp_string, i, i + 1) != "") {
    print_object["Path"]["Millis"].add(get_part(temp_string, i, i + 1).toInt());
    
    i = i + 1;
  }

  // Processing the Rssi Values
  temp_string = get_substring(transmission_incomming, "Rssi", " Snr");
  
  // Remove "Rssi "
  temp_string.remove(0,5);
  
  i = 0;
  
  while (get_part(temp_string, i, i + 1) != "") {
    print_object["Path"]["Rssi"].add(get_part(temp_string, i, i + 1).toDouble());
    
    i = i + 1;
  }

  // Processing the Snr Values
  temp_string = get_substring(transmission_incomming, "Snr", " ");
  
  // Remove "Snr "
  temp_string.remove(0,4);
  
  i = 0;
  
  while (get_part(temp_string, i, i + 1) != "") {
    print_object["Path"]["Snr"].add(get_part(temp_string, i, i + 1).toDouble());
    
    i = i + 1;
  }
  
  
  if (transmission_incomming.indexOf("/R") != -1) { // Message contains readings - print in JSON
    // Process the readings
    if (transmission_incomming.indexOf("/A") != -1) { // Message contains Accelerometer Data
        
        temp_string = get_substring(transmission_incomming, "/A", " "); // Get the Readings String
        temp_string.remove(0,3); // remove /A

        // Acceleration measurements
        print_object["Readings"]["A"]["X"] = get_part(temp_string, 0, 1).toDouble(); // X Value
        print_object["Readings"]["A"]["Y"] = get_part(temp_string, 1, 2).toDouble(); // Y Value
        print_object["Readings"]["A"]["Z"] = get_part(temp_string, 2, 3).toDouble(); // Z Value
    }
    
    if (transmission_incomming.indexOf("/G") != -1) { // Message contains Gravimetric Data
        
        temp_string = get_substring(transmission_incomming, "/G", " ");                   // Get the Readings String
        temp_string.remove(0,3);                                                          // remove /G

        // Gravimetric measurements
        print_object["Readings"]["G"]["Actual"] = get_part(temp_string, 0, 1).toDouble(); // Actual Value
        print_object["Readings"]["G"]["Min"]    = get_part(temp_string, 1, 2).toDouble(); // Min Value
        print_object["Readings"]["G"]["Mean"]   = get_part(temp_string, 2, 3).toDouble(); // Mean Value
        print_object["Readings"]["G"]["Max"]    = get_part(temp_string, 3, 4).toDouble(); // Max Value
    }
    
  }
  else if (transmission_incomming.indexOf("/C /h") != -1) { // Message is a ping message
    String message_sender = get_substring(transmission_incomming, "/n ID", " ");
    message_sender.remove(0, 6);
    message_sender = get_part(message_sender, 0 , 1);
    print_object["Ping"]["Sender"] = message_sender.toInt();
  }
  else if (transmission_incomming.indexOf("/S") != -1) { // Message is a Status message
    String message_sender = get_substring(transmission_incomming, "/n ID", " ");
    message_sender.remove(0, 6);
    message_sender = get_part(message_sender, 0 , 1);
    print_object["Status"]["Sender"] = message_sender.toInt();
  }
  else if (transmission_incomming.indexOf("/C") != -1) { // Message is a command message
    temp_string = get_substring(transmission_incomming, "/C", "/n"); // Get the Command String
    temp_string.remove(0,3); // remove /C
    
    print_object["Command"] = temp_string;
  }
  else if (transmission_incomming.indexOf("/M") != -1) { // Message is a chat message
    temp_string = get_substring(transmission_incomming, "/M", "/n"); // Get the chat String
    temp_string.remove(0,3); // remove /C
    
    print_object["Message"] = temp_string;
  }
  

  // Transform the JSON Document to a String
  transmission_incomming = "";
  serializeJson(print_object, transmission_incomming);

  if (serial_mode == 1) {
    Serial.println(transmission_incomming);
  }
}


void print_settings() {
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
  Serial.println("Sensor Package Settings");
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");

  Serial.print(" - Serial Mode: ");
  if (serial_mode == 1) {
    Serial.println("Enabled");
  }
  else {
    Serial.println("Disabled");
  }

  Serial.print(" - Debug Mode: ");
  if (debug_mode == 1) {
    Serial.println("Enabled");
  }
  else {
    Serial.println("Disabled");
  }

  Serial.print(" - Transmission Mode: ");
  if (constant_transmission_mode == 1) {
    Serial.println("Enabled");
  }
  else {
    Serial.println("Disabled");
  }

  Serial.print(" - Display Mode: ");
  if (display_mode == 1) {
    Serial.println("Enabled");
  }
  else {
    Serial.println("Disabled");
  }

  Serial.print(" - Measurement Mode:");
  Serial.println(measurement_mode);

  Serial.print(" - Processing Mode: ");
  if (processing_mode == 1) {
    Serial.println("Enabled");
  }
  else {
    Serial.println("Disabled - Raw Data");
  }

  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
  Serial.println("- Commands: Serial Mode; Debug Mode; Transmission Mode; Processing Mode;");
  Serial.println("            Display Mode; Calibration; Status; Chat; Reset; Remote");
  Serial.println(":::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::");
}


////////////////////////////////////////////////////////////////////////////////////
//  Serial Console
////////////////////////////////////////////////////////////////////////////////////

// Subprogramm to wait until a cammand is inserted in the seriel monitor
// Waits until manual input is available
String request_serial_input() {
  while (!Serial.available()) {}
  return read_serial_input();
}

// Subprogramm to read incomming commands
// If there are Messages / commands comming in, the sub program gives them out
// if there is no message / command comming in, the output is ""
String read_serial_input() {

  // Variable for command from seriel monitor
  String com = "";

  // Read every char from the seriel input
  while (Serial.available()) {
    com = com + String(char(Serial.read()));
    delay(10);
  }

  return com;
}

// Read  and execute commands
void excecute_serial_commands(String com) {

  // Commands from the Serial Monitor
  if (com == "") {
    com = read_serial_input();
  }

  if (com.startsWith("Serial Mode") || com.startsWith("serial mode")) {

    com.remove(0,12);
    
    // Print out the actual status
    if (debug_mode == 1) {
      Serial.println();
      Serial.print("Serial Mode: ");
  
      if (serial_mode == 1) {
        Serial.println("Enabled");
      }
      else {
        Serial.println("Disabled");
      }
      Serial.println();
  
      Serial.print("Change to: ");
    }

    // Type in the change command
    if (com == "") {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // change the settings acording to the typed in command
    if (com == "on" || com == "On" || com == "Enabled" || com == "enabled") {
      serial_mode = 1;
    }
    else if (com == "off" || com == "Off" || com == "Disabled" || com == "disabled") {
      serial_mode = 0;
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }

    if (debug_mode == 1) {
      Serial.println("");
    }

  }
  else if (com.startsWith("Debug Mode") || com.startsWith("debug mode")) {

    com.remove(0,11);
    
    // Print out the actual status
    if (debug_mode == 1) {
      Serial.println();
      Serial.print("Debug Mode: ");
  
      if (debug_mode == 1) {
        Serial.println("Enabled");
      }
      else {
        Serial.println("Disabled");
      }
      Serial.println();
  
      Serial.print("Change to: ");
    }

    // Type in the change command
    if (com == "" ) {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // change the settings acording to the typed in command
    if (com == "on" || com == "On" || com == "Enabled" || com == "enabled") {
      debug_mode = 1;
    }
    else if (com == "off" || com == "Off" || com == "Disabled" || com == "disabled") {
      debug_mode = 0;
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }

    if (debug_mode == 1) {
      Serial.println("");
    }

  }
  else if (com.startsWith("Transmission Mode") || com.startsWith("transmission mode")) {

    com.remove(0, 18);
    
    // Print out the actual status
    if (debug_mode == 1) {
      Serial.println();
      Serial.print("Transmission Mode: ");
  
      if (constant_transmission_mode == 1) {
        Serial.println("Enabled");
      }
      else {
        Serial.println("Disabled");
      }
      Serial.println();
  
      Serial.print("Chage to: ");
    }

    // Type in the change command
    if (com == "" ) {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // change the settings acording to the typed in command
    if (com == "on" || com == "On" || com == "Enabled" || com == "enabled") {
      constant_transmission_mode = 1;
    }
    else if (com == "off" || com == "Off" || com == "Disabled" || com == "disabled") {
      constant_transmission_mode = 0;
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }

    if (debug_mode == 1) {
      Serial.println("");
    }

  }
  else if (com.startsWith("Processing Mode") || com.startsWith("processing mode")) {

    com.remove(0, 16);
    
    // Print out the actual status
    if (debug_mode == 1) {
      Serial.println();
      Serial.print("Processing Mode: ");
  
      if (processing_mode == 1) {
        Serial.println("Enabled");
      }
      else {
        Serial.println("Disabled");
      }
      Serial.println();
  
      Serial.print("Chage to: ");
    }

    // Type in the change command
    if (com == "") {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // change the settings acording to the typed in command
    if (com == "on" || com == "On" || com == "Enabled" || com == "enabled") {
      processing_mode = 1;
    }
    else if (com == "off" || com == "Off" || com == "Disabled" || com == "disabled") {
      processing_mode = 0;
    }
    else {
      
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }

    if (debug_mode == 1) {
      Serial.println("");
    }

  }
  else if (com.startsWith("Display Mode") || com.startsWith("display mode")) {

    com.remove(0, 13);
    
    // Print out the actual status
    if (debug_mode == 1) {
      Serial.println();
      Serial.print("Display Mode: ");
  
      if (display_mode == 1) {
        Serial.println("Enabled");
      }
      else {
        Serial.println("Disabled");
      }
      Serial.println();
  
      Serial.print("Chage to: ");
    }

    // Type in the change command
    if (com == "") {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // change the settings acording to the typed in command
    if (com == "on" || com == "On" || com == "Enabled" || com == "enabled") {
      display_mode = 1;
    }
    else if (com == "off" || com == "Off" || com == "Disabled" || com == "disabled") {
      display_mode = 0;
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }

  }
  else if (com.startsWith("Calibration") || com.startsWith("calibration")) {

    com.remove(0, 12);

    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Calibration Mode: ");
      Serial.println();
    }

    int calibration = 1;

    if (debug_mode == 1) {
      Serial.println("Sensor Readings: ");
      Serial.println();
    }

    for (int i = 0; i < 10; i++) {
      get_sensor_readings();
      pre_process_sensor_readings();
      process_sensor_readings();
      
      serial_print_readings();
    }

    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Raw Sensor Readings: ");
      Serial.println();
    }

    processing_mode = 0;

    for (int i = 0; i < 10; i++) {
      get_sensor_readings();
      serial_print_readings();
    }

    processing_mode = 1;
    
    if (debug_mode == 1) {
      Serial.println("");
      Serial.print("Axis to calibrate (X/Y/Z): ");
    }

    // Type in the change command
    if (com == "") {
      com = request_serial_input();
    }

    if (com.startsWith("x") || com.startsWith("X") || com.startsWith("y") || com.startsWith("Y") || com.startsWith("Z") || com.startsWith("Z")) {

      char axis;

      if (com.startsWith("x") || com.startsWith("X")) {
        axis = 'x';
      }
      else if (com.startsWith("y") || com.startsWith("Y")) {
        axis = 'y';
      }
      else if (com.startsWith("z") || com.startsWith("Z")) {
        axis = 'z';
      }

      com.remove(0,2);

      if (debug_mode == 1) {
        Serial.println(axis);
        Serial.println("");
  
        Serial.print("Refs-Value (Min/Max/Offset): ");
      }

      String axis_ref;
      if (com == "") {
        axis_ref = request_serial_input();
      }
      else {
        if (com.startsWith("Min") || com.startsWith("min")) {
          axis_ref = "min";

          com.remove(0, 4);
        }
        else if (com.startsWith("Max") || com.startsWith("max")) {
          axis_ref = "max";

          com.remove(0, 4);
        }
        else if (com.startsWith("Offset") || com.startsWith("Offset")) {
          axis_ref = "Offset";

          com.remove(0, 7);
        }
      }

      if (axis_ref == "Min" || axis_ref == "min" || axis_ref == "Max" || axis_ref == "max" || axis_ref == "Offset" || axis_ref == "offset") {

        if (axis_ref == "Min") {
          axis_ref == "min";
        }
        else if (axis_ref == "Max") {
          axis_ref == "max";
        }
        else if (axis_ref == "Offset") {
          axis_ref == "offset";
        }

        if (debug_mode == 1) {
          Serial.println(axis_ref);
          Serial.println("");
  
          Serial.print("New Value: ");
        }

        // Type in the change command
        if (com == "") {
          com = request_serial_input();
        }
        else {
          com = get_part(com, 0, 1);
        }

        double new_value = double(com.toFloat());

        if (debug_mode == 1) {
          Serial.println(new_value);
          Serial.println("");
  
          Serial.println("");
          Serial.println("Value updated");
          Serial.println("");
        }

        if (axis == 'x') {
          if (axis_ref == "min") {
            ref_min[0] = new_value;
          }
          else if (axis_ref == "max") {
            ref_max[0] = new_value;
          }
          else if (axis_ref == "offset") {
            offset[0] = new_value;
          }
        }
        else if (axis == 'y') {
          if (axis_ref == "min") {
            ref_min[1] = new_value;
          }
          else if (axis_ref == "max") {
            ref_max[1] = new_value;
          }
          else if (axis_ref == "offset") {
            offset[1] = new_value;
          }
        }
        else if (axis == 'z') {
          if (axis_ref == "min") {
            ref_min[2] = new_value;
          }
          else if (axis_ref == "max") {
            ref_max[2] = new_value;
          }
          else if (axis_ref == "offset") {
            offset[2] = new_value;
          }
        }

        for (int i = 0; i < 10; i++) {
          get_sensor_readings();
          serial_print_readings();
        }

      }
      else {
        if (debug_mode == 1) {
          Serial.println("Value not supported");
        }
      }
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Value not supported");
      }
    }

  }
  else if (com.startsWith("Status") || com.startsWith("status")) {

    com.remove(0, 7);

    if (debug_mode == 1) {
      Serial.println("");
      Serial.println("Sensor Package " + String(node_ID) + " Status:");
      Serial.println("");
    }

    print_settings();

    
    Serial.println("Stored Transmissions");
    Serial.println("");
    
    for (int i = 0; i < (node_num * 4); i++) {
      if (transmission_storage[i] != "") {
       Serial.println(String(i) + ": " + transmission_storage[i]);
      }
      else {
        break;
      }
    }
    
    Serial.println("");
    Serial.println("Stored Paths");
    Serial.println("");

    for (int i = 0; i < (node_num * 4); i++) {
      if (network_paths[i] != "") {
        Serial.println(String(i) + ": " + network_paths[i]);
      }
      else {
        break;
      }
    }

    Serial.println("");
    Serial.println("Network Status");
    Serial.println("Observed Paths in the Network:");

    for (int i = 0; i < (node_num * 4); i++) {
      if (path_from[i] != -1) {
        Serial.print("[ From " + String(path_from[i]));
        
        if (path_over[i] != -1) {
          Serial.print(" Over " + String(path_over[i]));
        }
        else {
          Serial.print(" Over -");
        }
        
        Serial.println(" To " + String(path_to[i]) + " Millis " + String(path_millis[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) + " ]");
      }
      else {
        break;
      }
    }

    Serial.println("");

  }
  else if (com.startsWith("Chat") || com.startsWith("chat")) {

    com.remove(0, 5);

    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Cave Chat initialized - the cavest way to chat");
      Serial.println("Welcome to the Cave Chat");
      Serial.println();
  
      Serial.print("To Node: ");
    }

    int receiver_node;
    
    if (com == "") {
      receiver_node = request_serial_input().toInt();
    }
    else {
      receiver_node = get_part(com, 0, 1).toInt();
    }

    com.remove(0, get_part(com, 0, 1).length() + 1);

    if (debug_mode == 1) {
      Serial.println(String(receiver_node));
      Serial.print("Message: ");
    }

    if (com == "") {
      com = request_serial_input();
    }

    if (debug_mode == 1) {
      Serial.println(com);
    }

    // Generating the Transmission String
    com = "/M " + com;
    com = generate_transmission_string(com);

    // transmit the readings with LoRa
    LoRa_string(com, receiver_node, 2);

    // store transmission
    com = get_substring(com, "SP", "/n");
    com.remove(0, 3);
    store_to_transmission_storage(com);

    if (debug_mode == 1) {
      Serial.println("transmission complete!");
  
      Serial.println();
    }
  }
  else if (com.startsWith("SOS") || com.startsWith("Sos") || com.startsWith("sOs") || com.startsWith("soS") || com.startsWith("SOs") || com.startsWith("SoS") || com.startsWith("sOS") || com.startsWith("sos")) {
    String transmission_string = transmission_string + "/C SOS";

    display_mode = 0;

    transmission_string = generate_transmission_string(transmission_string);
    
    LoRa_string(transmission_string, -1, 2);
  }
  else if (com.startsWith("Remote") || com.startsWith("remote")) {

    com.remove(0, 7);
    
    String transmission_string = "/C";

    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Sensor Package remote Setting");
      Serial.println();
  
      Serial.print("To Node: ");
    }

    if (com == "") {
      com = request_serial_input();
    }

    int receiver_node = get_part(com, 0, 1).toInt();

    com.remove(0, get_part(com, 0, 1).length() + 1);
    
    if (receiver_node != 0 || com.startsWith("0")) {

      if (debug_mode == 1) {
        Serial.println(receiver_node);
  
        Serial.println("Implemented Commands: transmit_readings; constant_transmission_mode; processing_mode; display_mode; status; calibration; reset;");
        Serial.print("Command: ");
      }

      if (com == "") {
        com = request_serial_input();
      }

      if (debug_mode == 1) {
        Serial.println(com);
      }

      if (com.startsWith("transmit_readings")) {
        transmission_string = transmission_string + " TR";
      }
      else if (com.startsWith("constant_transmission_mode")) {

        com. remove(0, 18);
        
        if (debug_mode == 1) {
          Serial.print("Chage to: ");
        }

        // Type in the change command
        if (com == "") {
          com = request_serial_input();
        }

        if (com.startsWith("on") || com.startsWith("On") || com.startsWith("Enabled") || com.startsWith("enabled")) {
          if (debug_mode == 1) {
            Serial.println(com);
          }

          transmission_string = transmission_string + " TM ON";
        }
        else if (com.startsWith("off") || com.startsWith("Off") || com.startsWith("Disabled") || com.startsWith("disabled")) {
          if (debug_mode == 1) {
            Serial.println(com);
          }

          transmission_string = transmission_string + " TM OFF";
        }
        else {
          if (debug_mode == 1) {
            Serial.println("Command not supported");
          }
        }
      }
      else if (com.startsWith("processing_mode")) {

        com.remove(0, 16);
        
        if (debug_mode == 1) {
          Serial.print("Chage to: ");
        }

        // Type in the change command
        if (com =="") {
          com = request_serial_input();
        }

        if (com.startsWith("on") || com.startsWith("On") || com.startsWith("Enabled") || com.startsWith("enabled")) {
          if (debug_mode == 1) {
            Serial.println(com);
          }

          transmission_string = transmission_string + " PM ON";
        }
        else if (com.startsWith("off") || com.startsWith("Off") || com.startsWith("Disabled") || com.startsWith("disabled")) {
          if (debug_mode == 1) {
            Serial.println(com);
          }

          transmission_string = transmission_string + " PM OFF";
        }
        else {
          if (debug_mode == 1) {
            Serial.println("Command not supported");
          }
        }
      }
      else if (com.startsWith("status")) {
        transmission_string = transmission_string + " ST";
      }
      else if (com.startsWith("calibration")) {

        com.remove(0, 12);
        
        transmission_string = transmission_string + " CB";

        if (debug_mode == 1) {
          Serial.println();
          Serial.println("Calibration Mode: ");
          Serial.println();
  
          Serial.println("");
          Serial.print("Axis to calibrate (X/Y/Z): ");
        }
    
        // Type in the change command
        if (com =="") {
          com = request_serial_input();
        }
    
        if (com.startsWith("x") || com.startsWith("X") || com.startsWith("y") || com.startsWith("Y") || com.startsWith("Z") || com.startsWith("Z")) {
    
          char axis;
    
          if (com.startsWith("x") || com.startsWith("X")) {
            axis = 'x';
          }
          else if (com.startsWith("y") || com.startsWith("Y")) {
            axis = 'y';
          }
          else if (com.startsWith("z") || com.startsWith("Z")) {
            axis = 'z';
          }

          if (debug_mode == 1) {
            Serial.println(axis);
            Serial.println("");
          }

          com.remove(0, 1);
          
          if (debug_mode == 1) {
            Serial.print("Refs-Value (Min/Max/Offset): ");
          }
    
          String axis_ref;
          if (com == "") {
            axis_ref = request_serial_input();
          }
          else {
            axis_ref = get_part(com, 0, 1);
          }

          com.remove(0, get_part(com, 0, 1).length() + 1);
          
          if (axis_ref == "Min" || axis_ref == "min" || axis_ref == "Max" || axis_ref == "max" || axis_ref == "Offset" || axis_ref == "offset") {
    
            if (axis_ref == "Min") {
              axis_ref == "min";
            }
            else if (axis_ref == "Max") {
              axis_ref == "max";
            }
            else if (axis_ref == "Offset") {
              axis_ref == "offset";
            }

            if (debug_mode == 1) {
              Serial.println(axis_ref);
              Serial.println("");
      
              Serial.print("New Value: ");
            }
    
            // Type in the change command
            double new_value;
            
            if (com == "") {
              com = request_serial_input();
              
              new_value = double(com.toFloat());
            }
            else {
              new_value = double(get_part(com, 0, 1).toFloat());
            }
            
            if (debug_mode == 1) {
              Serial.println(new_value);
              Serial.println("");
      
              Serial.println("");
              Serial.println("Value updated");
              Serial.println("");
            }
    
            if (axis == 'x') {
              if (axis_ref == "min") {
                transmission_string = transmission_string + " X min " + String(new_value);
              }
              else if (axis_ref == "max") {
                transmission_string = transmission_string + " X max " + String(new_value);
              }
              else if (axis_ref == "offset") {
                transmission_string = transmission_string + " X offset " + String(new_value);
              }
            }
            else if (axis == 'y') {
              if (axis_ref == "min") {
                transmission_string = transmission_string + " Y min " + String(new_value);
              }
              else if (axis_ref == "max") {
                transmission_string = transmission_string + " Y max " + String(new_value);
              }
              else if (axis_ref == "offset") {
                transmission_string = transmission_string + " Y offset " + String(new_value);
              }
            }
            else if (axis == 'z') {
              if (axis_ref == "min") {
                transmission_string = transmission_string + " Z min " + String(new_value);
              }
              else if (axis_ref == "max") {
                transmission_string = transmission_string + " Z max " + String(new_value);
              }
              else if (axis_ref == "offset") {
                transmission_string = transmission_string + " Z offset " + String(new_value);
              }
            }
    
            for (int i = 0; i < 10; i++) {
              get_sensor_readings();
              serial_print_readings();
            }
    
          }
          else {
            if (debug_mode == 1) {
              Serial.println("Value not supported");
            }
          }
        }
        else {
          if (debug_mode == 1) {
            Serial.println("Value not supported");
          }
        }
      }
      else if (com.startsWith("reset") || com.startsWith("rst")) {
        transmission_string = transmission_string + " RST";
      }
      else {
        if (debug_mode == 1) {
          Serial.println("Command not supported");
          Serial.println("Implemented Commands: constant_transmission_mode; processing_mode; status; calibration; reset;");
        }
      }

      // If the String was filled before, send it and than store it
      if (transmission_string != "/C") {
        transmission_string = generate_transmission_string(transmission_string);
        LoRa_string(transmission_string, receiver_node, 2);
        
        if (debug_mode == 1) {
          Serial.println("Transmission Send!");
        }

        // Store the transmission in the transmission array
        // Shorten the String
        transmission_string = get_substring(transmission_string, "SP ", "/n");
        transmission_string.remove(0, 3);
        
        // Store it in the Array
        store_to_transmission_storage(transmission_string);
      }
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Unable to transmit - input is not an integer");
      }
    }
  }
  else if (com == "reset" || com == "Reset") {
    if (debug_mode == 1) {
      Serial.println("");
      Serial.println("-----------------------------------");
      Serial.println("-------------Resetting-------------");
      Serial.println("-----------------------------------");
      Serial.println("");
    }

    resetFunc();
  }
  else if (com == "") {

  }
  else {
    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Command not supported!");
      Serial.println();
  
      print_settings();
  
      Serial.println("");
    }

  }

}

////////////////////////////////////////////////////////////////////////////////////
//  Sensor Readings - Gravimetry
////////////////////////////////////////////////////////////////////////////////////

// Function for setting the Calibration Values initially
void init_calibration_value() {
  if (node_ID == 0) {
    ref_max[0] = +0.960;   // Max sensor readings X
    ref_max[1] = +0.980;   // Max sensor readings Y
    ref_max[2] = +0.870;   // Max sensor readings Z
    ref_min[0] = -0.960;   // Min sensor readings X
    ref_min[1] = -0.960;   // Min sensor readings Y
    ref_min[2] = -0.870;   // Min sensor readings Z
    offset[0]  = +0.000;   // Offset values X
    offset[1]  = -0.020;   // Offset values Y
    offset[2]  = +0.000;   // Offset values Z
  } 
  else if (node_ID == 1) {
    ref_max[0] = +0.960;   // Max sensor readings X
    ref_max[1] = +0.930;   // Max sensor readings Y
    ref_max[2] = +0.980;   // Max sensor readings Z
    ref_min[0] = -0.910;   // Min sensor readings X
    ref_min[1] = -0.940;   // Min sensor readings Y
    ref_min[2] = -0.810;   // Min sensor readings Z
    offset[0]  = -0.020;   // Offset values X
    offset[1]  = +0.010;   // Offset values Y
    offset[2]  = -0.020;   // Offset values Z
  }
  else if (node_ID == 2) {
    ref_max[0] = +0.900;   // Max sensor readings X
    ref_max[1] = +0.920;   // Max sensor readings Y
    ref_max[2] = +0.820;   // Max sensor readings Z
    ref_min[0] = -0.960;   // Min sensor readings X
    ref_min[1] = -0.920;   // Min sensor readings Y
    ref_min[2] = -0.830;   // Min sensor readings Z
    offset[0]  = +0.030;   // Offset values X
    offset[1]  = -0.020;   // Offset values Y
    offset[2]  = -0.080;   // Offset values Z
  }
  else { // If there are no Values for a specific Sensor, yet!
         // The reference values are set in a way so that the processing will not change the redings
    reference_value = +1.000;   // reset the reference value
    ref_max[0]      = +1.000;   // Max sensor readings X
    ref_max[1]      = +1.000;   // Max sensor readings Y
    ref_max[2]      = +1.000;   // Max sensor readings Z
    ref_min[0]      = +1.000;   // Min sensor readings X
    ref_min[1]      = +1.000;   // Min sensor readings Y
    ref_min[2]      = +1.000;   // Min sensor readings Z
    offset[0]       = +0.000;   // Offset values X
    offset[1]       = +0.000;   // Offset values Y
    offset[2]       = +0.000;   // Offset values Z
  } 

  if (debug_mode == 1) {
    Serial.println();
    Serial.println("Accelerometer Calibration complete!");
    Serial.println();
  }
}

// Function that reads out the sensor readings
void get_sensor_readings() {
  // Aquire sensor readings
  if (measurement_mode.indexOf(" A;") != -1) {
    sensors_event_t event; 
    accel.getEvent(&event);
    raw_readings[0] = event.acceleration.x;
    raw_readings[1] = event.acceleration.y;
    raw_readings[2] = event.acceleration.z;
  }

  // Calculate the number of actual measurement values
  num_measurements = num_measurements + 1;

}

// Function that processes the Sensor readings
void pre_process_sensor_readings() {

  if (processing_mode == 1) {
    if (measurement_mode.indexOf(" A;") != -1) {
      raw_readings[0] = raw_readings[0] * ((2 * reference_value) / (sqrt(sq(ref_min[0])) + sqrt(sq(ref_max[0])))) + offset[0]; // X-Axis: Processiong the sensor reading with the calibration Values
      raw_readings[1] = raw_readings[1] * ((2 * reference_value) / (sqrt(sq(ref_min[1])) + sqrt(sq(ref_max[1])))) + offset[1]; // Y-Axis: Processiong the sensor reading with the calibration Values
      raw_readings[2] = raw_readings[2] * ((2 * reference_value) / (sqrt(sq(ref_min[2])) + sqrt(sq(ref_max[2])))) + offset[2]; // Z-Axis: Processiong the sensor reading with the calibration Values

    }
  }
}

// Funtion that calculates the gravity values
void process_sensor_readings() {
  if (processing_mode == 1) {
    if (measurement_mode.indexOf(" A;") != -1) {
      processed_readings[0] = sqrt(sq(raw_readings[0]) + sq(raw_readings[1]) + sq(raw_readings[2]));  //this looks like
                                                                                                      //a way to detect
                                                                                                      //moonquakes
      //Serial.println(raw_readings[0]);
      //delay(500);
  
      processed_readings[2] = ((processed_readings[2] * (num_measurements - 1)) + processed_readings[0]) / (num_measurements);
      // This is the mean
  
      // Initializing the min/max values
      if (num_measurements == 1) {
        processed_readings[1] = processed_readings[0];  // This is the min
        processed_readings[3] = processed_readings[0];  // This is the max
      }
  
      // Updating the min/max value
      if (processed_readings[0] < processed_readings[1]) { // Updating the min value
        processed_readings[1] = processed_readings[0];
      }
      else if (processed_readings[0] > processed_readings[3]) { // Updating the max value
        processed_readings[3] = processed_readings[0];
      }
    }
  }
}

// Detection of unnormal events - here: moonquakes
void detect_events() {
  if (measurement_mode.indexOf(" A;") != -1) {
    // If the current gravity value is lower or higher than a specific amount, the reading will be sent immediately
    if (processed_readings[0] < processed_readings[2] * extraordinary_event_sensitivity || processed_readings[0] > processed_readings[2] * (extraordinary_event_sensitivity + 1)) {
    // If the actual acceleration is lower or higher than the average multiplied by a constant
      if (extraordinary_event != -1) {
        extraordinary_event = 1;
        Serial.println("Moonquake detected!");  // Remove this if you only want to receive this message in Debugging Mode
        if (debug_mode == 1) {                  // Message during Debugging Mode
          Serial.println("Moonquake detected!");
          Serial.println("");
        }

        delay(200);
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////
//  Communication Technology
////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//  Outgoing Transmissions
////////////////////////////////////////////////////////////////////////////////////

void manage_outgoing_transmissions() {
  if (constant_transmission_mode == 1 || extraordinary_event == 1) {
    // Sending Sensor readings
    if (millis() - lastsend > interval) {

      // Calling the Transmit Readings function to generate the transmission string and send it via LoRa
      if (node_ID != master_device) { // Only LoRa the readings if the device is not a Master
        transmit_readings(master_device);
      }

      // Setting the Sending Variables
      lastsend = millis();
      interval = random(interval_min, interval_max);
    }
    else if (extraordinary_event == 1) {

      // Calling the Transmit Readings function to generate the transmission string and send it via LoRa
      if (node_ID != master_device) { // Only LoRa the readings if the device is not a Master
        transmit_readings(master_device);
      }

      // Setting the Sending Variables
      lastsend = millis();
      interval = random(interval_min, interval_max);
    }
  }
  else if (init_sp == 0 && millis() > (60 * 1000)) { // Send first readings to the Master as a Hello Message
    init_sp = 1;                                     // This is only performed once, 1 minute ater the initialization
    
    // Calling the Transmit Readings function to generate the transmission string and send it via LoRa
    if (node_ID != master_device) { // Only LoRa the readings if the device is not a Master
      transmit_readings(master_device);
    }
  }
  else {
   if (debug_mode == 1) {
     Serial.println("Transmission Mode: Disabled");
   }
  }
}

void transmit_ping() {  
  if (millis() - lastsend_ping > interval_ping) {
      
    // Sending a ping message for every known paths to every device in transmission range
    for (int i = 0; i < (node_num * 4); i++) {
      if (path_from[i] != -1 && path_to[i] != -1) {
        if (path_from[i] == node_ID || path_to[i] == node_ID) {
          // Ping Messages simulate a message without payload coming from a surrounding SP, so that other sorrounding SP's know the surrounding of this SP
          String transmission_string = String(path_millis[i]) + " /C /h/n";
         
          if (path_from[i] == node_ID) {
            transmission_string = String(path_to[i]) + " " + transmission_string;
          }
          else if (path_to[i] == node_ID) {
            transmission_string = String(path_from[i]) + " " + transmission_string;
          }
            
          transmission_string = "SP " + transmission_string + " ID " + String(node_ID) + " Time " + String(millis()) + " Rssi " + String(path_rssi[i]) + " Snr " + String(path_snr[i]);
  
          // Transmit the readings with LoRa
          LoRa_string(transmission_string, -1, 1);
  
          // Store the transmission in the transmission array
          // Shorten the String
          transmission_string = get_substring(transmission_string, "SP ", "/n");
          transmission_string.remove(0, 3);
        
          // Store it in the Array
          store_to_transmission_storage(transmission_string);
        }
      }
    }
    
  // Setting the Sending Variables
  lastsend_ping = millis();
  interval_ping = random(interval_ping_min, interval_ping_max);
      
  }
}

void transmit_readings(int receiver_node) {

  // Generationg the String with the Data to be send
  String transmission_string;

  // Enhence the Transmission_String with measurement-values
  if (measurement_mode.indexOf(" A;") != -1) {
    
    transmission_string = transmission_string + "/R ";
        
    if (processing_mode == 1) {
      transmission_string = transmission_string + "/G " + String(processed_readings[0], 6) + " " + String(processed_readings[1], 6) + " " + String(processed_readings[2], 6)  + " " + String(processed_readings[3], 6);  // Gravity Measurements
    }

    transmission_string = transmission_string + " /A " + String(raw_readings[0], 6)  + " " + String(raw_readings[1], 6)  + " " + String(raw_readings[2], 6);                                         // Acceleration measurements
  }

  transmission_string = generate_transmission_string(transmission_string);

  // Transmit the readings with LoRa
  LoRa_string(transmission_string, receiver_node, 2);

  // Store the transmission in the transmission array
  // Shorten the String
  transmission_string = get_substring(transmission_string, "SP ", "/n");
  transmission_string.remove(0, 3);
  
  // Store the String
  store_to_transmission_storage(transmission_string);

  // Clearing variables
  num_measurements      = 0;
  raw_readings[0]       = 0;
  raw_readings[1]       = 0;
  raw_readings[2]       = 0;
  processed_readings[0] = 0;
  processed_readings[1] = 0;
  processed_readings[2] = 0;
  processed_readings[3] = 0;
}

// Stanp the Transmission String with the labels for this SP
String generate_transmission_string(String transmission_string) {

  transmission_string = "SP " + String(node_ID) + " " + String(millis()) + " " + transmission_string + " /n ID Time Rssi Snr";

  return transmission_string;
}

// Subfunction that tranmits a String using the LoRa chip
void LoRa_string(String transmission_string, int receiver_node, int transmission_security_factor) {

  // Enhancing the Transmission String
  // Adding the 'to' and 'over' stamps

 // Variable for the over stamp
 int best_path = -1;
    
  if (receiver_node != -1) {
    // Adding a 'to' stamp
    transmission_string = "to " + String(receiver_node) + " " + transmission_string;

    // If there is a special destination for the string, try to find a good path over another SP
    // Adding an 'over' stamp if necassary
    best_path = find_fastest_path(receiver_node);
    
    // If there is a best path in the data base, add an over stamp
    if (best_path > -1) { // If there is a path in the list ( != -1) and if there is an over Node (no! direct connection), add the over node
      transmission_string = "over " + String(best_path) + " " + transmission_string;
    }
  }

  //////////////////
  // Sending Packet
  //////////////////

  if (debug_mode == 1) { // Debugging Mode
    Serial.println("");
    Serial.print("Sending packet: ");
    Serial.println(transmission_string);
  }

  // Checking of the message security factor
  if (transmission_security_factor == 0) { // Security factor 0 - only send packages once
    // Get incomming transmission so long that there are incomming transmissions
    get_transmissions();
  
    LoRa.beginPacket();
    /*
      LoRa.setTxPower(txPower,RFOUT_pin);
      txPower -- 0 ~ 20
      RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
        - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
        - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
    */
    LoRa.setTxPower(14, PA_OUTPUT_PA_BOOST_PIN/*RF_PACONFIG_PASELECT_PABOOST*/);  // Set transmit power to 14 dBm
      
    for (int i = 0; i < transmission_string.length(); i++) {
      LoRa.print(transmission_string.charAt(i));
    }
    LoRa.endPacket();

    digitalWrite(led, LOW);    // Turn the LED off by making the voltage LOW
  }
  else if (transmission_security_factor == 1) { // Security factor 1 - send packages multiple times

    // Get incomming transmission so long that there are incomming transmissions
    get_transmissions();
    
    // After there is no SP transmitting transmit the message
    digitalWrite(led, HIGH);   // Turn the LED on
     
    // Sending the packet 5 times
    for (int package_send = 0; package_send < 5; package_send++) {

      if (debug_mode == 1) { // Debugging Mode
        Serial.println("");
        Serial.print("Sending packet: ");
        Serial.println(transmission_string);
      }

      LoRa.beginPacket();
      /*
        LoRa.setTxPower(txPower,RFOUT_pin);
        txPower -- 0 ~ 20
        RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
          - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
          - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
      */
      LoRa.setTxPower(14, PA_OUTPUT_PA_BOOST_PIN/*RF_PACONFIG_PASELECT_PABOOST*/);
      
      for (int i = 0; i < transmission_string.length(); i++) {
        LoRa.print(transmission_string.charAt(i));
      }
      LoRa.endPacket();
  
      delay(10);
    }
    
    digitalWrite(led, LOW);    // Turn the LED off by making the voltage LOW
  }
  else if (transmission_security_factor == 2) { // Security factor 2 - send packages multiple times until a received-message is receid
    package_arrived   = 0;                      // Will be set to one from the transmission interpreter if a message containing Ack was received
    
    last_transmission = get_substring(transmission_string, "SP", "/n"); // Stores the content of the last transmission. Used to detect passive Ack, e.g. received forwaredings by another SP
    
    // Variable to calculate the number of transmission cycles
    // try 5 times to send the transmission
    // break if transmission was received
    // clear the over stamp after 5 attempts
    int transmission_cycle = 0;

    // Variable to calculate the trasmission time
    // the package will then wait the same amout of time for acknowledgements
    int time_transmitting  = 0;
    
    while (package_arrived == 0) {
      // Get incomming transmission so long that there are incomming transmissions
      get_transmissions();

      // After there is no SP transmitting transmit the message
      time_transmitting = millis();
      
      digitalWrite(led, HIGH);   // Turn the LED on
      
      // Sending the packet 5 times
      for (int package_send = 0; package_send < 5; package_send++) {
  
        if (debug_mode == 1) { // Debugging Mode
          Serial.println("");
          Serial.print("Sending packet: ");
          Serial.println(transmission_string);
        }
  
        LoRa.beginPacket();
        /*
          LoRa.setTxPower(txPower,RFOUT_pin);
          txPower -- 0 ~ 20
          RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
            - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
            - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
        */
        LoRa.setTxPower(14, PA_OUTPUT_PA_BOOST_PIN);
        
        for (int i = 0; i < transmission_string.length(); i++) {
          LoRa.print(transmission_string.charAt(i));
        }
        LoRa.endPacket();
    
        delay(10);
      }
  
      digitalWrite(led, LOW);    // Turn the LED off by making the voltage LOW

      // Calculate transmission time so that the waiting funtion waits for the same amout of time for acks
      time_transmitting = millis() - time_transmitting;

      if (debug_mode == 1) { // Debugging Mode
        Serial.println("");
        Serial.print("Transmission done (" + String() + "ms)");
      }

      time_transmitting = time_transmitting * (random(80, 120)/100) * 2;

      if (time_transmitting < 100) {
        time_transmitting = 100;
      }

      if (debug_mode == 1) { // Debugging Mode
        Serial.println(" - Waiting for " + String(time_transmitting) + "ms");
        Serial.println("");
      }
      // Waiting Function
      // Wait for incoming transmission for the transmission time
      // Break if the receaval was acknowledged

      int time_waiting = millis();
      
      while (millis() - time_waiting < time_transmitting && package_arrived == 0) {  
        // Get the Sensor readings
        get_sensor_readings();
      
        // Process the sensor readings if necassary and detect events
        pre_process_sensor_readings();
        process_sensor_readings();
        detect_events();
        
        // Look for incomming transmission
        get_transmissions();
      }

      // If no ack was received after 5 transmission periods, the over package must be dead
      // Thus, remove the over stamp, so that every SP can foreward the package
      // In addition, remove the dead packages ID from the path memory 
      // Then find the next best path in the transmission memory
      // If thereit none, transmit the package without the over stamp so that any SP can foreward the package
      if (transmission_cycle == 4 && transmission_string.indexOf("over") != -1 && best_path != -1 && package_arrived == 0) {

        
        if (debug_mode == 1) { // Debugging Mode
          Serial.println("");
          Serial.println("Transmission was not possible - SP " + String(best_path) + " might be dead");
          Serial.println("");
        }
      
        // Removing the 'over' stamp
        transmission_string = get_substring(transmission_string, "to", " ");
        
        // Removing the ID from the transmission memory
        clear_path(best_path);

        // Find a new best path
        // If there is a special destination for the string, try to find a good path over another SP
        // adding an over stamp if necassary
        best_path = find_fastest_path(receiver_node);
        
        // If there is still a best path in the data base, add an over stamp
        if (best_path > -1) { // If there is a path in the list ( = -1) and if there is an over Node (no! direct connection), add the over node
          transmission_string = "over " + String(best_path) + " " + transmission_string;
        }
    
        transmission_cycle = 0;
      }
      else {
        transmission_cycle = transmission_cycle + 1;
      }
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.println("Transmission successfull!");
      Serial.println("");
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////
//  Incomming Transmissions
////////////////////////////////////////////////////////////////////////////////////

// Function to get incomming transmissions
void get_transmissions() {

  while(LoRa.parsePacket() > 0) {

    String transmission_incomming;
  
    digitalWrite(led, HIGH);    // Turn the LED on

    
    // Read incomming packet
    while (LoRa.available()) {
      transmission_incomming = transmission_incomming + (char)LoRa.read(); // Officially gets stuck here
    }
    
  
    digitalWrite(led, LOW);    // Turn the LED off by making the voltage LOW
  
    // Adding the receiving information to the string
    String temp_transmission_incomming = get_substring(transmission_incomming, "/n", " ");
    
    String temp_ID   = get_substring(temp_transmission_incomming, "ID", " Time") + " " + String(node_ID);
    String temp_time = get_substring(temp_transmission_incomming, "Time", " Rssi") + " " + String(millis());
    String temp_Rssi = get_substring(temp_transmission_incomming, "Rssi", " Snr") + " " + String(LoRa.packetRssi());
    String temp_Snr  = get_substring(temp_transmission_incomming, "Snr", " ") + " " + String(LoRa.packetSnr());
    
    transmission_incomming = get_substring(transmission_incomming, " ", "/n") + "/n " + temp_ID + " " + temp_time + " " + temp_Rssi + " " + temp_Snr;

    if (transmission_incomming != "") {
  
      if (debug_mode == 1) { // Debugging Mode
        Serial.println();
        Serial.println("Received Transmission: " + transmission_incomming);
        Serial.println();
      }

      manage_incoming_transmissions(transmission_incomming);
    }
  }
}

// Subfunction for processing and managing incomming transmissions
void manage_incoming_transmissions(String transmission_incomming) {

  // Message Processing
  if (transmission_incomming != "") {

    int From  = -1;
    int over  = -1;
    int To    = -1;

    ////////////////////////////////////
    // Pre-Processing                 //
    // getting the over value         //
    ////////////////////////////////////
    if (transmission_incomming.startsWith("over")) {
      // Converting the over stamt to an integer
      over = get_part(transmission_incomming, 1, 2).toInt();

      if (debug_mode == 1) {
        Serial.println("OVER: " + String(over));
      }

      // Deleting the over note from the transmission String
      transmission_incomming = get_substring(transmission_incomming, "to", " ");
    }

    // Getting the 'to' value
    if (transmission_incomming.startsWith("to")) {

      // Converting the 'to' stamp to an integer  
      To = get_part(transmission_incomming, 1, 2).toInt();

      if (debug_mode == 1) {
        Serial.println("TO: " + String(To));
      }
      
      // Deleting the to note from the transmission String
      String temp_string = get_part(transmission_incomming, 0, 2);
      transmission_incomming.remove(0, temp_string.length() + 1);
    }

    if (debug_mode == 1) {
      Serial.println(transmission_incomming);
    }
    
    // Sending an arrived/acknowledgement message
    // Every message tht is now a ping or an acknowledgement is send with transmission security factor 2 and therefore needs an Ack to make the transmittion SP stop transmitting
    if (transmission_incomming.indexOf("/C Ack") == -1 && transmission_incomming.indexOf("/h") == -1) { // For every message, that is not a ping ar and acknowledment, send an acknowledgement
      if (To == node_ID || To == -1 || over == node_ID) { // Only send it this SP is a destination of the message or when there is no destination-/To-Stamp
        String transmission_string = "/C Ack";
        transmission_string = generate_transmission_string(transmission_string);

        // Accessing which SP sended this String, so that the Acknowledgement can be send to the right package
        // Variable to store where the Message is from
        int from = get_part(get_substring(transmission_incomming, "SP", "/n"), 1, 2).toInt(); // If the ID String is to short, and contains only only one ID, the sending SP was the Origina SP
        
        // The package can also be forewarded by other SP, and therefore the Path-ID Stamps have to be assed
        String temp_from = get_substring(transmission_incomming, "/n ID", " Time");
        temp_from.remove(0, 6); // remove "/n ID "

        // Look trough the ID-Stamps for the SP, that has send the message - It is the secound last ID, becaus the last one is the ID of this SP
        if (get_part(temp_from, 0, 1) != "" && get_part(temp_from, 1, 2) != "") { // if there are in Min 2 ID's in the ID-String, the Origin SP is not the Sending SP
          
          int i = 0;
          
          while (get_part(temp_from, i + 1, i + 2) != "") {
            // The SP, that has send is message, is the secound last ID in the Stamp section, 
            if (get_part(temp_from, i, i + 1) != "" && get_part(temp_from, i + 1, i + 2) != "" && get_part(temp_from, i + 2, i + 3) == "") {
              from = get_part(temp_from, i, i + 1).toInt();
            }

            i = i + 1;
          }         
        }
  
        delay(random(2000, 4000)); //delay(random(2000, 8000));   // why??
          
        LoRa_string(transmission_string, from, 1);
    
        // Store the transmission in the transmission array
        // Shorten the String
        transmission_string = get_substring(transmission_string, "SP ", "/n");
        transmission_string.remove(0, 3);
          
        // Store the String
        store_to_transmission_storage(transmission_string);  
      }
    }    

    // If the message is a passive or active acknowledgement, set package arrived to 1
    // If the transmissions is an active acknowledgement
    if (transmission_incomming.indexOf("/C Ack") != -1 && node_ID == To) {    // If this message is an Acknowledgement and the receiver To stamp equals this packages ID
      if (debug_mode == 1) {
        Serial.println();
        Serial.println("Passiv Ack");
        Serial.println();
      }
      
      package_arrived = 1; // Variable that makes the LoRa_string functiong stopping the transmissions
    }
    else if (get_substring(transmission_incomming, "SP", "/n") == last_transmission) {    // If the transmissions is a passive acknowledgement, e.g. another package forwarding the initial transmission

      if (debug_mode == 1) {
        Serial.println();
        Serial.println("Active Ack");
        Serial.println();
      }
      
      package_arrived = 1; // Variable that makes the LoRa_string functiong stopping the transmissions
    }

    // Further action
    // If package_arrived == 0, there is a message that this package tries to transmit
    // therefore, no transmissions can be forewarded, and all incomin transmissions are stored in a memory and will be processed further after the message was transmitted
    // If there is no package to transmit, the function manage_incoming_transmissions_processing will decide for further actions regarding to the incoming message
    if (package_arrived != 0) {
      manage_incoming_transmissions_processing(transmission_incomming, over, To);
    }
    else if (package_arrived == 0){

      if (debug_mode == 1) {
        Serial.println("");        
        Serial.println("Currently transmitting, therefore the message will be stored ito the temporary transmission memory");
        Serial.println("");
      }

      for (int i = 0; i < 100; i++) {
        if (temp_incoming_transmissions_message[i] == "") {
          temp_incoming_transmissions_message[i] = transmission_incomming;
          temp_incoming_transmissions_over[i]    = over;
          temp_incoming_transmissions_to[i]      = To;

          if (temp_incoming_transmissions_message[i+1] != "") {
            temp_incoming_transmissions_message[i+1] = "";
            temp_incoming_transmissions_over[i+1]    = -1;
            temp_incoming_transmissions_to[i+1]      = -1;
          }
        }
        else if (i == 99) {
          temp_incoming_transmissions_message[0] = "";
          temp_incoming_transmissions_over[0]    = -1;
          temp_incoming_transmissions_to[0]      = -1;
        }
      }
    }
  }
}

void manage_incoming_transmissions_processing(String transmission_incomming, int over, int To) {
  
    /////////////////////////////////
    // Processing - the Magic begins
    /////////////////////////////////
    if (transmission_incomming.startsWith("SP")) { // Message from sensor package
      // Separating between the different message parts
      // String that contains readings/commands/messages
      String Readings = get_substring(transmission_incomming, "SP", "/n");
      Readings.remove(0, 3);

      // String that contains the path
      String Path = get_substring(transmission_incomming, "/n", " ");
      Path.remove(0, 3);

      // Adding the Origin-SP to the Path
      Path = get_part(Readings, 0, 2) + " /p " + Path;
      
      if (debug_mode == 1) { // Debugging Mode
        Serial.println("Message: " + Readings);
        Serial.println("Path: " + Path);
      }

      // Storing the Transmission and checking if it was stored before
      int message_is_new = store_to_transmission_storage(Readings);
      // Storing the Network Path
      int path_is_new    = store_to_network_paths(Path);

      // Print the message to the serial monitor if it is a new message or the path is new
      if (serial_mode == 1) {
        if ( message_is_new == 1 || path_is_new == 1) {
          if (node_ID == master_device) {
            print_as_JSON(transmission_incomming);
          }
          else {
            Serial.println(transmission_incomming);
          }
        }
      }
        
      // If the message is new, interpret or foreward it (only if this SP is not the destination)
      if (message_is_new == 1) {

        if (Readings.indexOf("/C SOS") == -1 && Readings.indexOf("/C Ack") == -1 && Readings.indexOf("/C /h") == -1) { // if this message is NO SOS message, Ping or Acknowledgement

          if (To == node_ID) { // Interpreting
            // Interpret only if the message is a command or a chat message
            if (Readings.indexOf("/C") != -1 || Readings.indexOf("/M") != -1) {
              transmission_interpreter(Readings);
            }
          }
          else if (To != node_ID && over == node_ID) { // Forewarding - if this device should foreward the message
            // Foreward the message, if it has not been forewarded before
            foreward_transmission(transmission_incomming, To);
          }
          else if (To != node_ID && over == -1) { // Forewarding - if the other device does not know it, this device should foreward the message
            // Foreward the message, if it has not been forewarded before
            foreward_transmission(transmission_incomming, To);
          }
          else if (To != node_ID && To != -1 && over != node_ID) { // This is SP is not the destination and also not the over-SP, so there must be a better path and therefore the message will not be forewarded
            if (debug_mode == 1) { // Debugging Mode
              Serial.println("There seems to be a better connection for this package, therefore it will not be forewarded");
            }
          }   
          else if (To == -1) { // Forewarding - if there was no path attched, send to master node
            foreward_transmission(transmission_incomming, master_device);
          }
          else {
            if (debug_mode == 1) { // Debugging Mode
              Serial.println("Interpreter Failiure");
            }
          }
        }
        
        if (Readings.indexOf("/C /h") != -1) { // If this message is a Ping
                                               // Do nothing!!!
        }
        else if (Readings.indexOf("/C SOS") != -1) { // If the message is an SOS message

          Readings = "SP " + Readings + "/n";

          foreward_transmission(Readings, -1);
          
          if (debug_mode == 1) {
            Serial.println();
            Serial.println("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////");
            Serial.println("SOS Message incomming");
          }
          
          Serial.println(Readings);
          
          if (debug_mode == 1) {
            Serial.println("////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////");
            Serial.println();
          }
          
          display_mode = 0;
        }
      }
    }
    else {
      if (debug_mode == 1) { // Debugging Mode
        Serial.println("Not able to interpret the Message: " + transmission_incomming);
      }
    }
}

void manage_archived_transmissions() {
//  if (debug_mode == 1) { // Debugging Mode
//    Serial.println("");
//    Serial.println("Now checking the list of messages received during last transmission:");
//    Serial.println("");
//  }

  int num_messages_in_temp = 0;
  
  for (int i = 0; i < 100; i++) {
    if (temp_incoming_transmissions_message[i] != "") {

      num_messages_in_temp = num_messages_in_temp + 1;
      
      if (debug_mode == 1) {
        Serial.println("Stored Transmission to compute: " + String(i) + " [To: " + String(temp_incoming_transmissions_to[i]) + "; Over: " + String(temp_incoming_transmissions_over[i]) + "] : " + temp_incoming_transmissions_message[i]);
      }
    }
  }
        
  // Manage all transmissions that were store during the last period
  if (num_messages_in_temp > 0) {
    for (int i = 0; i < 100; i++) {
      if (temp_incoming_transmissions_message[i] != "") {
  
        // Process the message
        manage_incoming_transmissions_processing(temp_incoming_transmissions_message[i], temp_incoming_transmissions_over[i], temp_incoming_transmissions_to[i]);
    
        // Clear the array
        temp_incoming_transmissions_message[i] = "";
        temp_incoming_transmissions_over[i] = -1;
        temp_incoming_transmissions_to[i] = -1;
      }
    }
  }
}

// Function that takes a received message as Input and forewards it if it was not forewarded before
void foreward_transmission(String transmission_string, int receiver_node) {

  if (debug_mode == 1) { // Debugging Mode
    Serial.println("");
    Serial.println("Forewarding packet: " + transmission_string);
  }

  delay(random(500, 1000));

  LoRa_string(transmission_string, receiver_node, 2);
}

// Function that interprets incomming Transmissions
void transmission_interpreter(String transmission_incomming) {

  if (transmission_incomming.indexOf("/M") != -1) { // This message is a chat message
    
    String temp_string = get_substring(transmission_incomming, "/M", " ");
    temp_string.remove(0, 3);

    if (debug_mode == 1) {
      Serial.println();
      Serial.println("Cave Chat - New Chat Message: ");
      Serial.println("Sender " + get_part(transmission_incomming, 0, 1) + ": " + temp_string);
      Serial.println();
      Serial.println("To answer, type chat");
      Serial.println();
    }
  }
  else if (transmission_incomming.indexOf("/C") != -1) { // Is the transmission a Command?

    if (debug_mode == 1) {
      Serial.println(transmission_incomming);
    }
    
    String temp_string = get_substring(transmission_incomming, "/C", " ");
    temp_string.remove(0, 3);

    if (temp_string.startsWith("TR")) { // Transmit Readings
      transmit_readings(master_device);
    }
    else if (temp_string.startsWith("TM")) { // Transmission Mode

      temp_string.remove(0, 3);

      if (debug_mode == 1) {
        Serial.println("Setting Changed: Transmission Mode");
      }

      if (temp_string.startsWith("ON")) {

        if (debug_mode == 1) {
          Serial.println("Enabled");
        }
        constant_transmission_mode = 1;
      }
      else if (temp_string.startsWith("OFF")) {

        if (debug_mode == 1) {
          Serial.println("Disabled");
        }

        constant_transmission_mode = 0;
      }
      else {
        if (debug_mode == 1) {
          Serial.println("Command not supported");
        }
      }
    }
    else if (temp_string.startsWith("PM")) { // Processing Mode
      temp_string.remove(0, 3);

      if (debug_mode == 1) {
        Serial.print("Setting Changed: Processing Mode");
      }

      if (temp_string.startsWith("ON")) {

        if (debug_mode == 1) {
          Serial.println(" Enabled");
        }

        processing_mode = 1;
      }
      else if (temp_string.startsWith("OFF")) {

        if (debug_mode == 1) {
          Serial.println("Disabled");
        }

        processing_mode = 0;
      }
      else {
        if (debug_mode == 1) {
          Serial.println("Command not supported");
        }
      }

    }
    else if (temp_string.startsWith("DM")) { // Display Mode
      temp_string.remove(0, 3);

      if (debug_mode == 1) {
        Serial.print("Setting Changed: Display Mode");
      }

      if (temp_string.startsWith("ON")) {

        if (debug_mode == 1) {
          Serial.println(" Enabled");
        }

        display_mode = 1;
      }
      else if (temp_string.startsWith("OFF")) {

        if (debug_mode == 1) {
          Serial.println("Disabled");
        }

        display_mode = 0;
      }
      else {
        if (debug_mode == 1) {
          Serial.println("Command not supported");
        }
      }

    }
    else if (temp_string.startsWith("CB")) { // Calibration
      temp_string.remove(0, 3);

      if (temp_string.startsWith("X")) {
        
        temp_string.remove(0, 2);

        if (temp_string.startsWith("min")) {
          temp_string = get_part(temp_string, 1, 2);

          ref_min[0] = double(temp_string.toFloat());
          
        }
        else if (temp_string.startsWith("max")) {
          temp_string = get_part(temp_string, 1, 2);
          
          ref_max[0] = double(temp_string.toFloat());
        }
        else if (temp_string.startsWith("off")) {
          temp_string = get_part(temp_string, 1, 2);

          
          offset[0] = double(temp_string.toFloat());
        }
      }
      else if (temp_string.startsWith("Y")) {
        
        temp_string.remove(0, 2);

        if (temp_string.startsWith("min")) {
          temp_string = get_part(temp_string, 1, 2);

          ref_min[1] = double(temp_string.toFloat());
          
        }
        else if (temp_string.startsWith("max")) {
          temp_string = get_part(temp_string, 1, 2);
          
          ref_max[1] = double(temp_string.toFloat());
        }
        else if (temp_string.startsWith("off")) {
          temp_string = get_part(temp_string, 1, 2);

          
          offset[1] = double(temp_string.toFloat());
        }
        
      }
      else if (temp_string.startsWith("Z")) {
        
        temp_string.remove(0, 2);

        if (temp_string.startsWith("min")) {
          temp_string = get_part(temp_string, 1, 2);

          ref_min[2] = double(temp_string.toFloat());
          
        }
        else if (temp_string.startsWith("max")) {
          temp_string = get_part(temp_string, 1, 2);
          
          ref_max[2] = double(temp_string.toFloat());
        }
        else if (temp_string.startsWith("off")) {
          temp_string = get_part(temp_string, 1, 2);

          
          offset[2] = double(temp_string.toFloat());
        }
        
      }
    }
    else if (temp_string.startsWith("ST")) { // Status

      // Variable for the Origin SP
      int From = get_part(transmission_incomming, 0, 1).toInt();

      for (int i = 0; i < (node_num * 4); i++) {
        if (path_from[i] == node_ID || path_to[i] == node_ID) {

          String transmission_string = String(path_millis[i]) + " /S /n";
          
          if (path_from[i] == node_ID) {
            if (path_over[i] == -1) {
              transmission_string = String(path_to[i]) + transmission_string;
            }
            else {
              transmission_string = String(path_over[i]) + transmission_string;
            }
          }
          else if (path_to[i] == node_ID) {
            if (path_over[i] == -1) {
              transmission_string = String(path_from[i]) + transmission_string;
            }
            else {
              transmission_string = String(path_over[i]) + transmission_string;
            }
          }

          transmission_string = "SP " + transmission_string + " ID " + String(node_ID) + " Time " + String(millis()) + " Rssi " + String(path_rssi[i]) + " Snr " + String(path_snr[i]);

          LoRa_string(transmission_string, From, 2);
    
          // Store the transmission in the transmission array
          // Shorten the String
          transmission_string = get_substring(transmission_string, "SP ", "/n");
          transmission_string.remove(0, 3);
          
          // Store the String
          store_to_transmission_storage(transmission_string);
        }
      }
        

      
    }
    else if (temp_string.startsWith("RST")) { // RESET

      if (debug_mode == 1) {
        Serial.println("");
        Serial.println("-----------------------------------");
        Serial.println("-------------Resetting-------------");
        Serial.println("-----------------------------------");
        Serial.println("");
      }

      resetFunc();
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Command not supported");
      }
    }
  }
  else {
    if (debug_mode == 1) {
      Serial.println("Interpreter was not able to interpret the incomming Message/Command: " + transmission_incomming);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////
//  String Processing
////////////////////////////////////////////////////////////////////////////////////

// Function that returns a String from a String, at the position in the string after a number of spaces
// get_part(SP 2 G 1.01 0.98, 3) will return 1.01 as String
String get_part(String input_string, int start_space, int end_space) {

  int space = 0;             // Number of spaces
  String output_string = ""; // Varialbel to store the returning string

  for (int i = 0; i < (input_string.length()); i++) {
    // Monitors the number of spaces
    if (input_string.charAt(i) == ' ') {
      space = space + 1;
    }

    // Creating the output string
    if (start_space <= space && space < end_space) {
      output_string = output_string + input_string.charAt(i);
    }

    if (end_space < space) {
      break;
    }
  }

  return output_string;
}

// Function that returns a supbstring from the input string
String get_substring(String input_string, String start_string, String end_string) {
  if (input_string.indexOf(start_string) == -1) {
    return "-1";
  }
  else if (input_string.indexOf(end_string) == -1) {
    return "-1";
  }
  else {
    if (start_string != " " && end_string != " ") {
      return input_string.substring(input_string.indexOf(start_string), input_string.indexOf(end_string));
    }
    else if (start_string == " " && end_string != " ") {
      return input_string.substring(0, input_string.indexOf(end_string));
    }
    else if (start_string != " " && end_string == " ") {
      return input_string.substring(input_string.indexOf(start_string), input_string.length());
    }
    else {
      return "-1";
    }
  }
}


////////////////////////////////////////////////////////////////////////////////////
// Information Storage
////////////////////////////////////////////////////////////////////////////////////

// Store a new message in the transmission storage array
int store_to_transmission_storage(String input_string) {

  // Check if the Message has been seen before, and if not, store the message
  int is_in_array = 0;
  // Check if the received message is in the memory
  for (int i = 0; i < node_num; i++) {

    if (transmission_storage[i] == input_string) {
      is_in_array = 1;
      break;
    }
  }

  // If the message is in Memory, only store the new path
  // Foreward the message if it is not already in the memory
  if (is_in_array != 1) {
    // Look for the next free line in the array
    int i = 0;

    for (i = 0; i < node_num; i++) {
      if (transmission_storage[i] == "" ) {
        break;
      }
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.println("Storing in transmission_storage[" + String(i) + "]: " + input_string);
    }

    // Store the Transmission in the next free line
    transmission_storage[i] = input_string;

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("transmission_storage[" + String(i) + "]: " + transmission_storage[i]);
    }

    // Clearing the next line so that the next message can be stored under it, or if the aray is full, in the first line
    if (i < (node_num - 2)) {
      i = i + 1;
    }
    else {
      i = 0;
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("Freeing Memory in transmission_storage[" + String(i) + "]: " + transmission_storage[i]);
    }

    transmission_storage[(i)] = "";

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("transmission_storage[" + String(i) + "]: " + transmission_storage[i]);
    }

    return 1;
  }
  else {

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("Transmission has been stored before");
    }

    return 0;
  }


}

// Store a net network path in the network path array
int store_to_network_paths(String input_string) {

  // Storing the Network Path
  int is_in_array = 0;

  // Check if the received Path is in the memory
  for (int i = 0; i < (node_num * 2); i++) {
    if (input_string == network_paths[i]) {
      is_in_array = 1;
      break;
    }
  }

  if (is_in_array != 1) {
    // Storing the Strings in their arrays
    // Look for the next free line in the array
    int i = 0;

    for (i = 0; i < (node_num * 2); i++) {
      if (network_paths[i] == "" ) {
        break;
      }
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.println("Storing in network_paths[" + String(i) + "]: " + input_string);
    }

    // Store the Transmission in the next free line
    network_paths[i] = input_string;

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("network_paths[" + String(i) + "]: " + network_paths[i]);
    }

    // Clearing the next line so that the next message can be stored under it, or if the aray is full, in the first line
    if (i < ((node_num * 2) - 2)) {
      i = i + 1;
    }
    else {
      i = 0;
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("Freeing Memory in network_paths[" + String(i) + "]: " + network_paths[i]);
    }

    network_paths[(i)] = "";

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("network_paths[" + String(i) + "]: " + network_paths[i]);
    }

    analyse_path(input_string);

    return 1;
  }
  else {
    if (debug_mode == 1) { // Debugging Mode
      Serial.println("Path is already in the Memory");
    }

    return 0;
  }

}

////////////////////////////////////////////////////////////////////////////////////
// Mesh Implementation
////////////////////////////////////////////////////////////////////////////////////

// Analyses a new found path
void analyse_path(String path_temp) {

  int    from = 0;
  int    over = 0;
  int    To[path_temp.length()];
  double rssi_temp[path_temp.length()];
  double snr_temp[path_temp.length()];

  // Initializing
  for (int i = 0; i < node_ID * 4; i++) {
    To[i] = -1;
    rssi_temp[i] = 0;
    snr_temp[i] = 0;
  }

  // Get the Package Origin
  from = get_part(path_temp, 0, 1).toInt();

  // Get the whole previous path
  path_temp = get_substring(path_temp, "/p", " ");
  path_temp.remove(0, 3);


  // Separately work on one stamp after another
  // ID Stamp
  // Getting a part from the Path that consists only the stamp for ID's
  String stamp_temp = get_substring(path_temp, "ID", " Time");
  
  // Remove "ID"
  stamp_temp.remove(0,3);
  
  int i = 0;
  
  while (get_part(stamp_temp, i, i + 1) != "") {
    To[i] = get_part(stamp_temp, i, i + 1).toInt();
    
    i = i + 1;
  }

  
  // RSSI Stamp
  // Getting a part from the Path that consists only the stamp for RSSI Values
  stamp_temp = get_substring(path_temp, "Rssi", " Snr");

  // Remove "Rssi"
  stamp_temp.remove(0,5);
  
  i = 0;
  while (get_part(stamp_temp, i, i + 1) != "") {
    rssi_temp[i] = get_part(stamp_temp, i, i + 1).toDouble();
    
    i = i + 1;
  }
  
  // Snr Stamp

  // Getting a part from the Path that consists only the stamp for Millis Values
  stamp_temp = get_substring(path_temp, "Snr", " ");

  // Remove "Snr"
  stamp_temp.remove(0,4);

  i = 0;
  while (get_part(stamp_temp, i, i + 1) != "") {
    snr_temp[i] = get_part(stamp_temp, i, i + 1).toDouble();

    i = i + 1;
  }

  i = i - 1;

  // Processing Path
  if (debug_mode == 1) { // Debugging Mode
    Serial.println("Processing Path: ");
  }

  while (i >= 0) {

    double rssi_sum = 0;
    double snr_sum  = 0;

    if (i >= 1) {
      over = To[i - 1];
    }
    else {
      over = To[i];
    }

    for (int index = 0; index <= i; index++) {
      rssi_sum = rssi_sum + rssi_temp[index];
      snr_sum = snr_sum + snr_temp[index];
      if (index == i) {
        rssi_sum = rssi_sum / (i + 1);
        snr_sum  = snr_sum / (i + 1);
      }
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.println("From: " + String(from) + " Over: " + String(over) + " To: " + String(To[i]) + " RSSI: " + String(rssi_sum) + " Snr: " + String(snr_sum));
    }

    store_update_path(from, over, To[i], rssi_sum, snr_sum);

    i = i - 1;
  }
}

void store_update_path(int from, int over, int To, double RSSI_sum, double Snr_sum) {
  //////////////////////////////////////////////////////////////////////////////////////////////
  // Store this path
  // Check if the Path has been seen before, and if not, store the message
  // In any case, store or update the RSSI Sum

  // Only store the path if from and to are different
  // Paths should connect two different devices, and not a single one in a circle
 
  if (from != To) {

    // Check if this is a direct path
    if (over == To || over == from) {
      over = -1;
    }
   
    int array_position = 0;

    // Check if the path is in the memory
    int i = 0;

    for (i = 0; i < (node_num * 4); i++) {

      if (path_from[i] == from && path_over[i] == over && path_to[i] == To) { // If the path is already known

        if (debug_mode == 1) { // Debugging Mode
          Serial.println("");
          Serial.println("Path is already known ");
        }

        break;
      }
      else if (path_from[i] == To && path_over[i] == over && path_to[i] == from) { // If the path is already known

        if (debug_mode == 1) { // Debugging Mode
          Serial.println("");
          Serial.println("Path is already known, but in the other direction ");
        }

        break;
      }
      else if (path_from[i] == -1) { // If a new line was found
        // Update the RSSI-Sum- and the millis-arrays only

        if (debug_mode == 1) { // Debugging Mode
          Serial.println("Found new Path");
        }

        path_from[i] = from;
        path_over[i] = over;
        path_to[i] = To;
        break;
      }
    }

    if (i < (node_num * 4) - 2) {

      // Update the RSSI-Sum- and the millis-arrays only
      if (debug_mode == 1) { // Debugging Mode
        Serial.println("Update the RSSI-Sum- and the millis-arrays");
      }

      path_rssi[i]   = RSSI_sum;
      path_snr[i]    = Snr_sum;
      path_millis[i] = millis();

    }
    else {

      int oldest_path = 0;

      if (debug_mode == 1) { // Debugging Mode
        Serial.println("Memory overload, clearing old paths");
      }

      // Look for the oldest paths
      for (int i = 0; i < (node_num * 4); i++) {
        if (oldest_path < path_millis[i]) {
          oldest_path = path_millis[i];
          array_position = i;
        }
      }

      if (debug_mode == 1) { // Debugging Mode
        Serial.print("Clearing Path: Register " + String(array_position) + " From " + String(path_from[array_position]) + " Over " + path_over[array_position]);
        Serial.println(" To " + String(path_to[array_position]) + " RSSI " + String(path_rssi[array_position]) + " Snr " + String(path_snr[i]) + " Millis " + String(path_millis[array_position]));
      }

      // Clear the oldest path and writing the new values
      path_from[array_position]   = from;
      path_over[array_position]   = over;
      path_to[array_position]     = To;
      path_rssi[array_position]   = RSSI_sum;
      path_snr[array_position]    = Snr_sum;
      path_millis[array_position] = millis();
    }

    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.print("Storing/Updating Path: Register " + String(array_position) + " From " + String(path_from[array_position]) + " Over " + path_over[array_position]);
      Serial.println(" To " + String(path_to[array_position]) + " RSSI " + String(path_rssi[array_position]) + " Millis " + String(path_millis[array_position]));
    }
  }
  else {
    if (debug_mode == 1) { // Debugging Mode
      Serial.println("");
      Serial.println("From equals to - this path is a circle");
    }
  }


  // Timeout function
  // Finally look for very old paths and clear them
  // Timeout is performed every 3 hours
  for (int i = 0; i < (node_num * 4); i++) {
    if ((millis() - path_millis[i]) > (3 * 60 * 60 * 1000) && path_from[i] != -1 && path_over[i] != -1 && path_to[i] != -1 && path_rssi[i] != -1 && path_snr[i] != -1) {

      if (debug_mode == 1) { // Debugging Mode
        Serial.println("");
        Serial.println("Timeout - Clearing old paths");
        Serial.print("Old Path: Register " + String(i) + " From " + String(path_from[i]) + " Over " + String(path_over[i]));
        Serial.println(" To " + String(path_to[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) + " Millis " + String(path_millis[i]));
      }

      // Clear the oldest path
      path_from[i]   = -1;
      path_over[i]   = -1;
      path_to[i]     = -1;
      path_rssi[i]   = -1;
      path_snr[i]    = -1;
      path_millis[i] = -1;

      if (debug_mode == 1) { // Debugging Mode
        Serial.print("Clearing Path: Register " + String(i) + " From " + String(path_from[i]) + " Over " + String(path_over[i]));
        Serial.println(" To " + String(path_to[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) +  " Millis " + String(path_millis[i]));
      }

    }
  }
}

// If in LoRa_String() was detected, that an over package might be dead, beacause no ack was received
// It is assumed that this package might be dead
// The ID of the dead package is inserted into this function, and it will clear all paths that use the dead packages ID
void clear_path(int dead_package) {
  
  if (debug_mode == 1) { // Debugging Mode
    Serial.println("");
    Serial.println("Package " + String(dead_package) + " is unreachable and possible died in the last period");
    Serial.println("Removing all path with its ID");
    Serial.println(""); 
  }

  for (int i = 0; i < (node_num * 4); i++) {
    if (path_from[i] != dead_package || path_over[i] != dead_package || path_to[i] != dead_package) {

      if (debug_mode == 1) { // Debugging Mode
        Serial.println("");
        Serial.print("Old Path: Register " + String(i) + " From " + String(path_from[i]) + " Over " + String(path_over[i]));
        Serial.println(" To " + String(path_to[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) + " Millis " + String(path_millis[i]));
      }

      // Clear the oldest path
      path_from[i]   = -1;
      path_over[i]   = -1;
      path_to[i]     = -1;
      path_rssi[i]   = -1;
      path_snr[i]    = -1;
      path_millis[i] = -1;

      if (debug_mode == 1) { // Debugging Mode
        Serial.print("Clearing Path: Register " + String(i) + " From " + String(path_from[i]) + " Over " + String(path_over[i]));
        Serial.println(" To " + String(path_to[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) +  " Millis " + String(path_millis[i]));
      }

    }
  }
}

// Looks in the path memory for the path with the highest mean RSSI value
// returns the ID of the Over package of the best path
int find_fastest_path(int receiver_node) {

  if (debug_mode == 1) { // Debugging Mode
    Serial.println();
    Serial.println("List of paths:");

    for (int i = 0; i < (node_num * 4); i++) {
      if (path_from[i] != -1) {
        Serial.print("[ From " + String(path_from[i]) + " Over ");
        
        if (path_over[i] != -1) {
          Serial.print(String(path_over[i]));
        }
        else {
          Serial.print("-");
        }
        
        Serial.println(" To " + String(path_to[i]) + " Millis " + String(path_millis[i]) + " RSSI " + String(path_rssi[i]) + " Snr " + String(path_snr[i]) + " ]");
      }
    }
  }

  int temp_rssi   = -32750;
  int temp_snr    = -32750;
  int temp_over   = -1;
  int temp_millis = -1;

  for (int i = 0; i < (node_num * 4); i++) {
    if (path_from[i] == receiver_node && path_to[i] == node_ID) {
      if (temp_rssi == path_rssi[i] && temp_millis < path_millis[i]) { // If two paths have the same rssi value, take the one with the higher millis value because it's newer
        temp_rssi = path_rssi[i];
        temp_snr = path_snr[i];
        temp_over = path_over[i];
        temp_millis = path_millis[i];
      }
      else if (temp_rssi < path_rssi[i]) { // Take the path with the smallest rsi value, because this indicates a good connection
        temp_rssi   = path_rssi[i];
        temp_snr    = path_snr[i];
        temp_over   = path_over[i];
        temp_millis = path_millis[i];
      }
    }
    else if (path_from[i] == node_ID && path_to[i] == receiver_node) {
      if (temp_rssi == path_rssi[i] && temp_millis < path_millis[i]) { // If two paths have the same rssi value, take the one with the higher millis value because it's newer
        temp_rssi   = path_rssi[i];
        temp_snr    = path_snr[i];
        temp_over   = path_over[i];
        temp_millis = path_millis[i];
      }
      else if (temp_rssi < path_rssi[i]) { // Take the path with the smallest rsi value, because this indicates a good connection
        temp_rssi   = path_rssi[i];
        temp_snr    = path_snr[i];
        temp_over   = path_over[i];
        temp_millis = path_millis[i];
      }
    }
  }

  if (temp_millis > -1) {
    if (temp_over != -1) {
      if (debug_mode == 1) {
        Serial.println("Found strongest Path from " + String(node_ID) + " over " + String(temp_over) + " to " + String(receiver_node) + " with RSSI " + String(temp_rssi) + " Snr " + String(temp_snr));
      }
    }
    else {
      if (debug_mode == 1) {
        Serial.println("Found strongest direct Path from " + String(node_ID) + " to " + String(receiver_node) + " with RSSI " + String(temp_rssi) + " Snr " + String(temp_snr));
      }

      temp_over = -1; // Indicates, that no path or a direct connect was found
    }
  }
  else {
    if (debug_mode == 1) {
      Serial.println("No Path found in memory");
    }
  }

  return temp_over;
}

////////////////////////////////////////////////////////////////////////////////////
// Main Loop
////////////////////////////////////////////////////////////////////////////////////

void loop() {
  
  // Get the Sensor readings
  get_sensor_readings();

  // Process the sensor readings if necassary
  pre_process_sensor_readings();
  process_sensor_readings();

  // Check if there are special events in this measurements
  detect_events();

  // Print the readings to the Serial Monitor
  if (millis() - lastprinted > interval_serial || extraordinary_event == 1) {
    
    if (serial_mode == 1) {
      serial_print_readings();
    }

    // Setting the Serial Variables
    lastprinted = millis();
    interval_serial = random(interval_min, interval_max);
  }

  // Send Sensor readings
  manage_outgoing_transmissions();
  
  // If a moonquake was defected, transmit the readings immediately
  // After that emergency transmission, wait for the current interval to finish and transmit it's readings
  // After that Emergency transmission will be possible again
  if (extraordinary_event == -1) {
    extraordinary_event = 0;
  }
  else if (extraordinary_event == 1) {
    extraordinary_event = -1;
  }
  
  // Send ping with known network paths
  transmit_ping();

  // Looking for Serial input
  excecute_serial_commands("");

  // Function to get commands from the master
  get_transmissions();

  // Finally transmit all the messages that were received during the last transmission period
  manage_archived_transmissions();
}
