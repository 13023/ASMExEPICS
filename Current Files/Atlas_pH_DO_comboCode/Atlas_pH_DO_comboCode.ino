//This code was written to be easy to understand.
//Modify this code as you see fit.
//This code will output data to the Arduino serial monitor.
//Type commands into the Arduino serial monitor to control the pH circuit.
//This code was written in the Arduino 2.0 IDE
//This code was last tested 10/2022

//DO code:
#ifdef USE_PULSE_OUT
  #include "do_iso_grav.h"       
  Gravity_DO_Isolated DO = Gravity_DO_Isolated(A0);         
#else
  #include "do_grav.h"
  Gravity_DO DO = Gravity_DO(A0);
#endif

uint8_t user_bytes_received = 0;
const uint8_t bufferlen = 32;
char user_data[bufferlen];


//pH code:

String inputstring = "";                              //a string to hold incoming data from the PC
String sensorstring = "";                             //a string to hold the data from the Atlas Scientific product
boolean input_string_complete = false;                //have we received all the data from the PC
boolean sensor_string_complete = false;               //have we received all the data from the Atlas Scientific product
float pH;                                             //used to hold a floating point number that is the pH



//DO code:
void parse_cmd(char* string) {
  strupr(string);
  String cmd = String(string);
  if(cmd.startsWith("CAL")){
    int index = cmd.indexOf(',');
    if(index != -1){
      String param = cmd.substring(index+1, cmd.length());
      if(param.equals("CLEAR")){
        DO.cal_clear();
        Serial.println("CALIBRATION CLEARED");
      }
    }
    else{
      DO.cal();
      Serial.println("DO CALIBRATED");
    }
  }
}




//pH code:

void setup() {                                        //set up the hardware
  Serial.begin(9600);                                 //set baud rate for the hardware serial port_0 to 9600
  Serial3.begin(9600);                                //set baud rate for software serial port_3 to 9600
  inputstring.reserve(10);                            //set aside some bytes for receiving data from the PC
  sensorstring.reserve(30);
  //DO code:
  delay(200);
  Serial.println(F("Use command \"CAL\" to calibrate the circuit to 100% saturation in air\n\"CAL,CLEAR\" clears the calibration"));
  if(DO.begin()){
    Serial.println("Loaded EEPROM");
  } 
                            //set aside some bytes for receiving data from Atlas Scientific product
}


void serialEvent() {                                  //if the hardware serial port_0 receives a char
  inputstring = Serial3.readStringUntil(13);           //read the string until we see a <CR>
  input_string_complete = true;                       //set the flag used to tell if we have received a completed string from the PC
}


void serialEvent3() {                                 //if the hardware serial port_3 receives a char
  sensorstring = Serial3.readStringUntil(13);         //read the string until we see a <CR>
  sensor_string_complete = true;                      //set the flag used to tell if we have received a completed string from the PC
}



void loop() {                                         //here we go...

//pH code:

  if (input_string_complete == true) {                //if a string from the PC has been received in its entirety
    Serial3.print(inputstring);                       //send that string to the Atlas Scientific product
    Serial3.print('\r');                              //add a <CR> to the end of the string
    inputstring = "";                                 //clear the string
    input_string_complete = false;                    //reset the flag used to tell if we have received a completed string from the PC
  }


  if (sensor_string_complete == true) {               //if a string from the Atlas Scientific product has been received in its entirety
    Serial.print("pH value: ");
    Serial.println(sensorstring);                     //send that string to the PC's serial monitor
                                                    //uncomment this section to see how to convert the pH reading from a string to a float 
    if (isdigit(sensorstring[0])) {                   //if the first character in the string is a digit
      pH = sensorstring.toFloat();                    //convert the string to a floating point number so it can be evaluated by the Arduino
      if (pH >= 7.0) {                                //if the pH is greater than or equal to 7.0
        Serial.println("pH is high");                       //print "high" this is demonstrating that the Arduino is evaluating the pH as a number and not as a string
      }
      if (pH <= 6.99) {                               //if the pH is less than or equal to 6.99
        Serial.println("pH is low");                        //print "low" this is demonstrating that the Arduino is evaluating the pH as a number and not as a string
      }
    }
  
  }
  sensorstring = "";                                  //clear the string:
  sensor_string_complete = false;                     //reset the flag used to tell if we have received a completed string from the Atlas Scientific product


//DO code:

  if (Serial.available() > 0) {
      user_bytes_received = Serial.readBytesUntil(13, user_data, sizeof(user_data));
  }

  if (user_bytes_received) {
    parse_cmd(user_data);
    user_bytes_received = 0;
    memset(user_data, 0, sizeof(user_data));
  }

  Serial.print("Dissolved Oxygen: ");
  Serial.println( DO.read_do_percentage());
  delay(1000);
}

//-------------------------------------------------
// Beginning of WaterLevel_sensor_8_sensors.ino
//-------------------------------------------------

#define relay 13          
//^Tells Arduino the relay is connected to pin 13
#define sensorPower 7
#define threshold 300

int waterVal = 0;
int stayOn = 1;

void setup()
{
  pinMode(relay, OUTPUT);       // Initialize the Atmel GPIO pin as an output
  
  Serial.begin(9600);

  Serial.println("Setting up...");

    // Set D7 as an OUTPUT
    pinMode(sensorPower, OUTPUT);
    
    // Set to LOW so no power flows through the sensor
    digitalWrite(sensorPower, LOW);

  Serial.println("Ready!");
}

void waterDetectionMain()                  // Loops forever
{
  // digitalWrite(relay, HIGH);   // Turn the relay on (HIGH is the voltage level = 1)
  // delay(10000);                 // Stay ON
  // digitalWrite(relay, LOW);    // Turn the relay off by making the voltage LOW = 0
  // delay(10000);                 // Stay OFF

  int level1 = readSensor(1);
  int level2 = readSensor(2);
  int level3 = readSensor(3);
  int level4 = readSensor(4);
  int level5 = readSensor(5);
  int level6 = readSensor(6);
  int level7 = readSensor(7);
  int level8 = readSensor(8);


  char lvlC[80];
  sprintf(lvlC, "Water level: %4d | %4d | %4d | %4d | %4d | %4d | %4d | %4d", level1, level2, level3, level4, level5, level6, level7, level8);
  Serial.println(lvlC);
    
    delay(500); //wait 0.5 seconds

  if (level1 > threshold || level2 > threshold || level3 > threshold || level4 > threshold || level5 > threshold || level6 > threshold || level7 > threshold || level8 > threshold)
  {
    stayOn = 0;
  }

  if (stayOn == 1)
  {
    digitalWrite(relay, HIGH);
  }
  else
  {
    digitalWrite(relay, LOW);
  }

}

int readSensor(int sensorPin) 
{
    digitalWrite(sensorPower, HIGH);    // Turn the sensor ON
    delay(10);                            // wait 10 milliseconds
  if (sensorPin == 1)
  {
    waterVal = analogRead(A0);
  }
  else if (sensorPin == 2)
  {
    waterVal = analogRead(A1);
  }
  else if (sensorPin == 3)
  {
    waterVal = analogRead(A2);
  }
  else if (sensorPin == 4)
  {
    waterVal = analogRead(A3);        // Read the analog value from sensor
  }
  else if (sensorPin == 5)
  {
    waterVal = analogRead(A4);
  }
  else if (sensorPin == 6)
  {
    waterVal = analogRead(A5);
  }
  else if (sensorPin == 7)
  {
    waterVal = analogRead(A7);
  }
  else
  {
    waterVal = analogRead(A8);
  }

  else 
    digitalWrite(sensorPower, LOW);        // Turn the sensor OFF
    return waterVal;                            // send current reading
}
//-------------------------------------------------
// End of WaterLevel_sensor_8_sensors.ino
//-------------------------------------------------



//DO CODE:

// to use the Atlas gravity circuits with 
// the gravity isolator board's pulse output 
// uncomment line 8: #define USE_PULSE_OUT
// you can use any pins instead of just the analog ones
// but it must be recalibrated
// note that the isolator's analog output also provides isolation

// #define USE_PULSE_OUT


