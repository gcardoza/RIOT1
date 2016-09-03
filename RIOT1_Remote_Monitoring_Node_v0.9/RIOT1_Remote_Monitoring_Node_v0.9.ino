//  Project:  RIOT1 - Remote IOT Node for monitoring Weather - Temperature, Humidity, Barometric Pressure
//  Author:   Geofrey Cardoza
//  Baseline: June 14th, 2016
//  Revision: August 14th, 2016  v0.9
//
//  Hardware Configuration:
//    Arduino Nano v3
//    LCD Nokia 5110 Display
//    DHT22 for Temperature and Pressure
//    BMP180 for Pressure and Temperature
//    Moisture Sensor
//    DS1307 Real-Time Clock

// ***** Include header files *****
  #include <stdio.h>
  #include "string.h"
  #include <SoftwareSerial.h>       // Library for Serial Communications
  #include "DHT22.h"                // Library for DHT22 Temperature and Humidity sensor
  #include <Wire.h>                 // Library for I2C Communication
//  #include "LowPower.h"           // Library for Low Power mode support for Arduino
  #include <LCD5110_Graph.h>        // Library for Nokia 5110 LCD display
  #include <SFE_BMP180.h>           // Library for BMP180 Pressure and Temperature sensor
  #include <DS1307RTC.h>            // Library for DS1307 Real-Time Clock
  #include <TimeLib.h>              // Library for Date and Time functions
  
// ***** Declare global variables for the RIOT1 Node
  #define Update_Interval 20000                         // set Data Update frequence to every 60 seconds (for debug I set it to 20)
  #define Max_Device_Fail_Count 2                       // read device a max of this count then ignore it
  int DHT22_Issues=0, BMP180_Issues=0, DS1307_Issues=0; // Is the sensor present and operational 0= Operational, >0 is the failed count
  unsigned long Update_Sequence = 0;                    // Update sequence number to base
  int debug = 1;                                        // Set to 0 for production and 1 for debug mode

// Initialize the Nokia 5110 LCD Display
  extern unsigned char SmallFont[];
  LCD5110 lcd(5, 6, 7, 8, 9);       // Format: LCD5110(int SCK, int MOSI, int DC, int RST, int CS);
  
// ***** Set Device ID Variables *****
  #define Device_ID1_Pin  2
  #define Device_ID2_Pin  3
  #define Device_ID3_Pin  4
  int Device_ID;

// ***** Set DHT22 Temp and Pressure Variables *****
  #define DHT22_Pin 13     // The digital IO pin the DHT22 is connected to
  DHT22 myDHT22(DHT22_Pin);
  float DHT22_Temperature, DHT22_Humidity;
  
// ***** Set BMP180 Pressure Sensor Variables *****
  SFE_BMP180 myBMP180;
  #define ALTITUDE 260.0 // Altitude at Home - 4 Peace Court Caledon ON, Canada
  float BMP180_Pressure, BMP180_Temperature;

// ***** Set Moisture Sensor Variables
  #define Moisture_1_Pin  0 // Analog input pin
  #define Moisture_2_Pin  1 // Analog input pin
  #define Moisture_3_Pin  2 // Analog input pin
  #define Wet_Level 1000  // Readings above this are considered soaked or fully wet
  #define Dry_Level 400   //Readings below this are considered bonme dry


// ***** Set DS1307 Real-Time Clock Variables *****
  //*?* Nothing for now
  
// ***** Set Serial Interface Variables to Bluetooth device *****
  #define Tx_Pin  1
  #define Rx_Pin  0


// ********** INITIALIZE ALL COMPONENTS OF THE SYSTEM **********
void setup(void)
{
  // Initialize the Serial Port to the Bluetooth adapter
    // Start Serial Port first so that debug data can be sent to the Base Station
  pinMode(Tx_Pin, OUTPUT);
  pinMode(Rx_Pin, INPUT);
  Serial.begin(9600);
  Serial.print(F("\n***** STARTING RIOT1 Node 20160814 v0.9 *****\n"));
  Serial.print(F("***** ENTERING SETUP MODE *****\n"));
        
  //  ***** Initialize Display *****
  Serial.print(F("-> Initializing Nokia 5110 LCD Display\n"));
  lcd.InitLCD();
  lcd.setContrast(55);
  lcd.setFont(SmallFont);
  lcd.print("RIOT1 v0.9",LEFT,0);     // Display Splash Page
  lcd.print("  Monitoring",LEFT,16);
  lcd.print("  System Node",LEFT,24);
  lcd.print("Excaliber Inc.",LEFT,40);
  lcd.update();
  delay(5000);

  // ***** 1. Set Device ID Input Pins *****
  Serial.print(F("-> Device ID: Initializing Dip Switch Pins\n"));
  pinMode(Device_ID1_Pin, INPUT_PULLUP);
  pinMode(Device_ID2_Pin, INPUT_PULLUP);
  pinMode(Device_ID3_Pin, INPUT_PULLUP);
        
  // ***** 2. Initialize DHT22 Temperature and Humidity sensor *****
  Serial.print(F("-> DHT22: Starting Temperature & Pressure Sensor "));
  read_DHT22();              // Read the device 2 times to see if it has started and is operational
  delay(2000);               // put a 2 sec delay between succesive reads to ensure it is up
  read_DHT22();
  Serial.print(F("\n"));

  // ***** 3. Initialize BMP180 Pressure sensor *****
  Serial.print(F("-> BMP180: Starting Pressure and Temperature Sensor "));
  myBMP180.begin(); //Initialize BMP180 Pressure sensor
  Serial.print(F("\n"));

  // ***** 4. Initialize Moisture Sensor *****
  Serial.print(F("-> Moisture: --> Starting Sensors"));
  Serial.print(F("\n"));
  
  // ***** 5. Initialize DH1307 Real-Time Clock *****
  Serial.print(F("-> DH1307: Starting Real-Time Clock Module"));
    //*?* Nothing for now
  Serial.print(F("\n"));
}

// ********** MAIN PROGRAM LOOP **********
void loop()
{
  // ***** Define variables *****
  int Status, v1, v2;
  char buffer[50], SW_b[10], ID_b[2], DT_b[10], DH_b[10], BT_b[10], BP_b[10], M1_b[10], M2_b[10], M3_b[10], SE_b[10];
  float Moisture1, Moisture2, Moisture3;
  float Moisture1_p, Moisture2_p, Moisture3_p;
  tmElements_t tm;    // Variable for RTC
  
  if(debug) Serial.print(F("***** ENTERING MAIN PROGRAM LOOP *****\n"));
  
  // ***** 1. Read the RIOT1 Device ID *****
  if (debug) Serial.print(F("-> DeviceID: Reading Dip Switches\n"));
  read_DeviceID();
  
  // ***** 2. Get Temperature & Humidity from DHT22 sensor and store in Global Vars *****
  if (DHT22_Issues < Max_Device_Fail_Count)
  {
    if (debug) Serial.print(F("-> DHT22: Reading Temperature and Humidity"));
    
    Status = read_DHT22();
    if (Status == 0)
    {
      if (debug) Serial.print(F("-> DHT22: Read OK\n"));
    }
    else
    {
      if (debug) Serial.print(F("-> DHT22: Read FAILED\n"));
      DHT22_Temperature = 0;
      DHT22_Humidity = 0;
      DHT22_Issues += 1;
    }
  }

  // ***** 3. Get Pressure and Temperature from BMP180 module and store in Global Vars *****
  if (BMP180_Issues < Max_Device_Fail_Count) 
  {
    if (debug) Serial.print(F("-> BMP180: Reading Temperature and Pressure"));
    
    Status = read_BMP180();
    if (Status == 0)
    {  
      if (debug) Serial.print(F("-> BMP180: Read OK\n"));
    }
    else
    {
      if (debug) Serial.print(F("-> BMP180: Read FAILED\n"));
      BMP180_Temperature = 0;
      BMP180_Pressure = 0;
      BMP180_Issues += 1;
    }
  }
    
  // ***** 4. Get Moisture Readings from sensors and store in Global Vars *****
  if (debug) Serial.print(F("-> Moisture: Reading Sensors\n"));
  Moisture1 = analogRead(Moisture_1_Pin);   // Moisture reading in 10 bit resolution. 5V = 1023
  Moisture2 = analogRead(Moisture_2_Pin);
  Moisture3 = analogRead(Moisture_3_Pin);

  Moisture1_p = convertToPercent(Moisture1);  // Moisture reading in percentage. 0 = dry, 100 = soaked
  Moisture2_p = convertToPercent(Moisture2);
  Moisture3_p = convertToPercent(Moisture3);
            
  // ***** 5. DS1307 Date and Time and store in Global Vars *****
  if (debug) Serial.print(F("-> DL1307: Reading RTC Date and Time\n"));

  if (RTC.read(tm))
  {
    if (debug) Serial.print ("Real Time Clock Read Successful\n");
  }
  else
  {
    if (debug) Serial.print ("Real Time Clock Read Failed\n");
  }
  
  // Format RIOT1 Sensor Data and send to Base Station
  sprintf(buffer, "%02d-%02d-%02d %02d:%02d:%02d", tmYearToCalendar(tm.Year), tm.Month, tm.Day, tm.Hour, tm.Minute, tm.Second);
  dtostrf(Device_ID, 2, 0, ID_b);
  dtostrf(DHT22_Temperature, 5, 1, DT_b);
  dtostrf(DHT22_Humidity, 5, 1, DH_b);
  dtostrf(BMP180_Temperature, 5, 1, BT_b);
  dtostrf(BMP180_Pressure, 5, 1, BP_b);
  dtostrf(Moisture1_p, 5, 1, M1_b);
  dtostrf(Moisture2_p, 5, 1, M2_b);
  dtostrf(Moisture3_p, 5, 1, M3_b);
  dtostrf(Update_Sequence, 6, 0, SE_b);
         
  Serial.print(F("RIOT1:  0.9"));             // Node Type and Software Version
  Serial.print(F(", ID:"));                   // Node ID
  Serial.print(ID_b);
  Serial.print(F(", TS:"));                   // Date and time(24hr) when RTC is integrated
  //Serial.print(F("2016-06-26 14:21:35"));
  Serial.print(buffer);
  Serial.print(F(", DT:"));                   // DHT22 Temperature
  Serial.print(DT_b);
  Serial.print(F(", DH:"));                   // DHT22 Humidity
  Serial.print(DH_b);
  Serial.print(F(", BT:"));                   // BMP180 Temperature
  Serial.print(BT_b);
  Serial.print(F(", BP:"));                   // BMP180 Pressure
  Serial.print(BP_b);
  Serial.print(F(", M1:"));                   // Moisture 1
  Serial.print(M1_b);
  Serial.print(F(", M2:"));                   // Moisture 2
  Serial.print(M2_b);
  Serial.print(F(", M3:"));                   // Moisture 3
  Serial.print(M3_b);  
  Serial.print(F(", SE:"));                   // Update sequence
  Serial.print(SE_b);  
  Serial.print(F("\n")); 
  
  // Clear Nokia 5110 LCD Display and print data on it
//  lcd.InitLCD();
  lcd.setFont(SmallFont);
  sprintf(buffer, "Excaliber RIOT");
  lcd.print(buffer,LEFT,0);
  
  sprintf(buffer, "20160625 20:31");
  lcd.print(buffer,LEFT,8);  
  
  v1 = DHT22_Temperature; //I couldn't get sprintf working with floats so had to kludge this
  v2 = DHT22_Humidity;
  sprintf(buffer, "DT:%3d DH:%3d", v1, v2);
  lcd.print(buffer,LEFT,16);  

  v1 = BMP180_Temperature;
  v2 = BMP180_Pressure;
  sprintf(buffer, "BT:%3d BP:%3d", v1, v2);
  lcd.print(buffer,LEFT,24); 

  v1 = Moisture1_p;
  v2 = Moisture2_p;
  sprintf(buffer, "M1:%3d M2:%3d", v1, v2);
  lcd.print(buffer,LEFT,32); 

  v1 = Moisture3_p;
  sprintf(buffer, "M3:%3d Node:%1d", v1, Device_ID);
  lcd.print(buffer,LEFT,40); 

  lcd.update();

  // Update Record Sequence
  Update_Sequence++;
  if (Update_Sequence > 999999)
    Update_Sequence = 0;
  
  // Reset Debug mode to off after the first loop
  debug = 0;
  
  // Wait update interval prior to sending next update
  delay(Update_Interval);
}

// *************** Sub-Routines *********************

// ***** Returns the device ID from the 3 ID pins - value 1 to 8 *****
void read_DeviceID()
{
  Device_ID = 1;
  if(digitalRead(Device_ID1_Pin) == HIGH) Device_ID += 1;
  if(digitalRead(Device_ID2_Pin) == HIGH) Device_ID += 2;
  if(digitalRead(Device_ID3_Pin) == HIGH) Device_ID += 4;
}

//  ***** Read the Temperature and Humidity from the DHT22 sensor over a serial digital I/O port *****
int read_DHT22()
{
  int DHT22_Read_Status;
  
  DHT22_Read_Status = myDHT22.readData();
  switch (DHT22_Read_Status)
  {
    case DHT_ERROR_NONE:  
      DHT22_Temperature = myDHT22.getTemperatureC();
      DHT22_Humidity = myDHT22.getHumidity();
      return(0);
    break;
    case DHT_ERROR_CHECKSUM: 
      Serial.print(F("-> DHT22: ERROR Checksum error"));
      return(-1);               
    break;
    case DHT_BUS_HUNG:
      Serial.print(F("-> DHT22: ERROR Bus Hung")); 
      return(-2);                
    break;
    case DHT_ERROR_NOT_PRESENT:
      Serial.print(F("-> DHT22: ERROR Not Present"));
      return(-3);
      break;
    case DHT_ERROR_ACK_TOO_LONG:
      Serial.print(F("-> DHT22: ERROR ACK time out"));
      return(-4);
      break;
    case DHT_ERROR_SYNC_TIMEOUT:
      Serial.print(F("-> DHT22: ERROR Sync Timeout"));
      return(-5);
      break;
    case DHT_ERROR_DATA_TIMEOUT:
      Serial.print(F("-> DHT22: ERROR Data Timeout"));
      return (-6);
      break;
    case DHT_ERROR_TOOQUICK:
      Serial.print(F("-> DHT22: ERROR Polled to quick"));
      return(-7);
      break;
    default: 
      Serial.print(F("-> DHT22: ERROR Unknown"));
      return(-10);                
    break;
  }
}

//  ***** Read the Temperature and Pressure from the BMP180 sensor over I2C *****
int read_BMP180()
{
  char status;
  double T,P,p0,a;

  status = myBMP180.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = myBMP180.getTemperature(T);
    if (status != 0)
    { 
      BMP180_Temperature = T;
      status = myBMP180.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = myBMP180.getPressure(P,T);
        if (status != 0)
        {
          BMP180_Pressure = myBMP180.sealevel(P,ALTITUDE)/10;  //Convert Pressure to kPa
          return(0);
        }
        else
        {
          Serial.print(F("-> BMP180: ERROR Starting Pressure Measurement"));
          return(-4);
        }
      }
      else
      {
        Serial.print(F("-> BMP180: ERROR Starting Pressure Measurement"));
        return(-3);
      }
    }
    else
    { 
      Serial.print(F("-> BMP180: ERROR Reading Temperature"));
      return(-2);
    }
  }
  else
  {  
    Serial.print(F("-> BMP180: ERROR Starting Temperature Measurement"));
    return(-1);
  }
}

// ***** Convert moisture reading to a percent 0% to 100% *****
int convertToPercent(int value)
{
  int percentValue = 0;
  percentValue = map(value, 1023, 465, 0, 100);
  return percentValue;
}

