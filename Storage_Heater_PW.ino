/*
  Storage heater software, 
  James Fotherby 3rd November 2022

  Description:
  This code runs on the Adafruit ESP32 Feather. It has 3 thermistor inputs and 3 outputs: 
    1) A relay that switches on/off the immersion heater
    2) A circulating pump - speed can be modulated via PWM
    3) A fan - speed can be modulated via PWM

  The ESP keeps an uptodate time using NTP
  During off peak electricity hours it heats up a water tank by turning the pump and immersion heater on.
  During on peak hours we run a PI controller to maintain a fixed heat power output - we adjust the pump and fan speeds to do this

  It's a little bit more complicated in reality:
  1)  When heating up the water tank we want to run the pump as slow as possible so the water remains stratified.
      This is done by monitoring the water output temp and modulating the pump speed to maintain a setpoint of ~80C.

  2)  When releasing heat we run a PI controller to adjust the pump speed (ie. flow rate) through the water to air heat exchanger.
      When the tank is very hot - the pump would have to run too slowly so instead we pulse it on/off according to a duty cycle
*/

#include <MovingAverage.h>
#include "time.h"
#include <PID_v1.h>
#include <InfluxDbClient.h>
#include <WiFi.h>
#include <WiFiMulti.h>

//--------------------------------------
#define DEVICE                    "ESP32"

#define WIFI_SSID                 "xxxxxx"
#define WIFI_PASSWORD             "xxxxxx"
#define INFLUXDB_URL              "http://192.168.1.151:8086"
#define INFLUXDB_DB_NAME          "Cheyney_DB"

#define L298_Enable_Pin           13
#define L298_Relay_Pin            33
#define L298_Fan_PWM_Pin          27
#define L298_Pump_PWM_Pin         32

#define Air_In_Temp_ADC_Pin       39
#define Air_Out_Temp_ADC_Pin      36
#define Water_Out_Temp_ADC_Pin    34

#define BCOEFFICIENT              3950
#define THERMISTORNOMINAL         10000
#define TEMPERATURENOMINAL        25 

#define MIN_Pump_Startup          460
#define STARTUP_WATER_FLOW_RATE   40

#define TWENTY_SECONDS            200
#define ONE_MINUTE                600
#define FIVE_MINUTES              3000
#define TEN_MINUTES               6000
#define FOUR_HOURS                144000

#define HEAT_UP_TIME              1
#define RELEASE_HEAT              2

#define PWM_Freq                  10000
#define PWM_Resolution            10

#define Fan_PWM_Ch                0
#define Pump_PWM_Ch               1

#define WATER_SETPOINT            82.0
#define AIR_SETPOINT              15.0
#define AIR_SETPOINT_SLEEP        5.0  

//--------------------------------------
float Convert_to_temperature(uint16_t Thermistor_ADC_Value, uint16_t Pullup_Resistance);
void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst);
void startWifi();
void Save_Influx_Point(float T_Air_In, float T_Air_Out, float T_Water_Out, float Flow_Rate);
void setTimezone(String timezone);
void initTime(String timezone);

float Air_In_Temp, Air_Out_Temp;

MovingAverage <uint16_t, 8> Air_In_Temp_Filter;
MovingAverage <uint16_t, 8> Air_Out_Temp_Filter;
MovingAverage <uint16_t, 8> Water_Out_Temp_Filter;

double    Setpoint_Water_Temp, Measured_Water_Temp, Pump_Speed;
double    Pump_Kp=1.0, Pump_Ki=0.05, Pump_Kd=0.0;
PID       Heat_up_Pump_PI(&Measured_Water_Temp, &Pump_Speed, &Setpoint_Water_Temp, Pump_Kp, Pump_Ki, Pump_Kd, REVERSE);

double    Setpoint_Air_Differential, Air_Differential, Water_flow_rate;
double    Heating_PID_Kp=0.1, Heating_PID_Ki=0.05, Heating_PID_Kd=0.0;
PID       Heat_release_Pump_PI(&Air_Differential, &Water_flow_rate, &Setpoint_Air_Differential, Heating_PID_Kp, Heating_PID_Ki, Heating_PID_Kd, DIRECT);

WiFiMulti       wifiMulti;
InfluxDBClient  client(INFLUXDB_URL, INFLUXDB_DB_NAME);
Point           sensor("Storage_Heater");
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Heat_up_Pump_PI.SetOutputLimits(300,480);   
  Heat_up_Pump_PI.SetSampleTime(200);
  Setpoint_Water_Temp = WATER_SETPOINT;
  Pump_Speed = MIN_Pump_Startup;  

  Heat_release_Pump_PI.SetOutputLimits(16,304);
  Heat_release_Pump_PI.SetSampleTime(200);
  Water_flow_rate = STARTUP_WATER_FLOW_RATE;
  
  Serial.begin(115200);
  startWifi();

  // Init and get the time
  initTime("GMT0BST,M3.5.0/1,M10.5.0"); 
  //setTime(2022, 11, 4, 0, 29, 0, 0);  

  // Configure Pins
  pinMode(L298_Enable_Pin, OUTPUT);
  pinMode(L298_Relay_Pin, OUTPUT);

  digitalWrite(L298_Enable_Pin, HIGH);

  ledcSetup(Fan_PWM_Ch, PWM_Freq, PWM_Resolution);
  ledcAttachPin(L298_Fan_PWM_Pin, Fan_PWM_Ch);

  ledcSetup(Pump_PWM_Ch, PWM_Freq, PWM_Resolution);
  ledcAttachPin(L298_Pump_PWM_Pin, Pump_PWM_Ch);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  static uint16_t Heater_MODE = RELEASE_HEAT;
  static float Avg_Water_flow_rate;
  static uint32_t Temperature_Reached_Count = 0, Heat_Depleted_Count = 0, Log_Data_Count = 0;

  // Get 8 samples and take an average to filter out noise
  for(int i = 0; i < 8; i++) {
    delay(12);

    Air_In_Temp_Filter.add(analogRead(Air_In_Temp_ADC_Pin));
    Air_Out_Temp_Filter.add(analogRead(Air_Out_Temp_ADC_Pin));
    Water_Out_Temp_Filter.add(analogRead(Water_Out_Temp_ADC_Pin));
  }

  // Convert these ADC values into floating point temperatures
  Air_In_Temp = Convert_to_temperature(Air_In_Temp_Filter.get(), 10000);
  Air_Out_Temp = Convert_to_temperature(Air_Out_Temp_Filter.get(), 10000);
  Measured_Water_Temp = Convert_to_temperature(Water_Out_Temp_Filter.get(), 4700);

  // Get the current time and set Off_Peak_Time
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("T Fail");  
  }
  else  {
    Serial.println(&timeinfo, "%H:%M");
    Setpoint_Air_Differential = AIR_SETPOINT_SLEEP;
    
    if(timeinfo.tm_hour == 0 && (timeinfo.tm_min == 30 || timeinfo.tm_min == 31)) {
      Pump_Speed = MIN_Pump_Startup;
      Heat_up_Pump_PI.SetMode(MANUAL);
      Heater_MODE = HEAT_UP_TIME;
    }
    else if(timeinfo.tm_hour == 0 && timeinfo.tm_min >= 30) {
      Heater_MODE = HEAT_UP_TIME;
    }
    else if(timeinfo.tm_hour == 1 || timeinfo.tm_hour == 2 || timeinfo.tm_hour == 3)  {
      Heater_MODE = HEAT_UP_TIME;
    }
    else if(timeinfo.tm_hour == 4 && timeinfo.tm_min < 30) {
      Heater_MODE = HEAT_UP_TIME;
    }
    else if(timeinfo.tm_hour == 4 && timeinfo.tm_min >= 30) {
      Heater_MODE = RELEASE_HEAT;
    }
    else  {      
      Heater_MODE = RELEASE_HEAT;
      Setpoint_Air_Differential = AIR_SETPOINT;
    }
  }

  // Display Temperatures
  Serial.print("I: "); Serial.print(Air_In_Temp); Serial.print("*C, ");
  Serial.print("O: "); Serial.print(Air_Out_Temp); Serial.print("*C, ");
  Serial.print("W: "); Serial.print(Measured_Water_Temp); Serial.println(" *C");

  // Every 1 minute, log our metrics to an influx database on a server on the local network
  Log_Data_Count++;
  if(Log_Data_Count >= TWENTY_SECONDS)  {
    Log_Data_Count = 0;
    Save_Influx_Point(Air_In_Temp, Air_Out_Temp, Measured_Water_Temp, Avg_Water_flow_rate);
  }  

  // ---------------------------------------------------------------------------------
  if(Heater_MODE == HEAT_UP_TIME && Temperature_Reached_Count < FIVE_MINUTES)         // If it's heat up time - Simply PI control pump speed to obtain water setpoint temp.
  {
    if(Measured_Water_Temp > 89.0) {                                                  // If the water out temp gets too hot
      digitalWrite(L298_Relay_Pin, LOW);                                              // This switches off the immersion heater in an overheat situation
    }   
    else if (Measured_Water_Temp < 88.0)  {
      digitalWrite(L298_Relay_Pin, HIGH);                                             // This switches on the immersion heater 
    }

    Heat_release_Pump_PI.SetMode(MANUAL);
    ledcWrite(Fan_PWM_Ch, 432);                                                       // fan to allow for pump input temp to be approximated and to cool enclosure
    Heat_Depleted_Count = 0;
    Water_flow_rate = STARTUP_WATER_FLOW_RATE;                                        // Reset this for when we enter heat release mode.
  
    Heat_up_Pump_PI.Compute();                                                        // Iterate the PI controller based on the water output temp and setpoint. 
    Heat_up_Pump_PI.SetMode(AUTOMATIC);
    ledcWrite(Pump_PWM_Ch, round(Pump_Speed));                                        // Update the pump speed based on the PI controller   

    if(Air_Out_Temp > 45 || Measured_Water_Temp > 86)  {                              // Air_Out_Temp approximates the pump input temperature. (Don't want this too hot)
      Temperature_Reached_Count++;                                                    // If this is above threshold for xx seconds then assume we have charged the tank
    }
    else  {
      if(Temperature_Reached_Count > 0) {
        Temperature_Reached_Count--;
      }
    }

    Avg_Water_flow_rate = (float)(Pump_Speed / 4.0); 
  }
  else                                                                                // Either the tank is fully charged or it's heat release time  
  {
    digitalWrite(L298_Relay_Pin, LOW);                                                // Ensure immersion heater is off 
    Heat_up_Pump_PI.SetMode(MANUAL);
    Heat_release_Pump_PI.SetMode(AUTOMATIC);

    if(Heater_MODE != HEAT_UP_TIME) {
      Temperature_Reached_Count = 0;                                                  // Reset count for the heat up phase   
      Pump_Speed = MIN_Pump_Startup;                                                  // Ensure we start from this pump speed when we switch over to heatup mode 
    }                                              
  }

  if(Heat_release_Pump_PI.GetMode() == AUTOMATIC)
  {
    Air_Differential = Air_Out_Temp - Air_In_Temp;
    Heat_release_Pump_PI.Compute();                                                   // This will output the required Water_flow_rate.

    // The flow rate by the pump is approximately proportional to the voltage applied to the pump. Or in case of on/off control - the average voltage.
    static uint16_t Timer_Count = 0, On_Count = 0;

    // State 1) No heat left in the tank - switch off everything 
    if(Heat_Depleted_Count > FOUR_HOURS)  {
      ledcWrite(Fan_PWM_Ch, 0);                                                    
      ledcWrite(Pump_PWM_Ch, 0);  
      Avg_Water_flow_rate = 0.0;    
    }

    // State 2) Moderately warm tank - can modulate pump speed to maintain a fixed air differential temperature
    else if(Water_flow_rate > 288.0 && On_Count > Timer_Count)  {                     // Tank must be medium temperature (pump must already be on to switch into this state)
      ledcWrite(Fan_PWM_Ch, 440);                                                     // Medium fan speed                                   
      ledcWrite(Pump_PWM_Ch, round(Water_flow_rate));                                 // Can run pump slowly but continuously. Vary speed to maintain air differential temp

      if(Water_flow_rate > 303.9)  {                                                                  
        Heat_Depleted_Count++;                                                 
      }

      Avg_Water_flow_rate = (float)(Water_flow_rate / 4.0);
    }

    // State 3) Tank very hot. Pump would have to run too slowly to run continuously so switch it on/off with a duty cycle that maintains the setpoint air temp differential
    else  {                                                                           // Tank must be very hot, we'll need to switch the pump on/off intermittently
      On_Count = round(TWENTY_SECONDS * Water_flow_rate / 300.0);
      Timer_Count++;
      if(Timer_Count >= TWENTY_SECONDS)  {
        ledcWrite(Pump_PWM_Ch, MIN_Pump_Startup);
        ledcWrite(Fan_PWM_Ch, 460);
        Serial.println("P On Boost");
        Timer_Count = 0;
      }
      else if(Timer_Count < 2)  {                                                     // This gives the pump high power for 200ms so it starts better
        Serial.println("P On Boost");
      }
      else if(On_Count >= Timer_Count)  {
        ledcWrite(Pump_PWM_Ch, 296);
        Serial.println("P On");
      }
      else  {
        ledcWrite(Fan_PWM_Ch, 454);                                                   // When the pump stops the supply voltage slightly rises so this compensates
        ledcWrite(Pump_PWM_Ch, 0);
        Serial.println("P Off");
      }

      Avg_Water_flow_rate = (float)(Water_flow_rate / 4.0);
    }      
    
    Serial.print("AD: "); Serial.print(Air_Differential);
    Serial.print(", AD Setpoint: "); Serial.print(Setpoint_Air_Differential);
    Serial.print(", T: "); Serial.println(Heat_Depleted_Count/10);
  }

  Serial.print("WFR: "); Serial.println(Avg_Water_flow_rate);                         // ranges from 4-76
  Serial.println();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float Convert_to_temperature(uint16_t Thermistor_ADC_Value, uint16_t Pullup_Resistance)
{  
  // convert the value to resistance
  float Thermistor_Resistance = 4095 / (float)Thermistor_ADC_Value - 1;
  Thermistor_Resistance = Pullup_Resistance / Thermistor_Resistance;
  
  float steinhart;
  steinhart = Thermistor_Resistance / THERMISTORNOMINAL;      // (R/Ro)
  steinhart = log(steinhart);                             // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                              // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);       // + (1/To)
  steinhart = 1.0 / steinhart;                            // Invert
  steinhart -= 278.0;                                    // convert absolute temp to C (together with a 5C fudge factor...)

  return(steinhart);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void startWifi()
{
  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();

  // Check influx server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setTimezone(String timezone)
{
  Serial.printf("  Setting Timezone to %s\n",timezone.c_str());
  setenv("TZ",timezone.c_str(),1);  //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void initTime(String timezone)
{
  struct tm timeinfo;

  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org");    // First connect to NTP server, with 0 TZ offset
  if(!getLocalTime(&timeinfo)){
    Serial.println("  Failed to obtain time");
    return;
  }
  Serial.println("  Got the time from NTP");
  // Now we can set the real timezone
  setTimezone(timezone);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setTime(int yr, int month, int mday, int hr, int minute, int sec, int isDst)
{
  struct tm tm;

  tm.tm_year = yr - 1900;   // Set date
  tm.tm_mon = month-1;
  tm.tm_mday = mday;
  tm.tm_hour = hr;      // Set time
  tm.tm_min = minute;
  tm.tm_sec = sec;
  tm.tm_isdst = isDst;  // 1 or 0
  time_t t = mktime(&tm);
  Serial.printf("Setting time: %s", asctime(&tm));
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//#########################################################################################################################################################################
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_Influx_Point(float T_Air_In, float T_Air_Out, float T_Water_Out, float Flow_Rate)
{
  // Store measured value into point
  sensor.clearFields();

  sensor.addField("Air_In_Temperature", T_Air_In);
  sensor.addField("Air_Out_Temperature", T_Air_Out);
  sensor.addField("Water_Out_Temperature", T_Water_Out);
  sensor.addField("Water_Flow_Rate", Flow_Rate);

  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(client.pointToLineProtocol(sensor));

  // If no Wifi signal, try to reconnect it
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }

  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  else  {
    Serial.println("Point added to database");
  }
}
