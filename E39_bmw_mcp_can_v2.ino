#include <SPI.h>
#include <mcp_can.h>
MCP_CAN CAN(53); //CS pin on arduino 10 mega y nano
#define lo8(x) ((int)(x)&0xff)   //LSB Byte menos significativo
#define hi8(x) ((int)(x)>>8)     //MSB Byte mas significativo
// variables del archivo dash_plugin.dll
#define PACKET_SYNC 0xFF
#define PACKET_VER  2
int serial_byte;
//var DME1 rpm
byte rpmL;
byte rpmH;
short tempRPM;
//int msb_rpm;
//int lsb_rpm;
int rpm;
//var Speed
#define SPEED_PIN 48 // Set Speedo Pin
int Speed;
#define SPEED_DEADZONE 5
int SpeedSendVal = 50;
// Can-Bus Variables
byte DME4_Load0;
byte DME4_Load3 = 0x00;
byte DME4_Load5 = 0x00;
//var gears
byte gearcan = 0x00;
byte gearcan2 = 0x00;
//game data variables  
  float speed; //1.-Absolute value for when reversing
  int speed_kph;
  float engine_rpm;//2.-RPM of the engine
  uint16_t msb_rpm;   
  uint16_t lsb_rpm;
  int engine_gear;//s32-17.-Gear currently selected in the engine.
  int gear_reverse; //46.-convertir negative value a positive value
  int forward_gear_count; //47.-ats //10 //13 //18 maybe 8 ets //12 //16 and more
  int input_steering;//18.-Steering received from input <-1;1>. counterclockwise.
  int input_throttle; //19.-Throttle received from input <0;1>
  int input_brake;//20.-Brake received from input <0;1>
  int input_clutch;//21.-Clutch received from input <0;1>
  int effective_steering;//22.-Steering as used by the simulation <-1;1>
  int effective_throttle;//23.-Throttle pedal input as used by the simulation <0;1>
  int effective_brake;//24.-Brake pedal input as used by the simulation <0;1>
  int effective_clutch;//25.-Clutch pedal input as used by the simulation <0;1>
  int cruise_control;//26.-Speed selected for the cruise control in m/s
  //int hshifter_slot;//u32-27.-Gearbox slot the h-shifter handle is currently in.
  //int retarder_level;//28.-Current level of the retarder.
  int brake_air_pressure; //3.-Pressure in the brake air tank in psi
  int brake_temperature; //4.-Temperature of the brakes in degrees celsius.
  int fuel_ratio; //5.-Fuel level porcentaje
  int fuel;//29.-Amount of fuel in liters
  int fuel_average_consumption;//30.-Average consumption of the fuel in liters/km
  int fuel_range;//31.-Estimated range of truck with current amount of fuel in km
  int adblue;//32.-Amount of AdBlue in liters
  //int adblue_average_consumption;//33.-Average consumption of the adblue in liters/km
  int oil_pressure; //6.-Pressure of the oil in psi
  int oil_temperature; //7.-Temperature of the oil in degrees celsius.
  float water_temperature; //8.-Temperature of the water in degrees celsius.
  int battery_voltage; //9.-Voltage of the battery in volts.
  //int light_aux_front;//u32-34.-Are the auxiliary front lights active?
  //int light_aux_roof;//35.-Are the auxiliary roof lights active?
  float wear_engine;//37.-Wear of the engine accessory as <0;1>
  float wear_transmission;//38.-Wear of the transmission accessory as <0;1>
  float wear_cabin;//39.-Wear of the cabin accessory as <0;1>
  float wear_chassis;//40.-Wear of the chassis accessory as <0;1>
  float wear_wheels;//41.-Average wear across the wheel accessories as <0;1>
  int odometer;//42.-The value of the odometer in km
  int navigation_distance;//43.-The value of truck's navigation distance (in meters).
  int navigation_time;//44.-The value of truck's navigation eta (in second).
  int navigation_speed_limit;//45.-The value of truck's navigation speed limit (in m/s).
  int dashboard_backlight; //48.-Intensity of the dashboard backlight as factor <0;1>
  int OCTET_0; //ligths
  int OCTET_1;//warnings
  int OCTET_2;//varias
  int OCTET_3;//ejes elevables

  int text_len; //screen
  //OCTET_0
    int light_beacon;//Are the beacon lights enabled? balizas
    int light_parking;//Are the parking lights enabled?
    int light_lblinker;//Is the light in the left blinker currently on?
    int light_rblinker;//Is the light in the right blinker currently on?
    int light_low_beam;//Are the low beam lights enabled?
    int light_high_beam;//Are the high beam lights enabled?
    int light_brake;//Is the brake light active?
    int light_reverse;//Is the reverse light active?
  //OCTET_1
    int parking_brake;//Is the parking brake enabled?
    int motor_brake;//Is the engine brake enabled?
    int brake_air_pressure_warning;//Is the air pressure warning active?
    int brake_air_pressure_emergency;//Are the emergency brakes active as result of low air pressure?
    int fuel_warning;//Is the low fuel warning active?
    int battery_voltage_warning;//Is the battery voltage/not charging warning active?
    int oil_pressure_warning;//Is the oil pressure warning active?
    int water_temperature_warning;//Is the water temperature warning active?
  //OCTET_2
    int lblinker;//Is the left blinker enabled?  it does not blink and ignores enable state of electric
    int rblinker;//Is the right blinker enabled? it does not blink and ignores enable state of electric
    int hazard_warning;//Are the hazard warning light enabled? it does not blink
    int wipers;//Are the wipers enabled?
    int differential_lock;//Is the differential lock enabled?
    int adblue_warning;//Is the low adblue warning active?
    int electric_enabled;//Is the electric enabled?
    int engine_enabled;//Is the engine enabled?
  //OCTET_3
    int lift_axle;//Is the lift axle control set to lifted state?
    int lift_axle_indicator;//Is the lift axle indicator lit?
    int trailer_lift_axle;//Is the trailer lift axle control set to lifted state?
    int trailer_lift_axle_indicator;//Is the trailer lift axle indicator lit?
// Set kBus
#define kbus Serial2  //Serial 0RX,1TX; Serial1 19RX1,18TX1; Serial2 17RX,16TX; Serial3 15RX,14TX
// K-Bus Variables
byte LightByte1 = 0x60; //00
byte LightByte2 = 0x30; //00
//int right_blinker_state;
//int left_blinker_state;
//int high_beam;
//int left_blinker;
//int right_blinker;

//// Set Relay Pins
int pinAirbag = 36;  //-
int pinOilpressure = 38;  //-
int pinABS = 40;  //-
//int pinParkingpbrake = 42;  //- 
const int PinMotor_brake=42;  
int PinBatt= 44; //-
int PinCheckengine= 46; //-
int pinFuelwarning = 47;
int pinClusterlight = 49;  //42
void setup() {
  kbus.begin(9600, SERIAL_8E1); 
  Serial.begin(115200);
  START_INIT:

      if(CAN_OK == CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ))                   // init can bus : baudrate = 250k
      {
          Serial.println("CAN BUS Shield init ok!");
      }
      else
      {
          Serial.println("CAN BUS Shield init fail");
          Serial.println("Init CAN BUS Shield again");
          delay(100);
          goto START_INIT;
      }
  CAN.setMode(MCP_NORMAL);
  // Output Definition
 // pinMode(SPEED_PIN, OUTPUT);
  pinMode(pinAirbag,OUTPUT);
  pinMode(pinOilpressure,OUTPUT);
  pinMode(pinABS, OUTPUT);
  pinMode(PinBatt,OUTPUT);
  pinMode(PinCheckengine,OUTPUT);
  pinMode(pinFuelwarning, OUTPUT);
  pinMode(pinClusterlight, OUTPUT);
  pinMode(PinMotor_brake,OUTPUT);
// // Output Set
  digitalWrite(pinAirbag, LOW); //36
  digitalWrite(pinOilpressure, LOW); //38
  digitalWrite(pinABS, LOW); //40
  digitalWrite(PinBatt,LOW); //44
  digitalWrite(PinCheckengine,LOW); //46
  digitalWrite(pinFuelwarning, HIGH); // 47
  digitalWrite(pinClusterlight,HIGH); //49
  digitalWrite(PinMotor_brake,LOW); //42
}

void loop() {
//serial
  if (Serial.available() < 43) //<42
    return;
    
    serial_byte = Serial.read();     
    if (serial_byte != PACKET_SYNC)
    return;

    serial_byte = Serial.read();
    if (serial_byte != PACKET_VER)
    return;
//game data variables  
  speed =                           Serial.read(); //1.-Absolute value for when reversing
  speed_kph =                       Serial.read();
  engine_rpm =                      Serial.read();//2.-RPM of the engine
  msb_rpm =                         Serial.read();   
  lsb_rpm =                         Serial.read();
  engine_gear =                     Serial.read();//s32-17.-Gear currently selected in the engine.
  gear_reverse =                    Serial.read(); //46.-convertir negative value a positive value
  forward_gear_count  =             Serial.read(); //47.-ats //10 //13 //18 maybe 8 ets //12 //16 and more
  input_steering =                  Serial.read();//18.-Steering received from input <-1;1>. counterclockwise.
  input_throttle =                  Serial.read(); //19.-Throttle received from input <0;1>
  input_brake =                     Serial.read();//20.-Brake received from input <0;1>
  input_clutch =                    Serial.read();//21.-Clutch received from input <0;1>
  effective_steering =              Serial.read();//22.-Steering as used by the simulation <-1;1>
  effective_throttle =              Serial.read();//23.-Throttle pedal input as used by the simulation <0;1>
  effective_brake =                 Serial.read();//24.-Brake pedal input as used by the simulation <0;1>
  effective_clutch =                Serial.read();//25.-Clutch pedal input as used by the simulation <0;1>
  cruise_control =                  Serial.read();//26.-Speed selected for the cruise control in m/s
  //hshifter_slot =                   Serial.read();//u32-27.-Gearbox slot the h-shifter handle is currently in.
  //retarder_level =                  Serial.read();//28.-Current level of the retarder.
  brake_air_pressure =              Serial.read(); //3.-Pressure in the brake air tank in psi
  brake_temperature =               Serial.read(); //4.-Temperature of the brakes in degrees celsius.
  fuel_ratio =                      Serial.read(); //5.-Fuel level porcentaje
  fuel =                            Serial.read();//29.-Amount of fuel in liters
  fuel_average_consumption =        Serial.read();//30.-Average consumption of the fuel in liters/km
  fuel_range =                      Serial.read();//31.-Estimated range of truck with current amount of fuel in km
  adblue =                          Serial.read();//32.-Amount of AdBlue in liters
  //adblue_average_consumption =      Serial.read();//33.-Average consumption of the adblue in liters/km
  oil_pressure =                    Serial.read(); //6.-Pressure of the oil in psi
  oil_temperature =                 Serial.read(); //7.-Temperature of the oil in degrees celsius.
  water_temperature =               Serial.read(); //8.-Temperature of the water in degrees celsius.
  battery_voltage =                 Serial.read(); //9.-Voltage of the battery in volts.
  //light_aux_front =                 Serial.read();//u32-34.-Are the auxiliary front lights active?
  //light_aux_roof =                  Serial.read();//35.-Are the auxiliary roof lights active?
  wear_engine =                     Serial.read();//37.-Wear of the engine accessory as <0;1>
  wear_transmission =               Serial.read();//38.-Wear of the transmission accessory as <0;1>
  wear_cabin  =                     Serial.read();//39.-Wear of the cabin accessory as <0;1>
  wear_chassis =                    Serial.read();//40.-Wear of the chassis accessory as <0;1>
  wear_wheels =                     Serial.read();//41.-Average wear across the wheel accessories as <0;1>
  odometer =                        Serial.read();//42.-The value of the odometer in km
  navigation_distance =             Serial.read();//43.-The value of truck's navigation distance (in meters).
  navigation_time  =                Serial.read();//44.-The value of truck's navigation eta (in second).
  navigation_speed_limit =          Serial.read();//45.-The value of truck's navigation speed limit (in m/s).
  dashboard_backlight =             Serial.read(); //48.-Intensity of the dashboard backlight as factor <0;1>
  OCTET_0 =                         Serial.read(); //ligths
  OCTET_1 =                         Serial.read();//warnings
  OCTET_2 =                         Serial.read();//varias
  OCTET_3 =                         Serial.read();//ejes elevables

  text_len =                        Serial.read(); //screen
  //OCTET_0
    light_beacon=                  ((OCTET_0 >> 7) & 0x01);//Are the beacon lights enabled? balizas
    light_parking=                 ((OCTET_0 >> 6) & 0x01);//Are the parking lights enabled?
    light_lblinker=                ((OCTET_0 >> 5) & 0x01);//Is the light in the left blinker currently on?
    light_rblinker=                ((OCTET_0 >> 4) & 0x01);//Is the light in the right blinker currently on?
    light_low_beam=                ((OCTET_0 >> 3) & 0x01);//Are the low beam lights enabled?
    light_high_beam=               ((OCTET_0 >> 2) & 0x01);//Are the high beam lights enabled?
    light_brake=                   ((OCTET_0 >> 1) & 0x01);//Is the brake light active?
    light_reverse=                 ((OCTET_0 >> 0) & 0x01);//Is the reverse light active?
  //OCTET_1
    parking_brake=                 ((OCTET_1 >> 7) & 0x01);//Is the parking brake enabled?
    motor_brake=                   ((OCTET_1 >> 6) & 0x01);//Is the engine brake enabled?
    brake_air_pressure_warning=    ((OCTET_1 >> 5) & 0x01);//Is the air pressure warning active?
    brake_air_pressure_emergency=  ((OCTET_1 >> 4) & 0x01);//Are the emergency brakes active as result of low air pressure?
    fuel_warning=                  ((OCTET_1 >> 3) & 0x01);//Is the low fuel warning active?
    battery_voltage_warning=       ((OCTET_1 >> 2) & 0x01);//Is the battery voltage/not charging warning active?
    oil_pressure_warning=          ((OCTET_1 >> 1) & 0x01);//Is the oil pressure warning active?
    water_temperature_warning=     ((OCTET_1 >> 0) & 0x01);//Is the water temperature warning active?
  //OCTET_2
    lblinker=                      ((OCTET_2 >> 7) & 0x01);//Is the left blinker enabled?  it does not blink and ignores enable state of electric
    rblinker=                      ((OCTET_2 >> 6) & 0x01);//Is the right blinker enabled? it does not blink and ignores enable state of electric
    hazard_warning=                ((OCTET_2 >> 5) & 0x01);//Are the hazard warning light enabled? it does not blink
    wipers=                        ((OCTET_2 >> 4) & 0x01);//Are the wipers enabled?
    differential_lock=             ((OCTET_2 >> 3) & 0x01);//Is the differential lock enabled?
    adblue_warning=                ((OCTET_2 >> 2) & 0x01);//Is the low adblue warning active?
    electric_enabled=              ((OCTET_2 >> 1) & 0x01);//Is the electric enabled?
    engine_enabled=                ((OCTET_2 >> 0) & 0x01);//Is the engine enabled?
  //OCTET_3
    lift_axle=                     ((OCTET_3 >> 3) & 0x01);//Is the lift axle control set to lifted state?
    lift_axle_indicator=           ((OCTET_3 >> 2) & 0x01);//Is the lift axle indicator lit?
    trailer_lift_axle=             ((OCTET_3 >> 1) & 0x01);//Is the trailer lift axle control set to lifted state?
    trailer_lift_axle_indicator=   ((OCTET_3 >> 0) & 0x01);//Is the trailer lift axle indicator lit?
 // delay (100);
 //*/ 
water_temperature =           map(water_temperature, 17, 135,  90, 255); // A + DE NOVENTA ENTRA EL WARNING
fuel_average_consumption =       map(fuel_average_consumption, 0, 100, 0, 25); //
//##############################################################################################################  

rpm = msb_rpm;
rpm = rpm <<8;
rpm = rpm + lsb_rpm;
 tempRPM = rpm*6.4;
  rpmL = lo8(tempRPM);
  rpmH = hi8(tempRPM);
//RPM
DME1();
//speed
Speed=speed_kph;
Speedometer();
//water (motor) temperature
DME2();
// enginecheck and cruise control
if ((water_temperature_warning != 0 ||oil_pressure_warning != 0 ||battery_voltage_warning != 0 ||wear_engine  >15 ||wear_transmission >15 || wear_cabin >15 || wear_chassis >15 )&& cruise_control > 0)  //==1
 { DME4_Load0 = 0x0A; } //0x0A AMBOS
else if (cruise_control > 0) {DME4_Load0 = 0x08;}
else if ((water_temperature_warning != 0 ||oil_pressure_warning != 0 ||battery_voltage_warning != 0 ||wear_engine  >15 ||wear_transmission >15 || wear_cabin >15 || wear_chassis >15)&& electric_enabled)
 { DME4_Load0 = 0x02;} 
else {DME4_Load0 = 0x00;}

if (oil_pressure <= 1) {
  DME4_Load3 = 0x02;
} else if(oil_pressure >= 120) {
  DME4_Load3 = 0x08;
} else {
  DME4_Load3 = 0x00;
}
DME4();
 
// brake_air_pressure_emergency
if (brake_air_pressure_emergency){brake_air_pressure_emergency =0xE6;}
ASC1();
//GEAR
if  (forward_gear_count == 18){
  if (rpm ==0) {gearcan = 0x08;gearcan2 = 0x60;}
  else if (light_reverse) {gearcan =0x07; gearcan2 = 0x60;}
  else if (engine_gear== 0) {gearcan =0x06;gearcan2 = 0x60;}
  else if (engine_gear== 1) {gearcan =0x01; gearcan2 = 0x20;}
  else if (engine_gear== 2) {gearcan =0x01;gearcan2 = 0x40;}
  else if (engine_gear== 3) {gearcan =0x02;gearcan2 = 0x20;}
  else if (engine_gear== 4) {gearcan =0x02;gearcan2 = 0x40;}
  else if (engine_gear== 5) {gearcan =0x03;gearcan2 = 0x20;}
  else if (engine_gear== 6) {gearcan =0x03;gearcan2 = 0x40;}
  else if (engine_gear== 7) {gearcan =0x04;gearcan2 = 0x20;}
  else if (engine_gear== 8) {gearcan =0x04;gearcan2 = 0x40;}
  else if (engine_gear== 9) {gearcan =0x09;gearcan2 = 0x20;}
  else if (engine_gear== 10) {gearcan =0x09;gearcan2 = 0x40;}
  else if (engine_gear== 11) {gearcan =0x02;gearcan2 = 0x20;}
  else if (engine_gear== 12) {gearcan =0x02;gearcan2 = 0x40;}
  else if (engine_gear== 13) {gearcan =0x03;gearcan2 = 0x20;}
  else if (engine_gear== 14) {gearcan =0x03;gearcan2 = 0x40;}
  else if (engine_gear== 15) {gearcan =0x04;gearcan2 = 0x20;}
  else if (engine_gear== 16) {gearcan =0x04;gearcan2 = 0x40;}
  else if (engine_gear== 17) {gearcan =0x09;gearcan2 = 0x20;}
  else if (engine_gear== 18) {gearcan =0x09;gearcan2 = 0x40;}
};
GEAR();
//kbus luces
/*
if (lblinker==0){lblinker = false;}
else {lblinker = true;}
if (rblinker==0){rblinker = false;}
else {rblinker = true;}
if (light_high_beam==0){light_high_beam = false;}
else {light_high_beam = true;}


// Begin K-Bus-Data
// Indicators Left / Right

if(lblinker==false && rblinker==false && light_high_beam ==false) {
  LightByte1 = 0x18;
} else if(lblinker==false && rblinker==false && light_high_beam ==true) {
  LightByte1 = 0x04;
} else if(lblinker==true && rblinker==false && light_high_beam ==false) {
  LightByte1 = 0x58;
} else if(lblinker==false && rblinker== true && light_high_beam ==false) {
  LightByte1 = 0x38;
} else if(lblinker==true && rblinker== true && light_high_beam ==false) {
  LightByte1 = 0x78;
} else if(lblinker==true && rblinker== false && light_high_beam ==true) {
  LightByte1 = 0x54;
} else if(lblinker==false && rblinker==true && light_high_beam==true) {
  LightByte1 = 0x34;
} else if(lblinker==true && rblinker==true && light_high_beam==true) {
  LightByte1 = 0x74; 
  
}*/
  // For Lightbyte1
  // 0x08=Front fog lamp
  // 0x10=Rear Fog Lamp
  // 0x18=Fog lamp front+fog lamp rear
  // 0x04=Indicator Highbeam
  // 0x08=Indicator Foglight rear
  // 0x10=Indicator Foglight front
  // 0x14=Fogrear+Highbeam
  // 0x20=Indicator Left
  // 0x24=Left+Highbeam
  // 0x28=Left+Fogrear
  // 0x30=Left+Fogfront
  // 0x34=Left+Fogfront+Highbeam
  // 0x38=Left+Fogfront+Fogrear
  // 0x40=Indicator Right
  // 0x44=Right+Highbeam
  // 0x50=Right+Fogfront
  // 0x54=Right+Fogfront+Highbeam
  // 0x58=Right+Fogfront+Fogrear
  // 0x60=Both Indicators
  // 0x64=Both+Highbeam
  // 0x70=Both+Fogfront
  // 0x74=Both+Fogrfront+Highbeam
  // 0x78=Both+Fogfront+Fogrear
  // For Lightbyte2
  // 0x30=Frontlights
  
LightByte1 = 0x00;
  if(electric_enabled && rblinker)
    LightByte1 = LightByte1 | 0x40;
    
  if(electric_enabled && lblinker)
    LightByte1 = LightByte1 | 0x20;
    
  if((electric_enabled && light_low_beam) && light_high_beam)
    LightByte1 = LightByte1 | 0x04;

  if(hazard_warning)//rblinker && lblinker
    LightByte1 = LightByte1 | 0x60;
  
  if (light_beacon) //light_reverse
    LightByte1 = LightByte1 | 0x10; //04 altas//08 fog //10fog //18 ambas fog
  
  if (electric_enabled && light_low_beam)
    LightByte1 = LightByte1 | 0x08; //04 altas//08 fog //10fog //18 ambas fog
  
  byte mes1[] = {0xD0, 0x08, 0xBF, 0x5B, LightByte1, 0x00, 0x00, LightByte2, 0x00, 0x58, 0x00};
  sendKbus(mes1);

//https://diolum.fr/analyse-gateway-e46
//
//calculadora de fuente E8: SPI D0: LCM 5B: IHKA 80:IKE
//Longitud del paquete. Corresponde al número de bytes contados desde el destino hasta el Checksum inclusive.
//Destino BF: Golbal E8: SPI D0: LCM 5B: IHKA 80:IKE
//Orden 01: Solicitud de estado del dispositivo 0C: Control del vehículo 27: solicitud de visualización MID 53: Solicitud de datos del vehículo 
//DatosOpcional
//Suma de comprobación XOR todos los Bits anteriores.

//digital outputs
if (brake_air_pressure>60) //minimum for good operation 100-120 normal operation
{digitalWrite(pinAirbag,LOW);}
else{digitalWrite(pinAirbag,HIGH);}
// Oilpressure
if (oil_pressure<10 && electric_enabled) //10psi (Idle), 25psi (1200 RPM) MAYBE 40
{digitalWrite(pinOilpressure,HIGH);}
else{digitalWrite(pinOilpressure,LOW);}
//Batery
if (battery_voltage<10) //!=0
{digitalWrite(PinBatt,HIGH);}
else{digitalWrite(PinBatt,LOW);}
// Parkingbrake // absWarning
if (parking_brake !=0) {
digitalWrite(pinABS, HIGH);}
else {digitalWrite(pinABS, LOW);}
//Check engine
//if (water_temperature_warning != 0 ||oil_pressure_warning != 0 ||battery_voltage_warning != 0 ||wear_engine  !=0 ||wear_transmission !=0 || wear_cabin !=0 || wear_chassis !=0)
//{digitalWrite(PinCheckengine,HIGH);}
//else{digitalWrite(PinCheckengine,LOW);}
// fuelWarning
if (fuel_warning!=0) {
digitalWrite(pinFuelwarning, LOW);}
else {digitalWrite(pinFuelwarning, HIGH);}
 //Cluster Backlight
if(light_parking!=0 && electric_enabled!=0) //light_low_beam!=0 && electric_enabled!=0
{digitalWrite(pinClusterlight, LOW);}
else {digitalWrite(pinClusterlight, HIGH);}  
// Motor brake
if(motor_brake>0)
{digitalWrite(PinMotor_brake,HIGH);}
else{digitalWrite(PinMotor_brake,LOW);}


}

void DME1(){
  //CAN ARBID 0x316 (DME1)
  uint8_t rpm_frame[8] = {0x49, 0x0E, 0x00,0x00, 0x0E, 0x00, 0x1B, 0x0E};
  rpm_frame[2] = (rpm & 0xFF);
  rpm_frame[3] = (rpmH);
  CAN.sendMsgBuf(0x316, 0, 8, rpm_frame);
}
void DME2(){
  // CAN ARBID 0x329 (DME2)
  uint8_t temp_frame[8] = {0x8E, 0x00, 0x00,0x00, 0x00, 0x10, 0x01, 0x20};
  temp_frame[1] = (water_temperature);
  CAN.sendMsgBuf(0x329, 0, 8, temp_frame);
}
void DME4(){
  // CAN ARBID 0x545 (DME4)
  uint8_t dme4_frame[8] = {(DME4_Load0), fuel_average_consumption, fuel_average_consumption,(DME4_Load3), (oil_temperature-48), 0x00, 0x00, 0xA0};
  //dme4_frame[0] = (DME4_Load0);
  //dme4_frame[1] = fuel_average_consumption;
  //dme4_frame[2] = fuel_average_consumption; 
  //dme4_frame[3] = (DME4_Load3);
  //dme4_frame[4] = (oil_temperature-48);
  CAN.sendMsgBuf(0x545, 0, 8, dme4_frame);
}
void ASC1(){
  // CAN ARBID 0x153 (ASC)
  uint8_t brakepad_frame[8] = {0x00, 0x00, 0x00,0x00, 0x00, 0x00, 0x00, 0x00};
  brakepad_frame[0] = (brake_air_pressure_emergency );
  CAN.sendMsgBuf(0x153, 0, 8, brakepad_frame);
}
void GEAR(){
  // CAN ARBID 0x545 (DME4)
  uint8_t gear_frame[8] = {0x00, 0x05, 0x20,0x00, 0x00, 0x00, 0x00, 0x00};
  gear_frame[1] = gearcan;
  gear_frame[2] = gearcan2;
  CAN.sendMsgBuf(0x43F, 0, 8, gear_frame);
}
void Speedometer(){
   // Reverse will yield a negative speed, invert.
   if (light_reverse){Speed =0;}
  if(Speed < 0)
  {    Speed = -Speed;  }
  // To prevent jitter at low speeds, this is a deadzone.
  if(Speed < SPEED_DEADZONE)
  {     Speed = 0;   }
  

  // Speedometer Calculate and Output
  if(Speed == 0)
  {
    noTone(SPEED_PIN); 
  }else{
    SpeedSendVal = map(Speed, 0, 250, 0, 1600);
    tone(SPEED_PIN, SpeedSendVal); // 250KmH=1680, 0KmH=0
  }

}
  void sendKbus(byte *data)
{
  int end_i = data[1]+2 ;
  data[end_i-1] = iso_checksum(data, end_i-1);
  kbus.write(data, end_i + 1);
}
byte iso_checksum(byte *data, byte len)//len is the number of bytes (not the # of last byte)
{
  byte crc=0;
  for(byte i=0; i<len; i++)
  {    
    crc=crc^data[i];
  }
  return crc;
}
