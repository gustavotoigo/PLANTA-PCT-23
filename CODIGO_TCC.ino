#include <max6675.h>



//HERE IS DEFINE ALL CONSTANTS VALUES

#define PWM_MAX                 255                 //MAXIMUM MAPPING VALUE
#define PWM_MIN                 0                   //MINIMUM MAPPING VALUE
#define RATE_SERIAL             9600                //RATE OF SERIAL
#define ADC_MAX_BITS            1023                //MAXIMUM VALUE ANALOG CONVERTER
#define ADC_MIN_BITS            0                   //MINIMUM VALUE ANALOG CONVERTER
#define DELAY_TIME              150                 //TIME LOOP
#define MIN_VALUE_PWM           30
#define MAX_TEMPERATURE_TEMP1   70
#define MAX_TEMPERATURE_TEMP2   70
#define MAX_TEMPERATURE_TEMP3   70
#define MAX_TEMPERATURE_TEMP4   70
#define MAX_TEMPERATURE_TEMP5   70
#define MAX_TEMPERATURE         100
#define MIN_TEMPERATURE         0

#define SENSOR_1                1
#define SENSOR_2                2
#define SENSOR_3                3
#define SENSOR_4                4
#define SENSOR_5                5

#define PRINT           Serial.print        
#define PRINTN          Serial.println

//FLAG USED BY COLLECT DATA (IF THIS FLAG HAVE HIS VALUE 1, THEN THE COLLECT ARE ACTVATED, AND THE VALUES OF TEMPERATURE WILL BE SENT TO SERIAL, BUT THE OTHER DATA OF SERIAL NOT
#define COLLECT_DATA   HIGH


//PINOS DIGITAIS DE LEITURA DO CLP SELECT SENSOR
#define PD1         26  //CINZA 
#define PD2         27  //AMARELO
#define PD3         28  //AZUL

//PINOS DE LEITURA DE 0-5V QUE VEM DO CLP
#define PWM_AD1     A0  //VERMELHO  (DATA 0) PARA LIGAR 8 PORTA DO CLP
#define PWM_AD2     A1  //ROXO      (DATA 1) PARA LIGAR PORTA 9 DO CLP

//PINOS DE SAIDA PWM DOS MOTORES
#define PWM_MOTOR1  8
#define PWM_MOTOR2  9

//PINO DE SAIDA DE TEMPERATURA
#define TEMP_TO_CLP_PIN 5   //MARROM

//PINOS DE ACIONAMENTO DA RESISTENCIA 1 (SAIDA ARDUINO)
#define R1A         29
#define R1B         30

//PINOS DE ACIONAMENTO DA RESISTENCIA 2 (SAIDA ARDUINO)
#define R2A         31
#define R2B         32

//PINO DE ACIONAMENTO DA RESISTENCIA 1 (ENTRADA ARDUINO) OBS: SINAL VEM DO CLP
#define AR1         33      //LARANJA

//PINO DE ACIONAMENTO DA RESISTENCIA 2 (ENTRADA ARDUINO) OBS: SINAL VEM DO CLP
#define AR2         34      //VERDE

#define BUZZER      3

//PINOS DE CLOCK DA SPI DE CADA MODULO
#define CLK_1       37  
#define CLK_2       40
#define CLK_3       43
#define CLK_4       46
#define CLK_5       49       

//PINOS DO CHIP SELECT DA SPI
#define CS_1        36
#define CS_2        39
#define CS_3        42
#define CS_4        45
#define CS_5        48

//PINOS DE DADOS DA SPI
#define SO_1        35
#define SO_2        38
#define SO_3        41
#define SO_4        44
#define SO_5        47

/*
LAYOUT CONECTOR CLP
PD1     PD2     PD3     PWM_AD1     PWM_AD2     TEMP_TO_CLP_PIN     AR1     AR2

*/

//PROTOTYPES FUNCTIONS
int SelectSensor(void);
float ReadTemp(int number_sensor);
void ReadVoltageToPWM(void);
void SendToCLP(float temp);
void ReadAnalogCLP(void);
void SendDataSerial(void);
void CheckResistances(void);
void CollectData(void);
bool WatcherTemperature(void);
void DisableResistances(void);
void EnableAlarm(bool);

//TEMPERATURE
float temp;

//VALUES RECEIVED FROM PLC 0-5V
int volt1,
    volt2;

//STATES RECEIVED FROM PLC
bool state_1 = 0,
     state_2 = 0,
     state_3 = 0,
     res_state_1 = 0,
     res_state_2 = 0;

int number_sensor;

//CREAT AN INSTANCE USING THE PINS (CLK, CS, SO)
MAX6675 ktc_1(CLK_1, CS_1, SO_1); 
MAX6675 ktc_2(CLK_2, CS_2, SO_2); 
MAX6675 ktc_3(CLK_3, CS_3, SO_3);
MAX6675 ktc_4(CLK_4, CS_4, SO_4); 
MAX6675 ktc_5(CLK_5, CS_5, SO_5); 
  
void setup(){
  Serial.begin(RATE_SERIAL); //INITIALIZE SERIAL
  delay(500); //DELAY TO STABILIZE, JUST ONE TIME

  //CONFIGURE READING PINS OF SENSOR SELECTION AS INPUT
  pinMode(PD1, INPUT);
  pinMode(PD2, INPUT);
  pinMode(PD3, INPUT);

  //CONFIGURE ANALOG INPUT PINS FROM DE PLC
  pinMode(PWM_AD1, INPUT);
  pinMode(PWM_AD1, INPUT);

  //CONFIGURE ENGINE OUTPUT PWM PINS
  pinMode(PWM_MOTOR1, OUTPUT);
  pinMode(PWM_MOTOR1, OUTPUT);

  pinMode(BUZZER, OUTPUT);
}
 
void loop(){
  number_sensor = SelectSensor();           //CALL FUNCTION RESPOSIBLE TO SELECT THE SENSOR CHOSEN
  temp = ReadTemp(number_sensor);           //CALL FUNCTION TO READ DE VALUE OF TEMPERATURE
  SendToCLP(temp);                          //SEND THE VALUE OF TEMPERATURE TO PLC
  ReadVoltageToPWM();                       //READ THE PLC PIN VOLTAGE AND CONVERT TO PWM SIGNAL (CONTROL THE MOTORS)
  
  if(!WatcherTemperature())
    CheckResistances();                     //CHECK IF PLC ENABLED THE RESISTANCE, IF YES THEN ACTIVATES THE CHOOSED RESISTANCE, IF NOT KEEP OFF (VERY CAREFUL WITH THIS FUNCTION)
                   
  SendDataSerial();                         //SEND SOME VALUES TO SERIAL, THESE VALUES ARE FOR TEST ONLY
  CollectData();                            //SEND ON SERIAL ONLY IF THE "COLLECT_DATA" DIRECTIVE IN THE "constants.h" MODULE IS ACTIVATED

  delay(DELAY_TIME);              
}


int  SelectSensor(){
  //READ THE STATES RECEIVED FROM PLC
  state_1 = digitalRead(PD1);
  state_2 = digitalRead(PD2);
  state_3 = digitalRead(PD3);
  

  //DECIDE WHICH SENSOR CHOOSE
  if((state_1 == 0) && (state_2 == 0) && (state_3 == 1))
    return SENSOR_1;
  
  if((state_1 == 0) && (state_2 == 1) && (state_3 == 1))  
    return SENSOR_2;

  if((state_1 == 1) && (state_2 == 1) && (state_3 == 1))
    return SENSOR_3;

  if((state_1 == 1) && (state_2 == 0) && (state_3 == 0))
    return SENSOR_4;

  if((state_1 == 1) && (state_2 == 0) && (state_3 == 1))
    return SENSOR_5;
}

/*TABLE OF POSSIBILITIES
 * PD1    PD2   PD3
 * 0      0     0       NAO TRANSMITE
 * 0      0     1       SENSOR 1
 * 0      1     1       SENSOR 2
 * 1      1     1       SENSOR 3
 * 1      0     0       SENSOR 4
 * 1      0     1       SENSOR 5
 */


//THIS FUNCTION RECEIVE DE VOLTAGE VALUE, CALCULATE THE PROPORTIONAL VALUE AND WRITE THIS VALUE IN TWO PWM PORTS
void ReadVoltageToPWM(void){
  //READ THE VOLTAGE VALUE FROM THE PLC
  volt1 = analogRead(PWM_AD1);
  volt2 = analogRead(PWM_AD2);

  //CALCULATE THE PROPOTIONAL VALUE 0-1024 TO 0-255
  volt1 = map(volt1, ADC_MIN_BITS, ADC_MAX_BITS, PWM_MIN , PWM_MAX);
  volt2 = map(volt2, ADC_MIN_BITS, ADC_MAX_BITS, PWM_MIN , PWM_MAX);

  if(volt1 < MIN_VALUE_PWM){
    volt1 = 0;
  }

  if(volt2 < MIN_VALUE_PWM){
    volt2 = 0;
  }

  //WRITE THE PWM VALUE IN TWO PORTS
  analogWrite(volt1, PWM_MOTOR1);
  analogWrite(volt2, PWM_MOTOR2);
}

//THIS FUNCTION READ THE VALUE OF TERMOCOUPLE CHOSEN AND RETURN THIS VALUE
float ReadTemp(int _sensor){
  switch (_sensor)
  {
  case 1:
    temp = ktc_1.readCelsius();
    return temp;
    break;

  case 2:
    temp = ktc_2.readCelsius();
    return temp;
    break;

  case 3:
    temp = ktc_3.readCelsius();
    return temp;
    break;

  case 4:
    temp = ktc_4.readCelsius();
    return temp;
    break;

  case 5:
    temp = ktc_5.readCelsius();
    return temp;
    break;

  default:
    return 0;
    break;
  }
}

//CHECK IF THE RESISTANCES ARE ENABLED, IF YES ACTIVATE DE RELAYS
void CheckResistances(void){
  res_state_1 = digitalRead(AR1);
  res_state_2 = digitalRead(AR2);

//RESISTANCE 1, USE 2 RELAYS
  if(res_state_1 == HIGH){
    digitalWrite(R1A, HIGH);  //RELAY A ENABLED
    digitalWrite(R1B, HIGH);  //RELAY B ENABLED
  }else{
    digitalWrite(R1A, LOW);   //RELAY A DISABLED
    digitalWrite(R1A, LOW);   //RELAY B DISABLED
  }

//RESISTANCE 2, USE 2 RELAYS
  if(res_state_2 == HIGH){
    digitalWrite(R2A, HIGH);  //RELAY C ENABLED
    digitalWrite(R2B, HIGH);  //RELAY D ENABLED
  }else{
    digitalWrite(R2A, LOW);   //RELAY C DISABLED
    digitalWrite(R2A, LOW);   //RELAY D DISABLED
  }
}

//ENVIA PARA A PORTA (VIN 0)
//SEND THE VALUE OF TEMPERATURE TO PLC
void SendToCLP(float temp_to_send){
  temp_to_send = map(temp_to_send, MIN_TEMPERATURE, MAX_TEMPERATURE, PWM_MIN, PWM_MAX);
  analogWrite(temp_to_send, TEMP_TO_CLP_PIN);
}

//THIS FUNCTION IS FOR TESTING ONLY
void SendDataSerial(void){
  if(!COLLECT_DATA){
    PRINT("ESTADO: ");
    PRINT(state_1);
    PRINT(state_2);
    PRINT(state_3);

    PRINTN();

    PRINT("N° Sensor: ");
    PRINT(number_sensor);

    PRINTN();

    PRINT("PWM_1: ");
    PRINT(volt1);
    PRINT("PWM_2: ");
    PRINT(volt2);

    PRINTN();

    PRINT("RESISTENCE 1: ");
    if(res_state_1 == HIGH){
      PRINT("ON");
    }else{
      PRINT("OFF");
    }
    
    PRINTN();

    PRINT("RESISTENCE 2: ");
    if(res_state_2 == HIGH){
      PRINT("ON");
    }else{
      PRINT("OFF");
    }

    PRINTN();
  }
}

//SEND ON SERIAL ONLY IF THE "COLLECT_DATA" DIRECTIVE IN THE "constants.h" MODULE IS ACTIVATED
//THIS FUNCTION IS USED ONLY WHEN SENSOR DATA IS REQUIRED
void CollectData(void){
  if(COLLECT_DATA){
    PRINT(ktc_1.readCelsius());
    PRINT(";");
    PRINT(ktc_2.readCelsius());
    PRINT(";");
    PRINT(ktc_3.readCelsius());
    PRINT(";");
    PRINT(ktc_4.readCelsius());
    PRINT(";");
    PRINT(ktc_5.readCelsius());
    PRINT(";");

    PRINTN();
  }
}


bool WatcherTemperature(void){
  if(ktc_1.readCelsius() > MAX_TEMPERATURE_TEMP1){
    DisableResistances();
    EnableAlarm(1);
    return 1;
  }
  if(ktc_2.readCelsius() > MAX_TEMPERATURE_TEMP2){
    DisableResistances();
    EnableAlarm(1);
    return 1;
  }
  if(ktc_3.readCelsius() > MAX_TEMPERATURE_TEMP3){
    DisableResistances();
    EnableAlarm(1);
    return 1;
  }
  if(ktc_4.readCelsius() > MAX_TEMPERATURE_TEMP4){
    DisableResistances();
    EnableAlarm(1);
    return 1;
  }
  if(ktc_5.readCelsius() > MAX_TEMPERATURE_TEMP5){
    DisableResistances();
    EnableAlarm(1);
    return 1;
  }
  return 0;
}

void DisableResistances(void){
  digitalWrite(R1A, LOW);
  digitalWrite(R1B, LOW);
  digitalWrite(R2A, LOW);
  digitalWrite(R2B, LOW);
}

void EnableAlarm(bool buzz){
  if(buzz){
    tone(BUZZER, 1500);
  }else{
    noTone(BUZZER);
  }
}
