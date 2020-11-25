#include <SPI.h>

// supply voltage measure
#define voltage_sens          A1
float supply_voltage;

// camera box light bars
#define light_bar_1           4
#define light_bar_2           5
#define light_bar_3           6
#define light_bar_4           7

// signal light tower
#define signal_light_1       14
#define signal_light_2       15
#define signal_light_3       16

// limit switches
#define limit_switch_1       22
#define limit_switch_2       24

// servo
#define servo_pin            23

// SPI
#define MOSI_PIN             51 //SDI
#define MISO_PIN             50 //SDO
#define SCK_PIN              52 //SCK

/* STEPPERS */

// Stepper 1 / Lift
#define LIFT_EN_PIN          53 //enable 
#define LIFT_DIR_PIN         43 //direction
#define LIFT_STEP_PIN        45 //step
#define LIFT_CS_PIN          49 //CS chip select

//Stepper 2 / Pusher
#define PUSHER_EN_PIN        39 //enable 
#define PUSHER_DIR_PIN       25 //direction
#define PUSHER_STEP_PIN      27 //step
#define PUSHER_CS_PIN        33 //CS chip select

//TMC2130 registers
#define WRITE_FLAG           (1<<7) //write flag
#define READ_FLAG            (0<<7) //read flag
#define REG_GCONF            0x00
#define REG_GSTAT            0x01
#define REG_IHOLD_IRUN       0x10
#define REG_CHOPCONF         0x6C
#define REG_COOLCONF         0x6D
#define REG_DCCTRL           0x6E
#define REG_DRVSTATUS        0x6F


volatile byte state_limit_switch_1 = LOW;
volatile byte state_limit_switch_2 = LOW;

void setup() {
  // put your setup code here, to run once:
  pinMode(light_bar_1, OUTPUT);
  pinMode(light_bar_2, OUTPUT);
  pinMode(light_bar_3, OUTPUT);
  pinMode(light_bar_4, OUTPUT);
  
  pinMode(signal_light_1, OUTPUT);
  pinMode(signal_light_2, OUTPUT);
  pinMode(signal_light_3, OUTPUT);
  
  pinMode(limit_switch_1, INPUT_PULLUP);
  pinMode(limit_switch_2, INPUT_PULLUP);

  digitalWrite(signal_light_1, LOW);
  digitalWrite(signal_light_2, LOW);
  digitalWrite(signal_light_3, LOW);

  
  // lift
  pinMode(LIFT_EN_PIN, OUTPUT);
  digitalWrite(LIFT_EN_PIN, HIGH); //deactivate driver (LOW active)
  pinMode(LIFT_DIR_PIN, OUTPUT);
  digitalWrite(LIFT_DIR_PIN, LOW); //LOW or HIGH
  pinMode(LIFT_STEP_PIN, OUTPUT);
  digitalWrite(LIFT_STEP_PIN, LOW);
  pinMode(LIFT_CS_PIN, OUTPUT);
  digitalWrite(LIFT_CS_PIN, HIGH);

  //pusher
  pinMode(PUSHER_EN_PIN, OUTPUT);
  digitalWrite(PUSHER_EN_PIN, HIGH); //deactivate driver (LOW active)
  pinMode(PUSHER_DIR_PIN, OUTPUT);
  digitalWrite(PUSHER_DIR_PIN, HIGH); // HIGH = pulling, LOW = pushing
  pinMode(PUSHER_STEP_PIN, OUTPUT);
  digitalWrite(PUSHER_STEP_PIN, LOW);
  pinMode(PUSHER_CS_PIN, OUTPUT);
  digitalWrite(PUSHER_CS_PIN, HIGH);

  pinMode(MOSI_PIN, OUTPUT);
  digitalWrite(MOSI_PIN, LOW);
  pinMode(MISO_PIN, INPUT);
  digitalWrite(MISO_PIN, HIGH);
  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, LOW);

  pinMode(voltage_sens, INPUT);


  //attachInterrupt(digitalPinToInterrupt(limit_switch_1), limit_switch_1_trigger, FALLING);
  //attachInterrupt(digitalPinToInterrupt(limit_switch_2), limit_switch_2_trigger, FALLING);

  Serial.begin(9600);

  Serial.println("Starting Setup");
  circle_blink();
  //blink_three_times();


  digitalWrite(light_bar_1, HIGH);
  digitalWrite(light_bar_2, HIGH);
  digitalWrite(light_bar_3, HIGH);
  digitalWrite(light_bar_4, HIGH);

  digitalWrite(signal_light_1, HIGH);
  delay(2000);
  digitalWrite(signal_light_2, HIGH);
  delay(2000);
  digitalWrite(signal_light_3, HIGH);

  Serial.println("Setup finished");
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000UL, MSBFIRST, SPI_MODE3));

  //set TMC2130 config
  tmc_write(PUSHER_CS_PIN, WRITE_FLAG|REG_GCONF,      0x00000001UL); //voltage on AIN is current reference
  tmc_write(PUSHER_CS_PIN, WRITE_FLAG|REG_IHOLD_IRUN, 0x00001010UL); //IHOLD=0x10, IRUN=0x10
  tmc_write(PUSHER_CS_PIN, WRITE_FLAG|REG_CHOPCONF,   0x02008008UL); //native 256 microsteps, MRES=0, TBL=1=24, TOFF=8
  
  /* REG_CHOPCONF Values
    0x00008008UL -> 256 microsteps, MRES=0, TBL=1=24, TOFF=8 (native)
    0x01008008UL -> 128 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x02008008UL ->  64 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x04008008UL ->  16 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x03008008UL ->  32 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x06008008UL ->   4 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x05008008UL ->   8 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x07008008UL ->   2 microsteps, MRES=0, TBL=1=24, TOFF=8
    0x08008008UL ->   1 microsteps, MRES=0, TBL=1=24, TOFF=8
  */

  tmc_write(LIFT_CS_PIN, WRITE_FLAG|REG_GCONF,      0x00000001UL); //voltage on AIN is current reference
  tmc_write(LIFT_CS_PIN, WRITE_FLAG|REG_IHOLD_IRUN, 0x00001010UL); //IHOLD=0x10, IRUN=0x10
  tmc_write(LIFT_CS_PIN, WRITE_FLAG|REG_CHOPCONF,   0x00008008UL); //native 256 microsteps, MRES=0, TBL=1=24, TOFF=8

  digitalWrite(PUSHER_EN_PIN, LOW);
  digitalWrite(LIFT_EN_PIN, LOW);

}



void loop() {

  static uint32_t last_time_10s=0;
  static uint32_t last_time_0_5s=0;
  uint32_t ms = millis();
    
  if((ms-last_time_10s) > 10000) //run every 1s
  {
    last_time_10s = ms;
    print_tmc_debug(PUSHER_CS_PIN);
    print_tmc_debug(LIFT_CS_PIN);
  }

  if((ms-last_time_0_5s) > 500) { // run every half second
    supply_voltage = analogRead(voltage_sens) * 0.0271267 ;
    if(supply_voltage < 10) {
      Serial.println("12V Spannung nicht online. Breche ab");
      while(true);  
    }
    last_time_0_5s = ms;
  }

  if(digitalRead(limit_switch_1) == HIGH) {
    digitalWrite(LIFT_STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(LIFT_STEP_PIN, LOW);
    delayMicroseconds(10);

  } else {
    Serial.println("Endschalter 1 hat ausgelöst");
  }

  if(digitalRead(limit_switch_2) == HIGH) {
      digitalWrite(PUSHER_STEP_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(PUSHER_STEP_PIN, LOW);
      delayMicroseconds(10);

  } else {
    Serial.println("Endschalter 2 hat ausgelöst");
  }
}



void limit_switch_1_trigger() {
  state_limit_switch_1 = HIGH;
}

void limit_switch_2_trigger() {
  state_limit_switch_2 = HIGH;
}



void blink_three_times() {
  for ( int i = 0; i < 3; i++){
      digitalWrite(light_bar_1, HIGH);
      digitalWrite(light_bar_2, HIGH);
      digitalWrite(light_bar_3, HIGH);
      digitalWrite(light_bar_4, HIGH);
      delay(500);
      digitalWrite(light_bar_1, LOW);
      digitalWrite(light_bar_2, LOW);
      digitalWrite(light_bar_3, LOW);
      digitalWrite(light_bar_4, LOW);
      delay(500);
    }
}

void circle_blink() {
  int delay_time = 200;
  while (delay_time > 0) {
    digitalWrite(light_bar_1, HIGH);
    delay(delay_time);
    digitalWrite(light_bar_1, LOW);
    digitalWrite(light_bar_2, HIGH);
    delay(delay_time);
    digitalWrite(light_bar_2, LOW);
    digitalWrite(light_bar_3, HIGH);
    delay(delay_time);
    digitalWrite(light_bar_3, LOW);
    digitalWrite(light_bar_4, HIGH);
    delay(delay_time);
    digitalWrite(light_bar_4, LOW);
    delay_time = delay_time*0.9;
  }
}

uint8_t tmc_write(uint8_t CS_PIN, uint8_t cmd, uint32_t data)
{
  uint8_t s;

  digitalWrite(CS_PIN, LOW);

  s = SPI.transfer(cmd);
  SPI.transfer((data>>24UL)&0xFF)&0xFF;
  SPI.transfer((data>>16UL)&0xFF)&0xFF;
  SPI.transfer((data>> 8UL)&0xFF)&0xFF;
  SPI.transfer((data>> 0UL)&0xFF)&0xFF;

  digitalWrite(CS_PIN, HIGH);

  return s;
}

uint8_t tmc_read(uint8_t CS_PIN, uint8_t cmd, uint32_t *data)
{
  uint8_t s;

  tmc_write(CS_PIN, cmd, 0UL); //set read address

  digitalWrite(CS_PIN, LOW);

  s = SPI.transfer(cmd);
  *data  = SPI.transfer(0x00)&0xFF;
  *data <<=8;
  *data |= SPI.transfer(0x00)&0xFF;
  *data <<=8;
  *data |= SPI.transfer(0x00)&0xFF;
  *data <<=8;
  *data |= SPI.transfer(0x00)&0xFF;

  digitalWrite(CS_PIN, HIGH);

  return s;
}

void print_tmc_debug(uint8_t CS_PIN) {

    uint32_t data;
    uint8_t s;
    String name;

    if (CS_PIN == PUSHER_CS_PIN) {
      name = "Pusher";
    } else if (CS_PIN == LIFT_CS_PIN) {
      name = "Lift";
    }

    Serial.print("============== ");
    Serial.print(name);
    Serial.println(" ==============");

    //show REG_GSTAT
    s = tmc_read(CS_PIN, REG_GSTAT, &data);
    Serial.print("GSTAT:     0x0");
    Serial.print(data, HEX);
    Serial.print("\t - ");
    Serial.print("Status: 0x");
    Serial.print(s, HEX);
    if(s & 0x01) Serial.print(" reset");
    if(s & 0x02) Serial.print(" error");
    if(s & 0x04) Serial.print(" sg2");
    if(s & 0x08) Serial.print(" standstill");
    Serial.println(" ");

    //show REG_DRVSTATUS
    s = tmc_read(CS_PIN, REG_DRVSTATUS, &data);
    Serial.print("DRVSTATUS: 0x");
    Serial.print(data, HEX);
    Serial.print("\t - ");
    Serial.print("Status: 0x");
    Serial.print(s, HEX);
    if(s & 0x01) Serial.print(" reset");
    if(s & 0x02) Serial.print(" error");
    if(s & 0x04) Serial.print(" sg2");
    if(s & 0x08) Serial.print(" standstill");
    Serial.println(" ");
}
