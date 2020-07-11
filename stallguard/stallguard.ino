#include <HardwareSerial.h>
#include <TMCStepper.h>


#define DIAG_PIN_2         19          // STALL motor 2
#define EN_PIN_2           5          // Enable
#define DIR_PIN_2          23          // Direction
#define STEP_PIN_2         14          // Step
#define SERIAL_PORT_2      Serial2    // TMC2208/TMC2224 HardwareSerial port
#define DRIVER_ADDRESS_2   0b00       // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE_2          0.11f      // E_SENSE for current calc.  
#define STALL_VALUE_2      2          // [0..255]

hw_timer_t * timer1 = NULL;
TMC2209Stepper driver2(&SERIAL_PORT_2, R_SENSE_2 , DRIVER_ADDRESS_2 );


void IRAM_ATTR onTimer() {

  digitalWrite(STEP_PIN_2, !digitalRead(STEP_PIN_2));
} 

void setup() {
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");
  SERIAL_PORT_2.begin(115200);
  
  pinMode(DIAG_PIN_2 ,INPUT);
  pinMode(EN_PIN_2 ,OUTPUT);
  pinMode(STEP_PIN_2 ,OUTPUT);
  pinMode(DIR_PIN_2 ,OUTPUT);

  digitalWrite(EN_PIN_2 ,LOW);
  digitalWrite(DIR_PIN_2 ,LOW);

  driver2.begin();
  driver2.toff(4);
  driver2.blank_time(24);
  driver2.rms_current(500); 
  driver2.microsteps(16);
  driver2.TCOOLTHRS(0xFFFFF); // 20bit max
  driver2.semin(0);
  driver2.semax(2);
  driver2.shaft(false);
  driver2.sedn(0b01);
  driver2.SGTHRS(STALL_VALUE_2);

  activate_interrupt();
}

void loop() {
 static uint32_t last_time=0;
 uint32_t ms = millis();
 if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    Serial.print("0 ");
    Serial.print(driver2.SG_RESULT(), DEC);
    Serial.print(" ");
    Serial.println(driver2.cs2rms(driver2.cs_actual()), DEC);
  }
}

void activate_interrupt(){
  {
    cli();//stop interrupts
    timer1 = timerBegin(3, 8,true); // Initialize timer 4. Se configura el timer,  ESP(0,1,2,3)
                                 // prescaler of 8, y true es una bandera que indica si la interrupcion se realiza en borde o en nivel
    timerAttachInterrupt(timer1, &onTimer, true); //link interrupt with function onTimer
    timerAlarmWrite(timer1, 8000, true); //En esta funcion se define el valor del contador en el cual se genera la interrupción del timer
    timerAlarmEnable(timer1);    //Enable timer        
    sei();//allow interrupts
  }
}
