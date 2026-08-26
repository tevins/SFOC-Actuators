/**
 *
 * Velocity motion control example
 * Steps:
 * 1) Configure the motor and magnetic sensor
 * 2) Run the code
 * 3) Set the target velocity (in radians per second) from serial terminal
 *
 *
 * By using the serial terminal set the velocity value you want to motor to obtain
 *
 */
#include "Arduino.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/MT6701/MagneticSensorMT6701SSI.h"

#define ENC_SDO PA6  // configured as MISO
#define ENC_NC PA7   // Not connected (dummy MOSI)
#define ENC_CLK PA5
#define ENC_CS PA4

#define IndicatorLED PB9  // 指示灯

//  Use SPI_2 for sensor since default SPI will be used by DRV8316C
//SPIClass SPI_2(ENC_NC, ENC_SDO, ENC_CLK);
MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(ENC_CS);

//#define SENSOR1_CS PA4 // some digital pin that you're using as the nCS pin
//MagneticSensorMT6701SSI sensor1(SENSOR1_CS);
//MagneticSensorMT6701SSI sensor = MagneticSensorMT6701SSI(SENSOR1_CS);
// magnetic sensor instance - SPI
//MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
// magnetic sensor instance - MagneticSensorI2C
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

// BLDC motor & driver instance
//BLDCMotor motor = BLDCMotor(7);
// PM2805
BLDCMotor motor = BLDCMotor(7);
//4315
//BLDCMotor motor = BLDCMotor(14,16.5,150);
//BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 7);
//BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA7, PB0, PB12);
// 2205
//BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA7, PB0, PB1);
// dian ji
BLDCDriver3PWM driver = BLDCDriver3PWM(PA3, PB0, PA8, PB1);
// 关节电机
//BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA7, PA8, PB12);

// Stepper motor & driver instance
//StepperMotor motor = StepperMotor(50);
//StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);

// velocity set point variable
float target_velocity = 6 ;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
//void onMotor(char* cmd){ command.motor(&motor,cmd); } 

void setup() {

  // initialise magnetic sensor hardware
  //sensor.init();

  sensor.init(&SPI);  //  Initialize sensor on SPI_2 bus
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // set motion control loop to be used
  motor.controller = MotionControlType::velocity;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.3;
  motor.PID_velocity.I = 15;
  motor.PID_velocity.D = 0;
  // default voltage_power_supply
  //motor.voltage_limit = 24;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering
  // default 5ms - try different values to see what is the best.
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  motor.voltage_limit = 8; // Volts - default driver.voltage_limit
// of current 
  motor.current_limit = 1.5; // Amps - default 0.2Amps

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  // add target command T
  command.add('T', doTarget, "target voltage");
  //command.add('M',onMotor,"my motor"); 

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

  pinMode(IndicatorLED,OUTPUT);
  digitalWrite(IndicatorLED, HIGH);


  _delay(1000);
}

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  //motor.monitor();

  // user communication
  command.run();
}
