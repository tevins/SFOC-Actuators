  /**
   *
   * Position/angle motion control example
   * Steps:
   * 1) Configure the motor and magnetic sensor
   * 2) Run the code
   * 3) Set the target angle (in radians) from serial terminal
   *
   */
  
  
  #include <MPU6050_tockn.h>
  #include <Wire.h>
  
  #include <SimpleFOC.h>
  // mpu6050
  MPU6050 mpu6050(Wire);
  // mpu6050
  
  
  // magnetic sensor instance - SPI
  //MagneticSensorSPI sensor = MagneticSensorSPI(AS5147_SPI, 10);
  // magnetic sensor instance - MagneticSensorI2C
  MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
  // magnetic sensor instance - analog output
  // MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);
  
  // BLDC motor & driver instance
  BLDCMotor motor = BLDCMotor(7);
  //BLDCDriver3PWM driver = BLDCDriver3PWM(PA1, PA2, PA3, PA4);
  BLDCDriver3PWM driver = BLDCDriver3PWM(PA6, PA7, PB0, PB12);
  //BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);
  // Stepper motor & driver instance
  //StepperMotor motor = StepperMotor(50);
  //StepperDriver4PWM driver = StepperDriver4PWM(9, 5, 10, 6,  8);
  
  // angle set point variable
  float target_angle = 0;
  
  float currentAngle;
  // instantiate the commander
  //Commander command = Commander(Serial);
  //void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }
  //void onMotor(char* cmd){ command.motor(&motor,cmd); } 
  
  void setup() {
    //mpu6050
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    //mpu6050
  
    
    // initialise magnetic sensor hardware
    sensor.init();
    // link the motor to the sensor
    motor.linkSensor(&sensor);
  
    // driver config
    // power supply voltage [V]
    driver.voltage_power_supply = 12;
    driver.init();
    // link the motor and the driver
    motor.linkDriver(&driver);
  
    // choose FOC modulation (optional)
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  
    // set motion control loop to be used
    motor.controller = MotionControlType::angle;
  
    // contoller configuration
    // default parameters in defaults.h
  
    // velocity PI controller parameters
    motor.PID_velocity.P = 0.025;
    motor.PID_velocity.I = 2;
    motor.PID_velocity.D = 0;
    // maximal voltage to be set to the motor
    motor.voltage_limit = 6;
  
    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.03;
  
    // angle P controller
    motor.P_angle.P = 4;
    // maximal velocity of the position control
    motor.velocity_limit = 50;
  
    // use monitoring with serial
    Serial.begin(9600);
    // comment out if not needed
    motor.useMonitoring(Serial);
  
  
    // initialize motor
    motor.init();
    // align sensor and start FOC
    //motor.initFOC(0.91,CCW);
    motor.initFOC();
  
    // add target command T
    //command.add('T', doTarget, "target angle");
    //command.add('M',onMotor,"my motor"); 
  
    Serial.println(F("Motor ready."));
    Serial.println(F("Set the target angle using serial terminal:"));
    _delay(4000);
  }
  
  void loop() {
  

    // main FOC algorithm function
    // the faster you run this function the better
    // Arduino UNO loop  ~1kHz
    // Bluepill loop ~10kHz
    motor.loopFOC();

    
    mpu6050.update();
    currentAngle = mpu6050.getAngleZ()/180*PI;
    Serial.print("target_angle : ");
    Serial.println(target_angle);
    if (currentAngle >= 0 && currentAngle <= PI/4){
     target_angle = currentAngle;
    }  
  
    // Motion control function
    // velocity, position or voltage (defined in motor.controller)
    // this function can be run at much lower frequency than loopFOC() function
    // You can also use motor.move() and set the motor.target in the code
    motor.move(target_angle);
  
  
    // function intended to be used with serial plotter to monitor motor variables
    // significantly slowing the execution down!!!!
    //motor.monitor();
  
    // user communication
   // command.run();
  }
