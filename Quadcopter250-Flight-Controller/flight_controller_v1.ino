  #include <AP_Common.h>
  #include <AP_Math.h>
  #include <AP_Param.h>
  #include <AP_Progmem.h>
  #include <AP_ADC.h>
  #include <AP_InertialSensor.h>
  
  #include <AP_HAL.h>
  #include <AP_HAL_AVR.h>
  
  #include <PID.h>
  
  const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;  // Hardware abstraction layer
  
  //rewriting the map function from Arduino
  long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
  
  //wrap function for yaw
  #define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
  
  
  //creating the Inertial Sensor
  AP_InertialSensor_MPU6000 ins;

  PID pids[6];
  #define PID_PITCH_RATE 0
  #define PID_ROLL_RATE 1
  #define PID_PITCH_STAB 2
  #define PID_ROLL_STAB 3
  #define PID_YAW_RATE 4
  #define PID_YAW_STAB 5


  //defining the motor outputs
  #define MOTOR_FL   0    // Front left    
  #define MOTOR_FR   3    // Front right
  #define MOTOR_BL   2    // back left
  #define MOTOR_BR   1    // back right

//setup function that can be run on each uart
static void setup_uart(AP_HAL::UARTDriver *uart, const char *name)
  {
      if (uart == NULL) {
          // that UART doesn't exist on this platform
          return;
      }
      uart->begin(57600);
  }
  
  
  ////////* SETUP */////////
  
void setup() 
  {
    setup_uart(hal.uartA, "uartA"); // console
    setup_uart(hal.uartB, "uartB"); // 1st GPS
    setup_uart(hal.uartC, "uartC"); // telemetry 1
    hal.uartA->println("Initializing console (uartA)");

    //setup the outputs
    hal.rcout->set_freq(0xF, 490);
    hal.rcout->enable_mask(0xFF);
    
    //setup the Inertial Sensor
    // Disable barometer to stop it corrupting bus
    hal.gpio->pinMode(40, GPIO_OUTPUT);
    hal.gpio->write(40, 1);
    
    // Initialise MPU6050 sensor
    ins.init(AP_InertialSensor::COLD_START,  
    		 AP_InertialSensor::RATE_100HZ,
    		 NULL);
    
    // Initialise MPU6050's internal sensor fusion (aka DigitalMotionProcessing)
    hal.scheduler->suspend_timer_procs();  // stop bus collisions
    ins.dmp_init();                        
    ins.push_gyro_offsets_to_dmp();
    hal.scheduler->resume_timer_procs();
    
    //Set up the PID gains
    pids[PID_PITCH_RATE].kP(0.66); //0.87
    pids[PID_PITCH_RATE].kI(1.15); //1.26
    pids[PID_PITCH_RATE].imax(50);
    
    pids[PID_ROLL_RATE].kP(0.44); //0.47 - note that 0.48 oscillates
    pids[PID_ROLL_RATE].kI(0.76); //0.8
    pids[PID_ROLL_RATE].imax(50);
    
    pids[PID_YAW_RATE].kP(1.1);
    pids[PID_YAW_RATE].kI(0);
    pids[PID_YAW_RATE].imax(50);
    
    pids[PID_PITCH_STAB].kP(3);
    pids[PID_ROLL_STAB].kP(3);
    pids[PID_YAW_STAB].kP(5);
    pids[PID_PITCH_STAB].kI(1);
    pids[PID_ROLL_STAB].kI(1);
  
    
    hal.uartA->println("...Finished setup");
  }
  
  //initializing variables that are used in the loop
  float roll, pitch, yaw;
  int counter = 0;
  float yaw_target = 0;
  
  void loop() {
    
    counter++;
    
    //wait until new inertial data available
    while (ins.num_samples_available() == 0);
    
    long ch0 = hal.rcin->read(0);
    long ch1 = hal.rcin->read(1);
    long ch2 = hal.rcin->read(2);
    long ch3 = hal.rcin->read(3);
    
    long rcthr, rcyaw, rcpit, rcroll;   // Variables to store rc input
      rcthr = map(ch2, 1067, 1900, 1000, 2000);
      rcyaw = map(ch3, 1061, 1900, -150, 150);
      rcpit = -1 * map(ch1, 1062, 1901, -45, 45);
      rcroll = map(ch0, 1058, 1898, -45, 45);
    
    /*
    if (counter>20) {
      counter = 0;
      hal.uartA->print(rcroll);
      hal.uartA->print(" | ");
      hal.uartA->print(rcpit);
      hal.uartA->print(" | ");
      hal.uartA->print(rcthr);
      hal.uartA->print(" | ");
      hal.uartA->print(rcyaw);
      hal.uartA->println();
    }
    */
    
    
    ins.update();
    ins.quaternion.to_euler(&roll, &pitch, &yaw);
    roll = ToDeg(roll)-4.9;
    pitch = ToDeg(pitch)+3.9;
    yaw = ToDeg(yaw);

  /*
    if (counter > 20) {
      counter = 0;
      hal.uartA->printf_P(
    	  PSTR("P:%4.1f  R:%4.1f Y:%4.1f\n"),
    			  pitch,
    			  roll,
    			  yaw);
  }
    
   */
    
    Vector3f gyro = ins.get_gyro();
    float gyroPitch = ToDeg(gyro.y)+1.46, gyroRoll = ToDeg(gyro.x)-1.46, gyroYaw = ToDeg(gyro.z);
    
    /*
    if (counter>20) {
      counter = 0;
      hal.uartA->print("\n GPit: ");
      hal.uartA->print(gyroPitch);
      hal.uartA->print("\n GRoll: ");
      hal.uartA->print(gyroRoll);
      hal.uartA->print("\n GYaw: ");
      hal.uartA->print(gyroYaw);
    }
    */
    
    
    if (rcthr > 1800) { //do not allow throttle to exceed 90%
                        //so that the quad can still stabilize at full stick
      rcthr = 1800;
    }

    if (rcthr > 1170) {
	
        //long pitch_output = pids[PID_PITCH_RATE].get_pid(gyroPitch - rcpit, 1);  
	//long roll_output = pids[PID_ROLL_RATE].get_pid(gyroRoll - rcroll, 1);  
	//long yaw_output = pids[PID_YAW_RATE].get_pid(gyroYaw - rcyaw, 1);  


        float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
        float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
        //float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid((float)rcyaw - yaw, 1), -360, 360);
  
        float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
        
        if (abs(rcyaw) > 5) { //if pilot commands a change in yaw
          yaw_stab_output = rcyaw;
          yaw_target = yaw;
        }
        

        // rate PIDS
        long pitch_output =  (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), -250, 250);  
        long roll_output =  (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -250, 250);  
        long yaw_output =  (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -250, 250);  
/*
        if (counter>20) {
          counter = 0;
          hal.uartA->print(pitch_output);
          hal.uartA->print(" | ");
          hal.uartA->print(roll_output);
          hal.uartA->print(" | ");
          hal.uartA->print(yaw_output);
          hal.uartA->print(" | ");
          hal.uartA->println();
        }
*/

          hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output + yaw_output);
  	  hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output - yaw_output);
  	  hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output - yaw_output);
  	  hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output + yaw_output);
        
    } else {  // MOTORS OFF
	hal.rcout->write(MOTOR_FL, 1000);
	hal.rcout->write(MOTOR_BL, 1000);
	hal.rcout->write(MOTOR_FR, 1000);
	hal.rcout->write(MOTOR_BR, 1000);

        yaw_target = yaw;
    }
}

AP_HAL_MAIN(); // special macro that replace's one of Arduino's to setup the code (e.g. ensure loop() is called in a loop).


