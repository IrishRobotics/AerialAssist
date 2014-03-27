package edu.wpi.first.wpilibj.defaultCode;

//import edu.wpi.first.wpilibj.Accelerometer;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
//import edu.wpi.first.wpilibj.Kinect;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Relay;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DefaultRobot extends IterativeRobot {

    // Declare variable for the robot drive system
    RobotDrive m_robotDrive;		// robot will use PWM 1-4 for drive motors

    //extra motor
    // Jaguar extra;
    Jaguar winder; //l and r windy motors
    Jaguar mouseTrap;

    // value sending to spikes controlling: hook and mousetrap arms
    Relay hook;
    Relay r_mouseTrapFeeder;

    // Declare variables for the two joysticks being used
    Joystick joystick;			// joystick 1 (arcade stick or right tank stick)
    // Declare variables for each of the eight solenoid outputs
    static final int NUM_SOLENOIDS = 8;
    Solenoid[] m_solenoids = new Solenoid[NUM_SOLENOIDS];

    //print to drivestation
    DriverStationLCD TV;
    DriverStation DS;
    SmartDashboard SDB;

    // Local variables to count the number of periodic loops performed
    int m_autoPeriodicLoops;
    int m_disabledPeriodicLoops;

    int teleopStartSec;
    
    //Switches
    //final private DigitalInput switch1;
    DigitalInput hook_forward, hook_backward; // switches for controling the hook
    
    char hookDirection, mouseTrapDirection;
    double lastPressY, lastPressA, lastPressB;
    
    DigitalInput tensionSensor; // when nothing in front of it = true, when something front = false;
    
    /**
     * Constructor for this "BuiltinDefaultCode" Class.
     *
     * The constructor creates all of the objects used for the different inputs
     * and outputs of the robot. Essentially, the constructor defines the
     * input/output mapping for the robot, providing named objects for each of
     * the robot interfaces.
     */
    public DefaultRobot() {
        System.out.println("BuiltinDefaultCode Constructor Started=========================================\n ");
        //ranger = new MaxbotixUltrasonic(1);//ultrasonic
        // drive on PWMS 1, 2
        m_robotDrive = new RobotDrive(1, 2);//robot drive
        joystick = new Joystick(1); //XBox controller
        hook_forward = new DigitalInput(5);//switch checking if hook set forward 
        hook_backward = new DigitalInput(2);//switch checking if hook set backward
      //  sw_Tension = new DigitalInput(6); //switch to read if tensioned
        tensionSensor = new DigitalInput(8);
        hook = new Relay(4); // reverse = forward move
        
        mouseTrap = new Jaguar(7);// reads signal from victor/(spike) in pwm 4
        
        //mouseTrap.set(0.0);
        gyro = new Gyro(2);
        r_mouseTrapFeeder = new Relay(5);
        //switches to control the mouse trap
        //sw_mouseTrapForward = new DigitalInput(8); 
        //sw_mouseTrapBackward = new DigitalInput(9);

        // int
        lastPressY = -1;
        lastPressA = -1;

        winder = new Jaguar(3);
        
        hookDirection = '*';
        mouseTrapDirection = '*';
       
        // Iterate over all the solenoids on the robot, constructing each in turn
        int solenoidNum;						// start counting solenoids at solenoid 1
        for (solenoidNum = 0; solenoidNum < NUM_SOLENOIDS; solenoidNum++) {
            m_solenoids[solenoidNum] = new Solenoid(solenoidNum + 1);
        }

        //drivestation
        TV = DriverStationLCD.getInstance();
        DS = DriverStation.getInstance();
        SDB = new SmartDashboard();
        // Initialize counters to record the number of loops completed in autonomous and teleop modes
        m_autoPeriodicLoops = 0;
        m_disabledPeriodicLoops = 0;

        System.out.println("BuiltinDefaultCode Constructor Completed\n");
    }

    /**
     * ******************************** Init Routines
     * ************************************
     */
    public void robotInit() {
        // Actions which would be performed once (and only once) upon initialization of the
        // robot would be put here.
        System.out.println("RobotInit() completed.\n ");
    }

    public void disabledInit() {
        m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
        startSec = (int) (Timer.getUsClock() / 1000000.0);
        printSec = startSec + 1;
    }
    public double startTimeAuto = 0;

    public void autonomousInit() {
        m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
        startTimeAuto = (Timer.getUsClock() / 1000000.0);
        //gyro.reset();
    }

    public void teleopInit() {
        System.out.println("teleopInit() started");

        startTimeAuto = (Timer.getUsClock() / 1000000.0);
        //TODO: Turn on during production.
        r_mouseTrapFeeder.set(Relay.Value.kReverse);
        gyro.reset();
        //mouseTrapBackward();
        //hookForward();
    }

    /**
     * ******************************** Periodic Routines
     * ************************************
     */
    static int printSec;
    static int startSec;

    /**
     * clear every line of the driver station
     */
    public void clearScreen() {
        //TV.println(DriverStationLCD.Line.kMain6, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser1, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser2, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser3, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser4, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser5, 1, "                                       ");
        TV.println(DriverStationLCD.Line.kUser6, 1, "                                       ");

    }

    /**
     * Returns the current time in seconds
     *
     * @return A representation of time in seconds.
     */
    public double getTimeInSeconds() {
        return (Timer.getUsClock() / 1000000.0);
    }

    public void disabledPeriodic() {
        //update driverstation
        TV.updateLCD();
        // feed the user watchdog at every period when disabled
        Watchdog.getInstance().feed();

        // increment the number of disabled periodic loops completed
        m_disabledPeriodicLoops++;

        // while disabled, printout the duration of current disabled mode in seconds;
        if ((Timer.getUsClock() / 1000000.0) > printSec) {
            System.out.println("Disabled seconds: " + (printSec - startSec));
            printSec++;
        }

    }

    public void autonomousPeriodic() {
        //update driverstation
        boolean waiting_for_switch = true, was_down = false; //values for when switch first pressed and if already pressed
        // feed the user watchdog at every period when in autonomous
        Watchdog.getInstance().feed();
        //for(int i = 0; i = 1; i++){
        clearScreen();
        //go forward for 5 seconds, 
        double seconds = getTimeInSeconds() - startTimeAuto;
        if(seconds < 1){
           winder.set(-1.0);
        }else if(seconds < 3.3){
            winder.set(1.0);
        }else{
            winder.set(0.0);
        }
        if(seconds < 3) {
            m_robotDrive.tankDrive(-0.5,-0.5);
        } else{
            m_robotDrive.tankDrive(0,0);/*
            if(!hook_forward.get()&& !was_down){
                hookForward();
            }else if(was_down){
                //does nothing if the ball was fired
            }else if(hook_forward.get()){
                if(!sw_Tension.get()){
                    winder.set(1.0);
                }else{
                    was_down = true;
                    onTensionTriggered();
                }
            }*/
        }
        
        m_autoPeriodicLoops++;
        TV.updateLCD();

    }
    double adjustmentAngle = 0.0;
   

    private void checkLimitSwitches() {
    	/* The Hook Switches go "low" when they have their
	 *  IR signal "broken." */

        /*
        System.out.print("hookDirection ");
        System.out.print(hookDirection);
        System.out.print(" Hook Forward: ");
        System.out.print(hook_forward.get());
        System.out.print(" Hook Backward: ");
        System.out.println(hook_backward.get());
        
       */
	if(!hook_forward.get() && (hookDirection == '*' || hookDirection == 'F')) {
            hook.set(Relay.Value.kOff);
        }
	if(!hook_backward.get() && (hookDirection == '*' || hookDirection == 'B')) {
            hook.set(Relay.Value.kOff);
        }
        if(/*sw_mouseTrapForward.get() &&*/ (mouseTrapDirection == '*' || mouseTrapDirection == 'F')) {
            mouseTrap.set(0.0);
          //  mouseTrap.set(0.0);
        }
        if(/*sw_mouseTrapBackward.get() &&*/ (mouseTrapDirection == '*' || mouseTrapDirection == 'B')) {
           // mouseTrap.set(0.0);
           mouseTrap.set(0.0);
        }
    }

    public void teleopPeriodic() {
        // feed the user watchdog at every period when in autonomous
        Watchdog.getInstance().feed();
        
        double seconds = getTimeInSeconds() - startTimeAuto;
        boolean mouseTrapOverRideRunning = false;
        
        double direction = joystick.getRawAxis(3);
        double left = joystick.getRawAxis(5);
        double right = joystick.getRawAxis(2);
        if(direction < -0.5) {
            direction = 1.0;
        } else {
            direction = -1.0;
            double x = left;
            left = right;
            right = x;
        }
        m_robotDrive.tankDrive(direction*left, direction*right);

        clearScreen();
        /* stop motors if they've hit their switches */
        checkLimitSwitches();
        //set hook light on or off
        // increment the number of teleop periodic loops completed

      //  TV.println(DriverStationLCD.Line.kUser1, 1, "hook is back: " + hook_backward.get());
        /*
        TV.println(DriverStationLCD.Line.kUser1, 1, "moustrap is back: " + sw_mouseTrapBackward.get());
        TV.println(DriverStationLCD.Line.kUser2, 1, "moustrap is forward: " + sw_mouseTrapForward.get());
        TV.println(DriverStationLCD.Line.kUser3, 1, "hook is forward: " + hook_forward.get());
        */
       // TV.println(DriverStationLCD.Line.kUser1, 1, "xAxis: " + joystick.getRawAxis(5));
       // TV.println(DriverStationLCD.Line.kUser2, 1, "yAxis: " + joystick.getRawAxis(2));
        
      double currentTime = getTimeInSeconds();

      // Button 1 == A
      if(joystick.getRawButton(1) && (currentTime - lastPressA) > 1.0){ 
        System.out.print("Hook position: ");
        System.out.println(hookPosition());
        lastPressA = currentTime;
        adjustmentAngle += 0.0;
	if(hookPosition() != 'F') {
                // run motor until hook forward is true.
                hookForward();
        } else {
                hookBackward();
        }
      }

      // Button 4 == Y
      if(joystick.getRawButton(4)) {
          mouseTrap.set(-1.0);
      // Button 3 == X
      } else if(joystick.getRawButton(3)) {
          mouseTrap.set(1.0);
      } else {
         // mouseTrap.set(Relay.Value.kOff);
          mouseTrap.set(0.0);
      }
      // B-button toggle mouseTrapFeeder
      if(joystick.getRawButton(2) && (currentTime - lastPressB) > 1.0){
          lastPressB = currentTime;
          if(r_mouseTrapFeeder.get() != Relay.Value.kOff){
              r_mouseTrapFeeder.set(Relay.Value.kOff);
          } else {
              r_mouseTrapFeeder.set(Relay.Value.kReverse);
          }
       }
      /* Button 7 == "Back" */
      if(joystick.getRawButton(7)) {
          hook.set(Relay.Value.kOff);
                mouseTrap.set(0.0);
                winder.set(0.0);
      }
      
      // Right Bumper, Wind
        if (joystick.getRawButton(6)) {     // right bumper wind
            winder.set(1.0);
        } else if (joystick.getRawButton(5)) {  //left bumper unwind
            winder.set(-1.0);
        } else {
            winder.set(0.0);
        }
        
        if(!tensionSensor.get()) {
           onTensionTriggered();
        }
        TV.println(DriverStationLCD.Line.kUser1, 1, "Hook Position: " + hookPosition());
        TV.println(DriverStationLCD.Line.kUser2,1,  "Launch Sensor: " + tensionSensor.get());
        
        //update driverstation
        TV.updateLCD();
    }
    
    private void onTensionTriggered() {
        //winder.set(0.0);  //stops the winder once sw_Tension so it doesnt over tension while the mousetrap is moving forward  
       // if(hook_forward.get()) {
            hookBackward();
       // }
            winder.set(0.0);
    }

    /** Returns the current position of the hook.
     *  "F" means forward, "B" means "Backward" and "*" means "between"
     */

    private char hookPosition(){
    	if(!hook_forward.get()) { return 'F'; }
	if(!hook_backward.get()) { return 'B'; }
	return '*';
    }
    
    private void hookForward(){
	if(hookPosition() != 'F') {
            hookDirection = 'F';
            hook.set(Relay.Value.kReverse);
        }
    }
    private void hookBackward(){
	if(hookPosition() != 'B') {
            hookDirection = 'B';
            hook.set(Relay.Value.kForward);
        }
    }
    private void mouseTrapForward(){
        if(mouseTrapDirection != 'F') {
            mouseTrapDirection = 'F';
            mouseTrap.set(-1);
        }
    }
    
    private void mouseTrapBackward(){
        if(mouseTrapDirection != 'B') {
            mouseTrapDirection = 'B';
            mouseTrap.set(1);
        }
    }
 
}
