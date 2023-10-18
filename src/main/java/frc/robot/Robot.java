// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  //sets up autonomous period for later use
  private Command m_autonomousCommand;
  //Robot container
  private RobotContainer m_robotContainer; 

  //Initializes joysticks (driving/operating control)
  public Joystick Xbox = new Joystick(0);
  public Joystick JoyStick1 = new Joystick(1);
  public Joystick Wheel = new Joystick(2);

  //assign Drivetrain motors
  public CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax MiddleLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax BackLeftMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax MiddleRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax BackRightMotor = new CANSparkMax(6, MotorType.kBrushless);


  //DriveTrain encoder initialization 
  public RelativeEncoder LeftEncoder = FrontLeftMotor.getEncoder();
  public RelativeEncoder RightEncoder = FrontRightMotor.getEncoder();


  //Arm Motors
  public CANSparkMax ArmUpOne = new CANSparkMax(7, MotorType.kBrushless);
  public CANSparkMax ArmUpTwo = new CANSparkMax(8, MotorType.kBrushless);

  //extension motors for the arm and floor intake declaration
  public WPI_TalonFX ExtensionMotorOne = new WPI_TalonFX(10);
  public WPI_TalonFX ExtensionMotorTwo = new WPI_TalonFX(11);
  public WPI_TalonFX Intake = new WPI_TalonFX(25);


  //timer Declarations
  public long LastCheckedTime;
  public long CurrentCheckTime;

  //Arm encoders (reads motor ticks and rotations for math)
  public RelativeEncoder ArmOneEncoder;
  public RelativeEncoder ArmTwoEncoder;
  
  //compressor applies suction to air (MUST BE ID 0 cause the system says so) 
  public Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  //solenoid controls the release of air from compressor
  public Solenoid Piston = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public Solenoid IntakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public Solenoid LightSolenoidPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

  //encoder value variables define (used to do math and extension limits)
  //we get from the motors rotation values
  public double LeftEncoderValue;
  public double RightEncoderValue;
  public double ArmOneEncoderValue;
  public double ArmTwoEncoderValue;
  public double AverageEncoderValue;
  public double AverageArmEncoderValue;

  public double AverageDriveTrainOffset;
  //used for drivetrain speed
  public double speed;
  public Timer auto_Timer = new Timer();
  //these variables control extension limit and angle limit for the arm
  //used so our robot is legal and doesn't break the frc guidelines and we don't get banned
  public double maxextensionlimit;
  public double mainlimit;
  public double maxanglelimit = 4;

  // variables to make sure that our motors don't get too much power, cool thersholds
  public boolean enable = true;
  public boolean brakemode = false;
  public double currentLimit = 60;
  public double triggerThresholdCurrent = 60;
  public double triggerThresholdTime = .1;
  public boolean timer_started = false;

  //used to set the new arm extension limit
  //when the arm angles down, the extension limit decreases due to this factor
  public double limitfactor;

  // Gyro declaration (google if needed)
  public WPI_PigeonIMU gyro = new WPI_PigeonIMU(12);

  // claw motors declaration
  public WPI_TalonSRX SRX_1 = new WPI_TalonSRX(15);
  public WPI_TalonSRX SRX_2 = new WPI_TalonSRX(16);
  public WPI_TalonSRX SRX_3 = new WPI_TalonSRX(17);

  // Auto Variables
  public int autoStep = 1;
  public double extensionvalue;

  //DB/String 2 is a field in the smart dashboard, just a variable to pick the print section


  // Smart dashboard values for buttons
  public boolean buttonValue;
  public boolean buttonValueTwo;
  public boolean buttonValueThree;
  public boolean buttonValueFour;

  //variables for auto extension and angling the arm
  public double highscorearm = -16.93;
  public double highscoreextend = 72000;
  public double mediumscorearm;
  public double mediumscoreextend;
  public double goalextend;

  // declaring auto names 
  private static final String Auto2 = "Default";
  private static final String ScoreLow = "ScoreLow";
  private static final String ScoreLowDriveBack = "ScoreLowDriveBack";
  private static final String ScoreLowTwice = "ScoreLowTwice";
  private static final String ScoreLowAndBalance = "ScoreLowAndBalance";
  private static final String DriveForward = "DriveForward";


  private String m_autoSelected;
// allows for us to display and choose auto options through Smart Dashboard
/* SendableChooser is the object that allows us to do this; changing the variable type in <> will change
the variable type able to be displayed on the Smart Dashboard
*/
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  // variables that we get from the gyro (google if needed)
  public double YAW;
  public double ROLL;
  public double PITCH;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

   // declaring auto selection options on Smart Dashboard
    m_chooser.setDefaultOption("Default Auto", Auto2);
    m_chooser.addOption("Score Low", ScoreLow);
    m_chooser.addOption("Score Low and Drive Back", ScoreLowDriveBack);
    m_chooser.addOption("Score Low, Grab Cube and Score again", ScoreLowTwice);
    m_chooser.addOption("Score Low and Balance", ScoreLowAndBalance);
    m_chooser.addOption("Drive Forward", DriveForward);
    SmartDashboard.putData("Auto choices", m_chooser);

    //declare encoders
    RelativeEncoder ArmOneEncoder = ArmUpOne.getEncoder();
    RelativeEncoder ArmTwoEncoder = ArmUpTwo.getEncoder();

    //initial calibrations of all the sensors 
    IntakePiston.set(false);
    Piston.set(false);
    gyro.reset();
    gyro.calibrate();
    
    //motor encoder reset so we get accurate values relative to origin
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);
    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
    ArmOneEncoder.setPosition(0);
    ArmTwoEncoder.setPosition(0);
    
    //disable compressor cause you legally cannot have energy stored in robot upon start
    pcmCompressor.disable();

   
    // Motor supply limits
    SupplyCurrentLimitConfiguration current_limit_config = new SupplyCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    ExtensionMotorOne.configSupplyCurrentLimit(current_limit_config);
    ExtensionMotorTwo.configSupplyCurrentLimit(current_limit_config);
  
    //Rest of drive train set to follow front motors
    //used for simplicity in code
    MiddleLeftMotor.follow(FrontLeftMotor);
    BackLeftMotor.follow(FrontLeftMotor);
    MiddleRightMotor.follow(FrontRightMotor);
    BackRightMotor.follow(FrontRightMotor);

    // Neo motor set current limit (as to not overload the motors)
    FrontLeftMotor.setSmartCurrentLimit(40);
    MiddleLeftMotor.setSmartCurrentLimit(40);
    BackLeftMotor.setSmartCurrentLimit(40);
    FrontRightMotor.setSmartCurrentLimit(40);
    MiddleRightMotor.setSmartCurrentLimit(40);
    BackRightMotor.setSmartCurrentLimit(40);

    // Setting Idle Mode to brake (neo motors) so that the robot does not run away when parked
    FrontLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MiddleLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BackLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    FrontRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MiddleRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BackRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    //set so that the robot does not move when "stopped" (frame of relativity)
    ArmUpOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ArmUpTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    
  } //End of robot initialization


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // sets up autonomous period
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
    autoStep = 1;
    m_autoSelected = m_chooser.getSelected();

    System.out.println("Auto Selected: " + m_autoSelected);

    // Reseting all of the encoder values, for teleop and auto
    // Enabling the PCM compressor/Pnuematically controlled, 
    pcmCompressor.enableDigital();
   
    timer_started = false;
    auto_Timer.stop();
    auto_Timer.reset();
    gyro.reset();
    gyro.setYaw(0);
    autoStep = 1;
    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
  }

  @Override
  public void autonomousPeriodic() {
     
    
    ///define yaw, pitch, roll
    YAW = gyro.getYaw();
    PITCH = gyro.getPitch();
    ROLL = gyro.getRoll() - 2;

    //display the values to the dashboard
    SmartDashboard.putString("DB/String 5", String.valueOf(ROLL));
    SmartDashboard.putString("DB/String 6", String.valueOf(YAW));
    
    SmartDashboard.putString("DB/String 7", ("Avg: " + String.valueOf(AverageEncoderValue)));
    //SmartDashboard.putString("DB/String 0", "printing to string 0");
    SmartDashboard.putString("DB/String 0", ("Left: " + String.valueOf(LeftEncoderValue)));

    SmartDashboard.putString("DB/String 1", ("Right: " + String.valueOf(RightEncoderValue)));

    // gets 3rd line from dashboard to run selected auto
    // autoChooser = "1";
    //autochooser function


    LeftEncoder = FrontLeftMotor.getEncoder();
    RightEncoder = FrontRightMotor.getEncoder();

    LeftEncoderValue = -LeftEncoder.getPosition();
    RightEncoderValue = RightEncoder.getPosition();

    //defining AverageEncoderValue averaging encoders makes encoder values more accurate (removes outliers)
    AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

    //Fix these autos
    //26.5 = 5ft

    switch (m_autoSelected) 
    {
      case ScoreLow:
        if (autoStep == 1) 
        {
          if (!timer_started)
          {
            auto_Timer.start();
            timer_started = true;
          }
      
          if (auto_Timer.get() < 3) 
          {
            Intake.set(-0.3);
          } 
          else 
          {
            Intake.set(0);
            autoStep = 2;
          }
        }
      break;
      case ScoreLowDriveBack:
        if (autoStep == 1) 
        {
          if (!timer_started) 
          {
            auto_Timer.reset();
            auto_Timer.start();
            timer_started = true;
          }
           if (auto_Timer.get() > 1.5) 
           {
            Intake.set(0);
            auto_Timer.stop();
            autoStep = 2;
           } 
          else 
          {
            Intake.set(-0.3);
          }
        }  
        if (autoStep == 2) 
        {
          if (AverageEncoderValue >= -45)
          {
            speed = -0.25;
            FrontRightMotor.set(speed * 0.9368259);
            FrontLeftMotor.set(-speed);
          }
          else
          {
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
          }
        }
      break;
      case ScoreLowTwice:
        if (autoStep == 1) 
        {
          //if timer has not started then reset timer and start it
          if (!timer_started) 
          {
            gyro.setYaw(0);
            auto_Timer.reset();
            auto_Timer.start();
            timer_started = true;
          }

          //if over 1.5 seconds have elapsed
          if (auto_Timer.get() > 1) 
          {
            //stop intake motors, timer, reset timer started. reset gyro values next step
            Intake.set(0);
            auto_Timer.stop();
            timer_started = false;
            gyro.reset();
            autoStep = 2;
          } 
          else 
          {
            //if less than 1.5 elapsed then outake.
            Intake.set(-0.3);
          }
        }  
        if (autoStep == 2)
        {
          //if have not turned 173 then go
          if (YAW <= 150)
          {
            speed = 0.25;
            FrontLeftMotor.set(speed);
            FrontRightMotor.set(speed);
          }
          else
          {
            //if have turned 173 then stop. reset and ++ autostep
            FrontLeftMotor.set(0);
            FrontRightMotor.set(0);

            RightEncoder.setPosition(0);
            LeftEncoder.setPosition(0);

            autoStep = 3;
          }
        }
        if (autoStep == 3)
        {
          if (AverageEncoderValue <= 53)
          {
            speed = 0.25;
            Intake.set(0.4);
            IntakePiston.set(true);
            FrontRightMotor.set(speed * 0.9368259);
            FrontLeftMotor.set(-speed);
          }
          else
          {
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);

            if (!timer_started) 
            {
              auto_Timer.reset();
              auto_Timer.start();
              timer_started = true;
            }
            if (auto_Timer.get() > 0.5) 
            {
              IntakePiston.set(false);
              Intake.set(0);
              auto_Timer.stop();
              timer_started = false;
              autoStep = 4;
            } 
            else 
            {
              Intake.set(0.4);
              IntakePiston.set(true);
            }
          }
        }
        if (autoStep == 4)
        {
          if (YAW <= 332)
          {
            speed = 0.25;
            FrontLeftMotor.set(speed);
            FrontRightMotor.set(speed);
          }
          else
          {
            FrontLeftMotor.set(0);
            FrontRightMotor.set(0);

            autoStep = 5;
          }
        }
        if (autoStep == 5)
        {
          if (AverageEncoderValue <= 115)
          {
            speed = 0.25;
            FrontRightMotor.set(speed * 0.9368259);
            FrontLeftMotor.set(-speed);
          }
          else
          {
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);

            autoStep = 6;
          }
        }
        if (autoStep == 6)
        {
          if (!timer_started) 
          {
            auto_Timer.reset();
            auto_Timer.start();
            timer_started = true;
          }
          if (auto_Timer.get() > 1.5) 
          {
            Intake.set(0);
            auto_Timer.stop();
            timer_started = false;
            gyro.reset();
          } 
          else 
          {
            Intake.set(-0.3);
          }
        }
      break;
      case DriveForward:
        speed = 0.25;
        FrontLeftMotor.set(-speed);
        FrontRightMotor.set(speed);
      break;
        
      case ScoreLowAndBalance:
        if (autoStep == 1) 
        {
          if (!timer_started)
          {
            auto_Timer.start();
            timer_started = true;
          }
      
          if (auto_Timer.get() < 3) 
          {
            Intake.set(-0.3);
          } 
          else 
          {
            Intake.set(0);
            autoStep = 2;
          }
        }
        
        if (autoStep == 2)
        {
          if (YAW <= 150)
          {
            speed = 0.25;
            FrontLeftMotor.set(speed);
            FrontRightMotor.set(speed);
          }
          else
          {
            //if have turned 173 then stop. reset and ++ autostep
            FrontLeftMotor.set(0);
            FrontRightMotor.set(0);

            RightEncoder.setPosition(0);
            LeftEncoder.setPosition(0);

            autoStep = 3;
          }  
        } 
        if (autoStep == 3)
        {
          ROLL = gyro.getRoll() - 2;
          //if robot parallel to ground then go forward
          if (ROLL >= -13 && ROLL <= 13)
          {
            FrontLeftMotor.set(-0.3);
            FrontRightMotor.set(0.3* 0.80);
          }
          //if going up then autostep increases
          else
          {
            FrontLeftMotor.set(-0.15);
            FrontRightMotor.set(0.15* 0.80);
            autoStep = 4;
          }
        }
        if (autoStep == 4)
        {
          ROLL = gyro.getRoll() - 2;
          //if the robot is going up the charging station, then go forward at half the speed that you ran up it.
          if (ROLL < -7)
          {
            speed = 0.12;

            FrontLeftMotor.set(-speed);
            FrontRightMotor.set(speed* 0.70);
          }   
          //if the robot has overshot, and is now going down the charging station, then back up.
          else if (ROLL > 7)
          {
            speed = -0.1;

            FrontLeftMotor.set(-speed);
            FrontRightMotor.set(speed* 0.70);
          }
          else if (ROLL >- 7 && ROLL < -3)
          {
            speed = 0.05;

            FrontLeftMotor.set(-speed);
            FrontRightMotor.set(speed* 0.70);
          }
          else if (ROLL < 7 && ROLL > 3)
          {
            speed = -0.05;

            FrontLeftMotor.set(-speed);
            FrontRightMotor.set(speed* 0.70);
          }
          else
          {
            FrontLeftMotor.set(0);
            FrontRightMotor.set(0);
          }
        }
      break;
      case Auto2:
      default:
        Intake.set(0);
      break; 
    }

  }

  //default package
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // calibrating all sensors/encoders back to zero for teleop session
    ArmOneEncoder = ArmUpOne.getEncoder();
    ArmTwoEncoder = ArmUpTwo.getEncoder();
    gyro.reset();
    gyro.setYaw(0);
    gyro.calibrate();
    Piston.set(false);
    IntakePiston.set(false);
    SRX_1.set(0);
    SRX_2.set(0);
    SRX_3.set(0);
    ArmOneEncoder.setPosition(0);
    ArmTwoEncoder.setPosition(0);
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);
    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
  }

  @Override
  public void teleopPeriodic() {

   //keeping it for rumble...and owen's sake
   //buttonValueTwo = SmartDashboard.getBoolean("DB/Button 1", false);
    
    LightSolenoidPCM.set(false); 

    //display values to the dashboard to keep track for testing and live troubleshooting
    SmartDashboard.putString("DB/String 0", "LD " + String.valueOf(LeftEncoderValue));
    SmartDashboard.putString("DB/String 1", "RD " + String.valueOf(RightEncoderValue));
    SmartDashboard.putString("DB/String 3", "EL " + String.valueOf(maxextensionlimit));
    SmartDashboard.putString("DB/String 4", "EV " + String.valueOf(extensionvalue));
    SmartDashboard.putString("DB/String 8", "ROLL " + ((String.valueOf(ROLL))));
    SmartDashboard.putString("DB/String 9", "YAW " + ((String.valueOf(YAW))));
    
    //dashboard testing(come back to it)
    // SmartDashboard.putString("DB/String 5", "AOV " + String.valueOf(ArmOneEncoderValue));
    //SmartDashboard.putString("DB/String 6", "ATV " + String.valueOf(ArmTwoEncoderValue));
    //SmartDashboard.putString("DB/String 7", "AVDV " + String.valueOf(AverageEncoderValue));
    //SmartDashboard.putNumber("DB/String 10", (ArmOneEncoderValue));
    //SmartDashboard.putNumber("DB/String 11", (ArmTwoEncoderValue));
    //SmartDashboard.putNumber("DB/String 12", (AverageEncoderValue)); 
    //System.out.println(ArmOneEncoderValue);
    //System.out.println(ArmTwoEncoderValue);
    
    //enabling the vacuum chamber during  
    pcmCompressor.enableDigital();

    //synchronising variables with the robot 
    extensionvalue = ExtensionMotorOne.getSelectedSensorPosition();

    //gyro starts at 2 degrees so we set it back to 0 manually
    //declaring variable responsible for front/back on a horizontal axis
    ROLL = gyro.getRoll() - 2;

    //declaing variable responsible for right/left on a vertical axis
    YAW = gyro.getYaw();

    //getting position of drivetrain encoder (for testing and troubleshooting)
    LeftEncoderValue = -LeftEncoder.getPosition();
    RightEncoderValue = RightEncoder.getPosition();
   
    //averaging encoder values to get more accurate values (remove outliers)
    AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

    //each respective arm encoding (mainlyfor testing and live troubleshooting)
    ArmOneEncoder = ArmUpOne.getEncoder();
    ArmTwoEncoder = ArmUpTwo.getEncoder();

    ArmOneEncoderValue = -ArmOneEncoder.getPosition();
    ArmTwoEncoderValue = ArmTwoEncoder.getPosition();
    //averaging encoder values to get more accurate values (remove outliers)
    AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
   
    /* rumble button (for testing)
    if (buttonValueTwo == true){
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
      Xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
    }
    
    else if (buttonValueTwo == false){
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
      Xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
      Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
    }*/

  //202.06/1 rotation (gear ratio for arm up and down)

  // values for calculating arm extension limit
  // 41 inches (extension limit in inches)
  // 2988.303 is multiplier by inches (encoder value per inch)
  
  //extension limit (inches times encoder value multiplier)
  mainlimit = 122520.423;
  
  // Setting extension limits
  if (AverageArmEncoderValue <= -15){
      // number in parameters + (number in parenthese subtracted from AverageArmValue) must equal division factor
      limitfactor = (AverageArmEncoderValue - 1) / -16;

      //the max the arm should extend
      maxextensionlimit = mainlimit / limitfactor;
  }
  else{
      // resetting limits
      maxextensionlimit = mainlimit;
  }
  //silly
  

  // Release the piston while the arm is being extended & retracted
  if (Xbox.getRawButtonPressed(2) || Xbox.getRawButtonPressed(1)){
      Piston.set(true);
  }
  // Stops firing piston while arm is extended & retracted
  if (Xbox.getRawButtonReleased(1) || Xbox.getRawButtonReleased(2)){
      Piston.set(false);
  }

  //testing section, currently does not work
  if (Xbox.getRawButton(8)){
    // arm to go forward
    if (extensionvalue <= maxextensionlimit){
      if (extensionvalue <= 75000){
          Piston.set(true);
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
      }
      else{
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
          Piston.set(false);
      }
    }
    else{
        ExtensionMotorOne.set(0.0);
        ExtensionMotorTwo.set(0.0);
    }

    if (AverageArmEncoderValue >= -47){
      if (AverageArmEncoderValue <= -17.5){
          ArmUpOne.set(-0.25);
          ArmUpTwo.set(0.25);
      }
      // arm to go down
      else if (AverageArmEncoderValue >= -15.5){
          ArmUpOne.set(0.25);
          ArmUpTwo.set(-0.25);
      }
      else{
          ArmUpOne.set(0);
          ArmUpTwo.set(0);
      }
    }
    else{
        ArmUpOne.set(0);
        ArmUpTwo.set(0);
    }
  }
  // Between 0 & 41 inches, the arm can retract & extend within extension limit of 41 inches ( 0 < x < 41) :)
        //button 1("A" on controller) extends
        //button 2("B" on controller) is retract
        //else if nothing is pressed dont move and make sure pistons are off
      if (extensionvalue <= maxextensionlimit && extensionvalue > 0.0){
        if (Xbox.getRawButton(1)){
            ExtensionMotorOne.set(0.3);
            ExtensionMotorTwo.set(0.3);
        }
        else if (Xbox.getRawButton(2)){
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
        }
        else {
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
          Piston.set(false);
        }
      }
      
      // when the extension value is over the max extension limit it can only retract (because the value of the arm cant go past the limit)
      //if past extension limit then retract when button 2 or "A" is pressed.
      //cant extend even if you try. Otherwise retract automatically
      if (extensionvalue >= maxextensionlimit){          
      
          if (Xbox.getRawButton(2)){
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
          }
          else {
            ExtensionMotorOne.set(-0.2);
            ExtensionMotorTwo.set(-0.2);
            Piston.set(true);
          }
      }

      // arm only extend (cant retract to far into itself. Just back to the starting position)
      //same as last function just for when below range
      if (extensionvalue <= 0.0) {
        
        if (Xbox.getRawButton(1)){
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
        }
        else{
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
        }
      }
      
      //assuming the arm encoder ticks (extension) above -25 and below 0 you can extend and retract as you wish
      // the value is -25 given the fact that it is the value obtained from testing the extension limits while looking at the encoder ticks 
      //printed to the dashboard.
      if ((AverageArmEncoderValue >= -25) && (AverageArmEncoderValue <= 0)){
          if (Xbox.getRawButton(6)){            
              ArmUpOne.set(-0.25);
              ArmUpTwo.set(0.25);
          }
          else if (Xbox.getRawButton(5)){
              ArmUpOne.set(0.25);
              ArmUpTwo.set(-0.25);
          }
          else{
              ArmUpOne.set(0);
              ArmUpTwo.set(0);
          }
      }

      //if arm enocoder ticks is below -25 the arm can only go up
      if (AverageArmEncoderValue <= -25) {
        if (Xbox.getRawButton(6)) {
            ArmUpOne.set(-0.25);
            ArmUpTwo.set(0.25);
        }
        else {
            ArmUpOne.set(0);
            ArmUpTwo.set(0);
        }
      }

      //encoder value greater than 0 then you can retract the arm freely
      if (AverageArmEncoderValue >= 0) {
        if (Xbox.getRawButton(5)) {
            ArmUpOne.set(0.25);
            ArmUpTwo.set(-0.25);
        }
        else {
            ArmUpOne.set(0);
            ArmUpTwo.set(0);
        }
      }
     
  //turns on front suction
  //(old version of cranium, before the new claw)
  if (Xbox.getRawAxis(3) >= 0.1){
      SRX_1.set(-0.6);
      SRX_2.set(-0.6);
      SRX_3.set(-0.6);

      LightSolenoidPCM.set(true); 
      
  }
  else if (Xbox.getRawAxis(2) >= 0.1){

      SRX_1.set(.3);
      SRX_2.set(.3);
      SRX_3.set(.3);
  }
  else{
    SRX_1.set(-.15);
    SRX_2.set(-.15);
    SRX_3.set(-.15);

    LightSolenoidPCM.set(false);
  }

  //deploys ground intake(rollers) using the joystick
  if (JoyStick1.getRawButton(1)){
      Intake.set(0.5);
      IntakePiston.set(true);
  }
  else if (JoyStick1.getRawButton(3)){
      Intake.set(-0.4);
  }
  else{
      Intake.set(0);
      IntakePiston.set(false);
  }

  //sets the wheel and joystick up to get position values for code. 
  //lmol and rmor are essentially auto-corrective functions that urges the the robot to drive straight. The robot naturally 
  //does not drive straight.
  double WheelX = -Wheel.getX();
  double JoyY = JoyStick1.getY();
  double lmol = ((WheelX * 0.5) + (0.6 * JoyY));
  double rmor = ((WheelX * 0.5) - (0.6 * JoyY)) * 0.9368259;
  FrontLeftMotor.set(lmol);
  FrontRightMotor.set(rmor);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    auto_Timer.reset();
    pcmCompressor.enableDigital();

  }

  @Override
  public void testPeriodic() {
    if (!timer_started) {
      auto_Timer.start();
      timer_started = true;
    }

    SmartDashboard.putString("DB/String 0", "timer: " + String.valueOf(auto_Timer.get()));

    if (auto_Timer.get() < 3) {
      Intake.set(-0.3);
    } else {
      Intake.set(0);
    }

    /* 
    ArmOneEncoderValue = ArmOneEncoder.getPosition();
    ArmTwoEncoderValue = ArmTwoEncoder.getPosition();
    AverageArmEncoderValue = (ArmOneEncoderValue + ArmTwoEncoderValue)/2;
    ExtensionMotorOne.getSelectedSensorPosition();

    System.out.println(ArmOneEncoderValue);
    System.out.println(ArmTwoEncoderValue);
    System.out.println(AverageArmEncoderValue);

    if (Xbox.getRawButton(8)){
      if (extensionvalue <= 74500){
          Piston.set(true);
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
      }
      else if(extensionvalue >= 75500){
        Piston.set(true);
        ExtensionMotorOne.set(-0.2);
        ExtensionMotorTwo.set(-0.2);
      }
      else{
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
          Piston.set(false);
      }
      if(AverageArmEncoderValue <= -9){
        ArmUpOne.set(0.1);
        ArmUpTwo.set(0.1);
      }
      else if(AverageArmEncoderValue >= -11){
        ArmUpOne.set(-0.1);
        ArmUpTwo.set(-0.1);
      }
      else{
        ArmUpOne.set(0);
        ArmUpTwo.set(0);
      }
    }
    */
  }
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}