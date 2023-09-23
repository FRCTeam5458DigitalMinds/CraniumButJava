// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import org.opencv.core.Scalar;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.GenericHID;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  //Joysticks
  public Joystick Xbox = new Joystick(0);
  public Joystick JoyStick1 = new Joystick(1);
  public Joystick Wheel = new Joystick(2);

  //Drivetrain motors
  public CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax MiddleLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax BackLeftMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax MiddleRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax BackRightMotor = new CANSparkMax(6, MotorType.kBrushless);

  //Intake motor
 // public CANSparkMax IntakeMotor = new CANSparkMax(25, MotorType.kBrushless);

  //DriveTrain encoder
  public RelativeEncoder LeftEncoder = FrontLeftMotor.getEncoder();
  public RelativeEncoder RightEncoder = FrontRightMotor.getEncoder();


  //Arm Motors
  public CANSparkMax ArmUpOne = new CANSparkMax(7, MotorType.kBrushless);
  public CANSparkMax ArmUpTwo = new CANSparkMax(8, MotorType.kBrushless);

  //public WPI_TalonFX ClawMotor = new WPI_TalonFX(9);
  public WPI_TalonFX ExtensionMotorOne = new WPI_TalonFX(10);
  public WPI_TalonFX ExtensionMotorTwo = new WPI_TalonFX(11);
  public WPI_TalonFX Intake = new WPI_TalonFX(25);


  //Arm encoders
  public RelativeEncoder ArmOneEncoder;
  public RelativeEncoder ArmTwoEncoder;
  
  
  //Solenoids & compresser
  public Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public Solenoid Piston = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public Solenoid IntakePiston = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public Solenoid Lights = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

  public Solenoid Vent1 = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
  public Solenoid Vent2 = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
  public Solenoid Vent3 = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

  //robot variables
  public double LeftEncoderValue;
  public double RightEncoderValue;
  public double ArmOneEncoderValue;
  public double ArmTwoEncoderValue;
  public double speed;
  public double AverageEncoderValue;
  public double AverageArmEncoderValue;
  public double newEncoderUno;
  public double newEncoderDos;
  public boolean autoStart = false;
  public double maxextensionlimit;
  public double mainlimit;
  public double maxanglelimit = 4;
  public boolean subpreset = false;
  public boolean pistonenable = false;
  // set amp limit
  public boolean enable = true;
  public boolean brakemode = false;
  public double currentLimit = 60;
  public double triggerThresholdCurrent = 60;
  public double triggerThresholdTime = .1;

  public double currentarm;
  public double currentextend;
  public double limitfactor;
  // Gyro
  public WPI_PigeonIMU gyro = new WPI_PigeonIMU(12);

  // compressors
  public WPI_TalonSRX SRX_1 = new WPI_TalonSRX(15);
  public WPI_TalonSRX SRX_2 = new WPI_TalonSRX(16);
  public WPI_TalonSRX SRX_3 = new WPI_TalonSRX(17);

  // Auto Variables
  public int autoStep = 1;
  public double extensionvalue;
  public String autoChooser = SmartDashboard.getString("DB/String 2", "myDefaultData");;

  // Timer
  public boolean gyroResetted = false;
  public long last = System.currentTimeMillis();

  // dashboard variables
  public boolean buttonValue;
  public boolean buttonValueTwo;
  public boolean buttonValueThree;
  public boolean buttonValueFour;
  public double k_rampTimeSeconds = 0.25;

  public boolean timerStarted = false;

  public double highscorearm = -16.93;
  public double highscoreextend = 72000;
  public double mediumscorearm;
  public double mediumscoreextend;

  public double goalextend;

  // PID Aspects
  public double YAW;
  public double ROLL;
  public double PITCH;
  public double SPEED;
  public double driveLimit;

  public boolean coneintake;

  // Intake Outake Variable
  public int bothTake = 1;

  @Override
  public void robotInit() {
    RelativeEncoder ArmOneEncoder = ArmUpOne.getEncoder();
    RelativeEncoder ArmTwoEncoder = ArmUpTwo.getEncoder();

    m_robotContainer = new RobotContainer();
    IntakePiston.set(true);
    gyro.reset();
    Piston.set(false);

    gyro.calibrate();
    pcmCompressor.disable();
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);

    // Reset encoders
    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
    ArmOneEncoder.setPosition(0);
    ArmTwoEncoder.setPosition(0);

    // camera
    CameraServer.startAutomaticCapture();

    // Motor supply limits
    SupplyCurrentLimitConfiguration current_limit_config = new SupplyCurrentLimitConfiguration(enable, currentLimit, triggerThresholdCurrent, triggerThresholdTime);
    //SupplyCurrentLimitConfiguration current_claw_config = new SupplyCurrentLimitConfiguration(enable, 30, 30, triggerThresholdTime);
    ExtensionMotorOne.configSupplyCurrentLimit(current_limit_config);
    ExtensionMotorTwo.configSupplyCurrentLimit(current_limit_config);
    //ClawMotor.configSupplyCurrentLimit(current_claw_config);

    // Follow front motors
    MiddleLeftMotor.follow(FrontLeftMotor);
    BackLeftMotor.follow(FrontLeftMotor);
    MiddleRightMotor.follow(FrontRightMotor);
    BackRightMotor.follow(FrontRightMotor);

    // Neo motor current limit
    FrontLeftMotor.setSmartCurrentLimit(40);
    MiddleLeftMotor.setSmartCurrentLimit(40);
    BackLeftMotor.setSmartCurrentLimit(40);
    FrontRightMotor.setSmartCurrentLimit(40);
    MiddleRightMotor.setSmartCurrentLimit(40);
    BackRightMotor.setSmartCurrentLimit(40);

    // Setting Idle Mode to brake (neo motors)
    FrontLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MiddleLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BackLeftMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    FrontRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    MiddleRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    BackRightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

    ArmUpOne.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ArmUpTwo.setIdleMode(CANSparkMax.IdleMode.kBrake);
    ExtensionMotorOne.setSelectedSensorPosition(0);
  }
  
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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    // Enabling the PCM compressor/Pnuematically controlled
    pcmCompressor.enableDigital();
    ArmOneEncoder.setPosition(0);
    ArmTwoEncoder.setPosition(0);
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);
    gyro.reset();
    autoStep = 1;
    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);

    gyro.setYaw(0);

    CameraServer.startAutomaticCapture();

    timerStarted = false;
  }

  @Override
  public void autonomousPeriodic() {
    // Enabling the PCM compressor
    Piston.set(false);
    YAW = gyro.getYaw();
    PITCH = gyro.getPitch();
    ROLL = gyro.getRoll() - 2;
    SmartDashboard.putString("DB/String 8", String.valueOf(ROLL));
    SmartDashboard.putString("DB/String 9", String.valueOf(YAW));
    SmartDashboard.putString("DB/String 3", String.valueOf(maxextensionlimit));
    SmartDashboard.putString("DB/String 8", String.valueOf(ROLL));
    autoChooser = SmartDashboard.getString("DB/String 2", "myDefaultData");
    if (autoChooser == "1"){
      SmartDashboard.putString("DB/String 3", "AutoOne");
    }
    else if (autoChooser == "2"){
      SmartDashboard.putString("DB/String 3", "AutoTwo");
    }
    else if (autoChooser == "3"){
      SmartDashboard.putString("DB/String 3", "AutoThree");
    }

    AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

    LeftEncoderValue = -LeftEncoder.getPosition();
    RightEncoderValue = RightEncoder.getPosition();

    SmartDashboard.putString("DB/String 7", ("Left: " + String.valueOf(AverageEncoderValue)));
    SmartDashboard.putString("DB/String 0", ("Left: " + String.valueOf(LeftEncoderValue)));
    SmartDashboard.putString("DB/String 1", ("Right: " + String.valueOf(RightEncoderValue)));

    AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
    ArmOneEncoderValue = -ArmOneEncoder.getPosition();
    ArmOneEncoder.getPosition();

    extensionvalue = ExtensionMotorOne.getSelectedSensorPosition();
    
    if (autoChooser == "1")
    {
      // Goes forwards towards the charge station inversed motors
      if (autoStep == 1 && AverageEncoderValue <= 26.5)
      {
        speed = 0.4;
        FrontRightMotor.set(speed);
        FrontLeftMotor.set(-speed);

        autoStep++;
      }
      // makes robot stop on top of charge station
      else if (autoStep == 2 && AverageEncoderValue >= 26.5)
      {
        ROLL = gyro.getRoll() - 2;

        if (ROLL >= -1 && ROLL <= 1)
        {
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
        }
        if (ROLL >= 1)
        {
          FrontLeftMotor.set(0.3);
          FrontRightMotor.set(-0.3);
        }
        else if (ROLL <= -1)
        {
          FrontLeftMotor.set(-0.3);
          FrontRightMotor.set(0.3);
        }
      }
    }

    if (autoChooser == "2"){
      if (autoStep == 1){
        IntakePiston.set(false);
        Intake.set(-0.3);
        long now = System.currentTimeMillis();
        if(now - last >= 2000){
          Intake.set(0);
          autoStep++;
        }
      }

      if (autoStep == 2 && AverageEncoderValue >= -37){
        speed = 0.4;
        FrontRightMotor.set(-speed);
        FrontLeftMotor.set(speed);
        autoStep++;
      }
      else if (autoStep == 3 && AverageEncoderValue <= -37){
        FrontLeftMotor.set(0);
        FrontRightMotor.set(0);
      }
    }

    // Auto 3: Score and leave the community
    if (autoChooser == "3"){

      // Arm and Extension (Scoring during auto)
      extensionvalue = ExtensionMotorOne.getSelectedSensorPosition();

      if (autoStep == 1){
        //ClawMotor.set(0.3);
        SRX_1.set(1);
        SRX_2.set(1);
        SRX_3.set(1);
        Vent1.set(false);
        Vent2.set(false);
        Vent3.set(false);
        if (AverageArmEncoderValue >= highscorearm){
          ArmUpOne.set(0.2);
          ArmUpTwo.set(-0.2);
        }
        else{
          ArmUpOne.set(0);
          ArmUpTwo.set(0);
        }
        if (extensionvalue <= highscoreextend){
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
        }
        else{
          ExtensionMotorOne.set(0);
          ExtensionMotorTwo.set(0);
        }
        if (extensionvalue >= highscoreextend && AverageArmEncoderValue <= highscorearm){
          autoStep++;
          last = System.currentTimeMillis();
        }
      }
      // this will drop the cube
      else if (autoStep == 2){
        long now = System.currentTimeMillis();
        if (now - last <= 2000){
          SRX_1.set(0);
          SRX_2.set(0);
          SRX_3.set(0);
          Vent1.set(true);
          Vent2.set(true);
          Vent3.set(true);
          //ClawMotor.set(-0.3);
        }

        if (now - last >= 2000){
          if (AverageArmEncoderValue <= -2){
            ArmUpOne.set(-0.2);
            ArmUpTwo.set(0.2);
          }
          else{
            ArmUpOne.set(0);
            ArmUpTwo.set(0);
          }
          if (extensionvalue >= 3000){
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
          }
          else{
            ExtensionMotorOne.set(0);
            ExtensionMotorTwo.set(0);
            //autoStep++;
          }
        }
      }
      // this will go out of the community
      else if (autoStep == 3){
        if (YAW <= 3 && YAW >= -3){
          if (autoStep == 2 && AverageEncoderValue >= -30){
            speed = -0.3;
            FrontRightMotor.set(speed);
            FrontLeftMotor.set(-speed);
          }
        }
        else{
          if (AverageEncoderValue >= -30){
            if (YAW >= 3){
              FrontRightMotor.set(speed);
              FrontLeftMotor.set(-speed * 0.7);
            }
            if (YAW <= -3){
              FrontRightMotor.set(speed * 0.7);
              FrontLeftMotor.set(-speed);
            }
          }
        }
      }
    }

    // Auto 4: Scoring High
    if (autoChooser == "4")
    {
      // Arm and Extension (Scoring during auto)
      if (autoStep == 1)
      {
        if (AverageArmEncoderValue >= highscorearm)
        {
          ArmUpOne.set(0.2);
          ArmUpTwo.set(-0.2);
        }
        else
        {
          ArmUpOne.set(0);
          ArmUpTwo.set(0);
        }
        if (extensionvalue <= highscoreextend)
        {
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
        }
        else
        {
          ExtensionMotorOne.set(0);
          ExtensionMotorTwo.set(0);
        }
        if (extensionvalue >= highscoreextend && AverageArmEncoderValue <= highscorearm)
        {
          autoStep++;
          last = System.currentTimeMillis();
        }
      }
      else if (autoStep == 2)
      {
        long now = System.currentTimeMillis();
        if (now - last >= 2000)
        {
          SRX_1.set(0);
          SRX_2.set(0);
          SRX_3.set(0);
          //ClawMotor.set(-0.3);
          //added
        }

        if (now - last >= 2000)
        {
          if (AverageArmEncoderValue <= 7)
          {
            ArmUpOne.set(-0.2);
            ArmUpTwo.set(0.2);
          }
          else
          {
            ArmUpOne.set(0);
            ArmUpTwo.set(0);
          }
          if (extensionvalue >= 3000)
          {
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
          }
          else
          {
            ExtensionMotorOne.set(0);
            ExtensionMotorTwo.set(0);
          }
        }
      }
    }

    if (autoChooser == "5"){
      if (autoStep == 1){
        IntakePiston.set(false);
        Intake.set(-0.4);
        long now = System.currentTimeMillis();
        if (now - last >= 1500){
          Intake.set(0);
          autoStep++;
        }
        
      }
      // Leave Community
      if (autoStep == 2){
        if (YAW <= 3 && YAW >= -3){
          if (autoStep == 2 && AverageEncoderValue >= -45){
            speed = -0.3;
            FrontRightMotor.set(speed * 0.9);
            FrontLeftMotor.set(-speed);
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
          }
        }
        else{
          if (AverageEncoderValue >= -45){
            if (YAW >= 3){
              FrontRightMotor.set(speed);
              FrontLeftMotor.set(-speed * 0.7);
            }
            if (YAW <= -3){
              FrontRightMotor.set(speed * 0.7);
              FrontLeftMotor.set(-speed);
            }
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
          }
        }
      }
    }

    if (autoChooser == "6"){
      if (autoStep == 1){
        IntakePiston.set(false);
        Intake.set(-0.4);
        long now = System.currentTimeMillis();
        if (now - last >= 1500){
          Intake.set(0);
          autoStep++;
          last = System.currentTimeMillis();
        }
      }
      if (autoStep == 2){
        if (YAW <= 173){
          speed = 0.15;
          FrontLeftMotor.set(speed);
          FrontRightMotor.set(speed);
        }
        else{
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
          RightEncoder.setPosition(0);
          LeftEncoder.setPosition(0);
          long now = System.currentTimeMillis();
          if (now - last >= 500){
            autoStep++;
          }
        }
      }
      if (autoStep == 3 && AverageEncoderValue < 31){
        speed = 0.5;
        FrontRightMotor.set(speed);
        FrontLeftMotor.set(-speed);
      }
      if (autoStep == 3 && AverageEncoderValue >= 31){
        ROLL = gyro.getRoll() - 2;
        if (ROLL >= -3 && ROLL <= 3){
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
        }
        if (ROLL >= 3){
          if (YAW <= 183 && YAW >= 177){
            FrontLeftMotor.set(0.1);
            FrontRightMotor.set(-0.09368259);
          }
          else if (YAW >= 183){
            FrontLeftMotor.set(0.1);
            FrontRightMotor.set(-0.07);
          }
          else if (YAW <= 177){
            FrontLeftMotor.set(0.07);
            FrontRightMotor.set(-0.1);
          }
        }
        else if (ROLL <= -3){
          if (YAW <= 183 && YAW >= 177){
            FrontLeftMotor.set(-0.1);
            FrontRightMotor.set(0.09368259);
          }
          else if (YAW >= 183){
            FrontLeftMotor.set(-0.07);
            FrontRightMotor.set(0.1);
          }
          else if (YAW <= 177){
            FrontLeftMotor.set(-0.1);
            FrontRightMotor.set(0.07);
          }
        }
      }
    }
    if (autoChooser == "7"){
    long now = System.currentTimeMillis();
    IntakePiston.set(false);
    Intake.set(-0.4);
      if (now - last >= 1500){
        Intake.set(0);
        autoStep++;
      }
    }

    if (autoChooser == "8"){
      long now = System.currentTimeMillis();
      if (autoStep == 1){
        Intake.set(-0.4);
        if (now - last >= 1500){
          Intake.set(0);
          autoStep++;
        }
      }
      if (autoStep == 2){
        if (YAW <= 173){
          speed = 0.15;
          FrontLeftMotor.set(speed);
          FrontRightMotor.set(speed);
        }
        else{
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
          LeftEncoder.setPosition(0);
          RightEncoder.setPosition(0);
          autoStep++;
        }
      }
      if (autoStep == 3){
        if (YAW <= 180 && YAW >= 174){
          if (AverageEncoderValue <= 55){
            speed = 0.3;
            FrontRightMotor.set(speed);
            FrontLeftMotor.set(-speed);
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
            autoStep++;
          }
        }
        else{
          if (AverageEncoderValue <= 55){
            if (YAW >= 180){
              FrontRightMotor.set(speed * 0.7);
              FrontLeftMotor.set(-speed);
            }
            if (YAW <= 174){
              FrontRightMotor.set(speed);
              FrontLeftMotor.set(-speed * 0.7);
            }
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
            Intake.set(0);
            LeftEncoder.setPosition(0);
            RightEncoder.setPosition(0);

            autoStep++;
            last = System.currentTimeMillis();
          }
        }
      }
      if (autoStep == 4){
        IntakePiston.set(true);
        Intake.set(0.4);
        if (now - last >= 2000){
          IntakePiston.set(false);
          Intake.set(0);
          autoStep = 5;
        }
      }
      if (autoStep == 5){
        if (YAW >= 7){
          speed = -0.15;
          FrontLeftMotor.set(speed);
          FrontRightMotor.set(speed);
        }
        else{
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
          LeftEncoder.setPosition(0);
          RightEncoder.setPosition(0);
          autoStep++;
        }
      }
      if (autoStep == 6){
        if (YAW <= 3 && YAW >= -3){
          if (AverageEncoderValue <= 55){
            speed = 0.3;
            FrontRightMotor.set(speed);
            FrontLeftMotor.set(-speed);
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
            autoStep++;
          }
        }
        else{
          if (AverageEncoderValue <= 55){
            if (YAW >= 3){
              FrontRightMotor.set(speed * 0.7);
              FrontLeftMotor.set(-speed);
            }
            if (YAW <= -3){
              FrontRightMotor.set(speed);
              FrontLeftMotor.set(-speed * 0.7);
            }
          }
          else{
            FrontRightMotor.set(0);
            FrontLeftMotor.set(0);
            // autoStep++;
          }
        }
      }
    }

    if (autoChooser == "9"){
      long now = System.currentTimeMillis();
      // dispersing cube
      if (autoStep == 1){
        Intake.set(-0.48);
        if (now - last >= 500){
          Intake.set(0);
          autoStep = 2;
        }
      }
      // 180 degree turn
      if (autoStep == 2){
        if (YAW <= 173){
          speed = 0.15;
          FrontLeftMotor.set(speed);
          FrontRightMotor.set(speed);
        }
        else{
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
          LeftEncoder.setPosition(0);
          RightEncoder.setPosition(0);
          autoStep = 3;
        }
      }
      // over charging station
      if (autoStep == 3 && AverageEncoderValue < 40){
        speed = 0.3;
        FrontRightMotor.set(speed * 0.9);
        FrontLeftMotor.set(-speed);
      }
      else{
        autoStep++;
      }
      // turning again
      if (autoStep == 4){
        if (YAW >= 7){
          speed = -0.15;
          FrontLeftMotor.set(speed);
          FrontRightMotor.set(speed);
        }
        else{  
          gyro.setYaw(0);
          LeftEncoder.setPosition(0);
          RightEncoder.setPosition(0);
          autoStep++;
        }
      }
      // onto charging station
      if (autoStep == 5 && AverageEncoderValue < 15){
        speed = 0.3;
        FrontRightMotor.set(speed);
        FrontLeftMotor.set(-speed * 0.9);
      }
      // balancing charging station
      if (autoStep == 5 && AverageEncoderValue >= 15){
        ROLL = gyro.getRoll() - 2;
        if (ROLL >= -3 && ROLL <= 3){
          FrontLeftMotor.set(0);
          FrontRightMotor.set(0);
        }
        if (ROLL >= 3){
          if (YAW <= 3 && YAW >= -3){
            speed = -0.07;
            FrontLeftMotor.set(0.1);
            FrontRightMotor.set(-0.09);
          }
          else if (YAW >= 3){
            FrontLeftMotor.set(0.1 * 0.7);
            FrontRightMotor.set(-0.09);
          }
          else if (YAW <= -3){
            FrontLeftMotor.set(0.1);
            FrontRightMotor.set(-0.09 * 0.7);
          }
        }
        else if (ROLL <= -3){
          if (YAW <= 3 && YAW >= -3){
            FrontLeftMotor.set(-0.1);
            FrontRightMotor.set(0.09);
          }
          else if (YAW >= 3){
            FrontLeftMotor.set(-0.1 * 0.7);
            FrontRightMotor.set(0.09);
          }
          else if (YAW <= -3){
            FrontLeftMotor.set(-0.1);
            FrontRightMotor.set(0.09 * 0.7);
          }
        }
      }
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    SmartDashboard.putString("DB/String 2", ("i am normal"));
    ArmOneEncoder = ArmUpOne.getEncoder();
    ArmTwoEncoder = ArmUpTwo.getEncoder();
    timerStarted = false;
    gyro.setYaw(0);
    gyro.calibrate();
    Piston.set(false);
    IntakePiston.set(false);
    Vent1.set(false);
    Vent2.set(false);
    Vent3.set(false);
    SRX_1.set(0);
    SRX_2.set(0);
    SRX_3.set(0);

    //ClawMotor.set(0);
    ArmOneEncoder.setPosition(0);
    ArmTwoEncoder.setPosition(0);
    ExtensionMotorOne.setSelectedSensorPosition(0);
    ExtensionMotorTwo.setSelectedSensorPosition(0);

    gyro.reset();
    // Resetting sensor positions using buttons on the dashboard

    RightEncoder.setPosition(0);
    LeftEncoder.setPosition(0);
    gyro.reset();
    last = System.currentTimeMillis();
  }

  @Override
  public void teleopPeriodic() {

    buttonValueTwo = SmartDashboard.getBoolean("DB/Button 1", false);

    SmartDashboard.putString("DB/String 0", "LD " + String.valueOf(LeftEncoderValue));
    SmartDashboard.putString("DB/String 1", "RD " + String.valueOf(RightEncoderValue));
    SmartDashboard.putString("DB/String 3", "EL " + String.valueOf(maxextensionlimit));
    SmartDashboard.putString("DB/String 4", "EV " + String.valueOf(extensionvalue));
    SmartDashboard.putString("DB/String 5", "AOV " + String.valueOf(ArmOneEncoderValue));
    SmartDashboard.putString("DB/String 6", "ATV " + String.valueOf(ArmTwoEncoderValue));
    SmartDashboard.putString("DB/String 7", "AVDV " + String.valueOf(AverageEncoderValue));
    SmartDashboard.putString("DB/String 8", "ROLL " + ((String.valueOf(ROLL))));
    SmartDashboard.putString("DB/String 9", "YAW " + ((String.valueOf(YAW))));

    pcmCompressor.enableDigital();
    extensionvalue = ExtensionMotorOne.getSelectedSensorPosition();

    ROLL = gyro.getRoll() - 2;
    YAW = gyro.getYaw();

    LeftEncoderValue = -LeftEncoder.getPosition();
    RightEncoderValue = RightEncoder.getPosition();

   

    AverageEncoderValue = (LeftEncoderValue + RightEncoderValue) / 2;

    //ArmOneEncoder = ArmUpOne.getEncoder();
    // ArmTwoEncoder = ArmUpTwo.getEncoder();
    newEncoderUno = -ArmOneEncoder.getPosition();
    newEncoderDos = ArmTwoEncoder.getPosition();

    if (String.valueOf(newEncoderUno) != ("") && String.valueOf(newEncoderDos) != ("")) {
      ArmOneEncoderValue = newEncoderUno;
      ArmTwoEncoderValue = newEncoderDos;
    }
    AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;
    /*
    if (String.valueOf(newEncoderUno) == ("") && String.valueOf(newEncoderDos) == ("")) {
      ArmOneEncoderValue = newEncoderUno;
      ArmTwoEncoderValue = newEncoderDos;
    }
     */
    //controls.controls();
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
    }

  // 41 inches
  // 2988.303 is multiplier by inches

  mainlimit = 122520.423;
  // Setting extension limits
  if (AverageArmEncoderValue <= -15){
      // number in parameters + (number in parenthese subtracted from AverageArmValue) must equal division factor
      limitfactor = (AverageArmEncoderValue - 1) / -16;
      maxextensionlimit = mainlimit / limitfactor;
  }
  else{
      // resetting limits
      maxextensionlimit = mainlimit;
  }

  // Release the piston while the arm is being extended & retracted
  if (Xbox.getRawButtonPressed(2) || Xbox.getRawButtonPressed(1)){
      Piston.set(true);
  }
  // Stops firing piston while arm is extended & retracted
  if (Xbox.getRawButtonReleased(1) || Xbox.getRawButtonReleased(2)){
      Piston.set(false);
  }

  if (Xbox.getRawButton(8)){
    // arm to go
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
  // Between ___ & 41 inches, the arm can retract & extend
      if (extensionvalue <= maxextensionlimit && extensionvalue > 0.0){
        if (Xbox.getRawButton(1)){
            ExtensionMotorOne.set(0.3);
            ExtensionMotorTwo.set(0.3);
            currentextend = ExtensionMotorOne.getSelectedSensorPosition();
        }
        else if (Xbox.getRawButton(2)){
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
        }
        else {
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
          Piston.set(false);
          currentextend = ExtensionMotorOne.getSelectedSensorPosition();
        }
      }
      // arm only retract
      if (extensionvalue >= maxextensionlimit){
        SmartDashboard.putString("DB/String 2", ("i am too long"));
          if (Xbox.getRawButton(2)){
            ExtensionMotorOne.set(-0.3);
            ExtensionMotorTwo.set(-0.3);
            currentextend = ExtensionMotorOne.getSelectedSensorPosition();
          }
          else {
            ExtensionMotorOne.set(-0.2);
            ExtensionMotorTwo.set(-0.2);
            Piston.set(true);
          }
      }
      // arm only extend
      if (extensionvalue <= 0.0) {
        SmartDashboard.putString("DB/String 2", ("i am too short"));
        if (Xbox.getRawButton(1)){
          ExtensionMotorOne.set(0.3);
          ExtensionMotorTwo.set(0.3);
          currentextend = ExtensionMotorOne.getSelectedSensorPosition();
        }
        else{
          ExtensionMotorOne.set(0.0);
          ExtensionMotorTwo.set(0.0);
          currentextend = ExtensionMotorOne.getSelectedSensorPosition();
        }
      }
//47 heheheheheheheh I LOVE CODE HEHEHEHEHEHEHEH
      if ((AverageArmEncoderValue >= -25) && (AverageArmEncoderValue <= 0)){
          // autopreset for cube
          if (Xbox.getRawButton(6)){
              SmartDashboard.putString("DB/String 2", ("6666666666666666"));
              ArmUpOne.set(-0.25);
              ArmUpTwo.set(0.25);
              currentarm = ArmOneEncoder.getPosition();
          }
          else if (Xbox.getRawButton(5)){
              SmartDashboard.putString("DB/String 2", ("555555555555"));
              ArmUpOne.set(0.25);
              ArmUpTwo.set(-0.25);
              currentarm = ArmOneEncoder.getPosition();
          }
          else{
              ArmUpOne.set(0);
              ArmUpTwo.set(0);
              currentarm = ArmOneEncoder.getPosition();
          }
      }

      if (AverageArmEncoderValue <= -25){
          if (Xbox.getRawButton(6)){
              ArmUpOne.set(-0.25);
              ArmUpTwo.set(0.25);
              currentarm = ArmOneEncoder.getPosition();
          }
          else{
              ArmUpOne.set(0);
              ArmUpTwo.set(0);
              currentarm = ArmOneEncoder.getPosition();
          }
      }
      if (AverageArmEncoderValue >= 0){
          if (Xbox.getRawButton(5)){
              ArmUpOne.set(0.25);
              ArmUpTwo.set(-0.25);
              currentarm = ArmOneEncoder.getPosition();
          }
          else{
              ArmUpOne.set(0);
              ArmUpTwo.set(0);
              currentarm = ArmOneEncoder.getPosition();
          }
      }
  if (Xbox.getRawAxis(3) >= 0.1){
      SRX_1.set(-0.6);
      SRX_2.set(-0.6);
      SRX_3.set(-0.6);
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
  }

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

  // setting base values for teleop
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
  }

  @Override
  public void testPeriodic() {

    SmartDashboard.putString("DB/String 1", "ArmValue" + String.valueOf(AverageArmEncoderValue));

    if (Xbox.getRawButton(2)) {
      ArmUpOne.set(-0.25);
      ArmUpTwo.set(0.25);    
    }
    else if (Xbox.getRawButton(1)) {
      ArmUpOne.set(0.25);
      ArmUpTwo.set(-0.25);
    }
    else {
      ArmUpOne.set(0);
      ArmUpTwo.set(0);
    }
  }
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
