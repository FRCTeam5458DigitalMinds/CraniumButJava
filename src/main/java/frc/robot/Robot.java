// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import org.opencv.core.Scalar;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.AutoF;
import frc.Controls;

public class Robot extends TimedRobot {
  private AutoF autoF = new AutoF();
  private Controls controls = new Controls();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  //Joysticks
  public Joystick Xbox = new Joystick(2), JoyStick1 = new Joystick(2), Wheel = new Joystick(2);

  //Drivetrain motors
  public CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax MiddleLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax BackLeftMotor = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax MiddleRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax BackRightMotor = new CANSparkMax(6, MotorType.kBrushless);

  //Intake motor
  public CANSparkMax IntakeMotor = new CANSparkMax(25, MotorType.kBrushless);

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
  public RelativeEncoder ArmOneEncoder = ArmUpOne.getEncoder(), ArmTwoEncoder = ArmUpTwo.getEncoder();
  
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
    ArmTwoEncoderValue = ArmTwoEncoder.getPosition();

    extensionvalue = ExtensionMotorOne.getSelectedSensorPosition();
    autoF.autoF1();
    autoF.autoF2();
    autoF.autoF3();
    autoF.autoF4();
    autoF.autoF5();
    autoF.autoF6();
    autoF.autoF7();
    autoF.autoF8();
    autoF.autoF9();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
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

    ArmOneEncoderValue = -ArmOneEncoder.getPosition();
    ArmTwoEncoderValue = ArmTwoEncoder.getPosition();
    AverageArmEncoderValue = (ArmTwoEncoderValue + ArmOneEncoderValue) / 2;

    controls.controls();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
