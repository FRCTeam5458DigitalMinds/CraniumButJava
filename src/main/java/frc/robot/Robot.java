// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import com.revrobotics.CANEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.lang.String;
import com.ctre.phoenix.Util;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSensor;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.util.Units; 
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.CompressorConfigType;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import java.lang.Math;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.cameraserver.CameraServer;
import java.util.Timer;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  //Joysticks
  Joystick xbox = new Joystick(2), joystick = new Joystick(2), wheel = new Joystick(2);

  //Drivetrain motors
  CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax MiddleLeftMotor = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax BackLeftMotor = new CANSparkMax(5, MotorType.kBrushless);
  CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax MiddleRightMotor = new CANSparkMax(4, MotorType.kBrushless);
  CANSparkMax BackRightMotor = new CANSparkMax(6, MotorType.kBrushless);

  //Intake motor
  CANSparkMax IntakeMotor = new CANSparkMax(25, MotorType.kBrushless);

  //DriveTrain encoder
  

  //Arm Motors
  CANSparkMax ArmUpOne = new CANSparkMax(7, MotorType.kBrushless);
  CANSparkMax ArmUpTwo = new CANSparkMax(8, MotorType.kBrushless);

  WPI_TalonFX ClawMotor = new WPI_TalonFX(9);
  WPI_TalonFX ExtensionMotorOne = new WPI_TalonFX(10);
  WPI_TalonFX ExtensionMotorTwo = new WPI_TalonFX(11);
  WPI_TalonFX Intake = new WPI_TalonFX(25);

  //Arm encoders
  ArmOneEncoder = ArmUpOne.getEncoder(), ArmTwoEncoder = ArmUpTwo.getEncoder();
  

  

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
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
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
