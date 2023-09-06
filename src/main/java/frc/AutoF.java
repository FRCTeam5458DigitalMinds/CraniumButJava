package frc;

import frc.robot.Robot;

public class AutoF {

  Robot robot = new Robot();
  public long last = System.currentTimeMillis();

  public void autoF1(){
    if (robot.autoChooser == "1")
    {
      // Goes forwards towards the charge station inversed motors
      if (robot.autoStep == 1 && robot.AverageEncoderValue <= 26.5)
      {
        robot.speed = 0.4;
        robot.FrontRightMotor.set(robot.speed);
        robot.FrontLeftMotor.set(-robot.speed);

        robot.autoStep++;
      }
      // makes robot stop on top of charge station
      else if (robot.autoStep == 2 && robot.AverageEncoderValue >= 26.5)
      {
        robot.ROLL = robot.gyro.getRoll() - 2;

        if (robot.ROLL >= -1 && robot.ROLL <= 1)
        {
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
        }
        if (robot.ROLL >= 1)
        {
          robot.FrontLeftMotor.set(0.3);
          robot.FrontRightMotor.set(-0.3);
        }
        else if (robot.ROLL <= -1)
        {
          robot.FrontLeftMotor.set(-0.3);
          robot.FrontRightMotor.set(0.3);
        }
      }
    }
  }

  public void autoF2() {
    if (robot.autoChooser == "2"){
      if (robot.autoStep == 1){
        robot.IntakePiston.set(false);
        robot.Intake.set(-0.3);
        long now = System.currentTimeMillis();
        if(now - last >= 2000){
          robot.Intake.set(0);
          robot.autoStep++;
        }
      }

      if (robot.autoStep == 2 && robot.AverageEncoderValue >= -37){
        robot.speed = 0.4;
        robot.FrontRightMotor.set(-robot.speed);
        robot.FrontLeftMotor.set(robot.speed);
        robot.autoStep++;
      }
      else if (robot.autoStep == 3 && robot.AverageEncoderValue <= -37){
        robot.FrontLeftMotor.set(0);
        robot.FrontRightMotor.set(0);
      }
    }
  }

  public void autoF3(){
    // Auto 3: Score and leave the community
    if (robot.autoChooser == "3"){

      // Arm and Extension (Scoring during auto)
      robot.extensionvalue = robot.ExtensionMotorOne.getSelectedSensorPosition();

      if (robot.autoStep == 1){
        //robot.ClawMotor.set(0.3);
        robot.SRX_1.set(1);
        robot.SRX_2.set(1);
        robot.SRX_3.set(1);
        robot.Vent1.set(false);
        robot.Vent2.set(false);
        robot.Vent3.set(false);
        if (robot.AverageArmEncoderValue >= robot.highscorearm){
          robot.ArmUpOne.set(0.2);
          robot.ArmUpTwo.set(-0.2);
        }
        else{
          robot.ArmUpOne.set(0);
          robot.ArmUpTwo.set(0);
        }
        if (robot.extensionvalue <= robot.highscoreextend){
          robot.ExtensionMotorOne.set(0.3);
          robot.ExtensionMotorTwo.set(0.3);
        }
        else{
          robot.ExtensionMotorOne.set(0);
          robot.ExtensionMotorTwo.set(0);
        }
        if (robot.extensionvalue >= robot.highscoreextend && robot.AverageArmEncoderValue <= robot.highscorearm){
          robot.autoStep++;
          last = System.currentTimeMillis();
        }
      }
      // this will drop the cube
      else if (robot.autoStep == 2){
        long now = System.currentTimeMillis();
        if (now - last <= 2000){
          robot.SRX_1.set(0);
          robot.SRX_2.set(0);
          robot.SRX_3.set(0);
          robot.Vent1.set(true);
          robot.Vent2.set(true);
          robot.Vent3.set(true);
          //robot.ClawMotor.set(-0.3);
        }

        if (now - last >= 2000){
          if (robot.AverageArmEncoderValue <= -2){
            robot.ArmUpOne.set(-0.2);
            robot.ArmUpTwo.set(0.2);
          }
          else{
            robot.ArmUpOne.set(0);
            robot.ArmUpTwo.set(0);
          }
          if (robot.extensionvalue >= 3000){
            robot.ExtensionMotorOne.set(-0.3);
            robot.ExtensionMotorTwo.set(-0.3);
          }
          else{
            robot.ExtensionMotorOne.set(0);
            robot.ExtensionMotorTwo.set(0);
            //robot.autoStep++;
          }
        }
      }
      // this will go out of the community
      else if (robot.autoStep == 3){
        if (robot.YAW <= 3 && robot.YAW >= -3){
          if (robot.autoStep == 2 && robot.AverageEncoderValue >= -30){
            robot.speed = -0.3;
            robot.FrontRightMotor.set(robot.speed);
            robot.FrontLeftMotor.set(-robot.speed);
          }
        }
        else{
          if (robot.AverageEncoderValue >= -30){
            if (robot.YAW >= 3){
              robot.FrontRightMotor.set(robot.speed);
              robot.FrontLeftMotor.set(-robot.speed * 0.7);
            }
            if (robot.YAW <= -3){
              robot.FrontRightMotor.set(robot.speed * 0.7);
              robot.FrontLeftMotor.set(-robot.speed);
            }
          }
        }
      }
    }
  }

  public void autoF4(){
    // Auto 4: Scoring High
    if (robot.autoChooser == "4")
    {
      // Arm and Extension (Scoring during auto)
      if (robot.autoStep == 1)
      {
        if (robot.AverageArmEncoderValue >= robot.highscorearm)
        {
          robot.ArmUpOne.set(0.2);
          robot.ArmUpTwo.set(-0.2);
        }
        else
        {
          robot.ArmUpOne.set(0);
          robot.ArmUpTwo.set(0);
        }
        if (robot.extensionvalue <= robot.highscoreextend)
        {
          robot.ExtensionMotorOne.set(0.3);
          robot.ExtensionMotorTwo.set(0.3);
        }
        else
        {
          robot.ExtensionMotorOne.set(0);
          robot.ExtensionMotorTwo.set(0);
        }
        if (robot.extensionvalue >= robot.highscoreextend && robot.AverageArmEncoderValue <= robot.highscorearm)
        {
          robot.autoStep++;
          last = System.currentTimeMillis();
        }
      }
      else if (robot.autoStep == 2)
      {
        long now = System.currentTimeMillis();
        if (now - last >= 2000)
        {
          robot.SRX_1.set(0);
          robot.SRX_2.set(0);
          robot.SRX_3.set(0);
          //robot.ClawMotor.set(-0.3);
          //added
        }

        if (now - last >= 2000)
        {
          if (robot.AverageArmEncoderValue <= 7)
          {
            robot.ArmUpOne.set(-0.2);
            robot.ArmUpTwo.set(0.2);
          }
          else
          {
            robot.ArmUpOne.set(0);
            robot.ArmUpTwo.set(0);
          }
          if (robot.extensionvalue >= 3000)
          {
            robot.ExtensionMotorOne.set(-0.3);
            robot.ExtensionMotorTwo.set(-0.3);
          }
          else
          {
            robot.ExtensionMotorOne.set(0);
            robot.ExtensionMotorTwo.set(0);
          }
        }
      }
    }
  }

  public void autoF5(){
    if (robot.autoChooser == "5"){
      if (robot.autoStep == 1){
        robot.IntakePiston.set(false);
        robot.Intake.set(-0.4);
        long now = System.currentTimeMillis();
        if (now - last >= 1500){
          robot.Intake.set(0);
          robot.autoStep++;
        }
        
      }
      // Leave Community
      if (robot.autoStep == 2){
        if (robot.YAW <= 3 && robot.YAW >= -3){
          if (robot.autoStep == 2 && robot.AverageEncoderValue >= -45){
            robot.speed = -0.3;
            robot.FrontRightMotor.set(robot.speed * 0.9);
            robot.FrontLeftMotor.set(-robot.speed);
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
          }
        }
        else{
          if (robot.AverageEncoderValue >= -45){
            if (robot.YAW >= 3){
              robot.FrontRightMotor.set(robot.speed);
              robot.FrontLeftMotor.set(-robot.speed * 0.7);
            }
            if (robot.YAW <= -3){
              robot.FrontRightMotor.set(robot.speed * 0.7);
              robot.FrontLeftMotor.set(-robot.speed);
            }
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
          }
        }
      }
    }
  }

  public void autoF6(){
    if (robot.autoChooser == "6"){
      if (robot.autoStep == 1){
        robot.IntakePiston.set(false);
        robot.Intake.set(-0.4);
        long now = System.currentTimeMillis();
        if (now - last >= 1500){
          robot.Intake.set(0);
          robot.autoStep++;
          last = System.currentTimeMillis();
        }
      }
      if (robot.autoStep == 2){
        if (robot.YAW <= 173){
          robot.speed = 0.15;
          robot.FrontLeftMotor.set(robot.speed);
          robot.FrontRightMotor.set(robot.speed);
        }
        else{
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
          robot.RightEncoder.setPosition(0);
          robot.LeftEncoder.setPosition(0);
          long now = System.currentTimeMillis();
          if (now - last >= 500){
            robot.autoStep++;
          }
        }
      }
      if (robot.autoStep == 3 && robot.AverageEncoderValue < 31){
        robot.speed = 0.5;
        robot.FrontRightMotor.set(robot.speed);
        robot.FrontLeftMotor.set(-robot.speed);
      }
      if (robot.autoStep == 3 && robot.AverageEncoderValue >= 31){
        robot.ROLL = robot.gyro.getRoll() - 2;
        if (robot.ROLL >= -3 && robot.ROLL <= 3){
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
        }
        if (robot.ROLL >= 3){
          if (robot.YAW <= 183 && robot.YAW >= 177){
            robot.FrontLeftMotor.set(0.1);
            robot.FrontRightMotor.set(-0.09368259);
          }
          else if (robot.YAW >= 183){
            robot.FrontLeftMotor.set(0.1);
            robot.FrontRightMotor.set(-0.07);
          }
          else if (robot.YAW <= 177){
            robot.FrontLeftMotor.set(0.07);
            robot.FrontRightMotor.set(-0.1);
          }
        }
        else if (robot.ROLL <= -3){
          if (robot.YAW <= 183 && robot.YAW >= 177){
            robot.FrontLeftMotor.set(-0.1);
            robot.FrontRightMotor.set(0.09368259);
          }
          else if (robot.YAW >= 183){
            robot.FrontLeftMotor.set(-0.07);
            robot.FrontRightMotor.set(0.1);
          }
          else if (robot.YAW <= 177){
            robot.FrontLeftMotor.set(-0.1);
            robot.FrontRightMotor.set(0.07);
          }
        }
      }
    }
  }

  public void autoF7(){
    long now = System.currentTimeMillis();
    if (robot.autoChooser == "7"){
    robot.IntakePiston.set(false);
    robot.Intake.set(-0.4);
      if (now - last >= 1500){
        robot.Intake.set(0);
        robot.autoStep++;
      }
    }
  }

  public void autoF8(){
    long now = System.currentTimeMillis();
    if (robot.autoChooser == "8"){
      if (robot.autoStep == 1){
        robot.Intake.set(-0.4);
        if (now - last >= 1500){
          robot.Intake.set(0);
          robot.autoStep++;
        }
      }
      if (robot.autoStep == 2){
        if (robot.YAW <= 173){
          robot.speed = 0.15;
          robot.FrontLeftMotor.set(robot.speed);
          robot.FrontRightMotor.set(robot.speed);
        }
        else{
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
          robot.LeftEncoder.setPosition(0);
          robot.RightEncoder.setPosition(0);
          robot.autoStep++;
        }
      }
      if (robot.autoStep == 3){
        if (robot.YAW <= 180 && robot.YAW >= 174){
          if (robot.AverageEncoderValue <= 55){
            robot.speed = 0.3;
            robot.FrontRightMotor.set(robot.speed);
            robot.FrontLeftMotor.set(-robot.speed);
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
            robot.autoStep++;
          }
        }
        else{
          if (robot.AverageEncoderValue <= 55){
            if (robot.YAW >= 180){
              robot.FrontRightMotor.set(robot.speed * 0.7);
              robot.FrontLeftMotor.set(-robot.speed);
            }
            if (robot.YAW <= 174){
              robot.FrontRightMotor.set(robot.speed);
              robot.FrontLeftMotor.set(-robot.speed * 0.7);
            }
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
            robot.Intake.set(0);
            robot.LeftEncoder.setPosition(0);
            robot.RightEncoder.setPosition(0);

            robot.autoStep++;
            last = System.currentTimeMillis();
          }
        }
      }
      if (robot.autoStep == 4){
        robot.IntakePiston.set(true);
        robot.Intake.set(0.4);
        if (now - last >= 2000){
          robot.IntakePiston.set(false);
          robot.Intake.set(0);
          robot.autoStep = 5;
        }
      }
      if (robot.autoStep == 5){
        if (robot.YAW >= 7){
          robot.speed = -0.15;
          robot.FrontLeftMotor.set(robot.speed);
          robot.FrontRightMotor.set(robot.speed);
        }
        else{
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
          robot.LeftEncoder.setPosition(0);
          robot.RightEncoder.setPosition(0);
          robot.autoStep++;
        }
      }
      if (robot.autoStep == 6){
        if (robot.YAW <= 3 && robot.YAW >= -3){
          if (robot.AverageEncoderValue <= 55){
            robot.speed = 0.3;
            robot.FrontRightMotor.set(robot.speed);
            robot.FrontLeftMotor.set(-robot.speed);
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
            robot.autoStep++;
          }
        }
        else{
          if (robot.AverageEncoderValue <= 55){
            if (robot.YAW >= 3){
              robot.FrontRightMotor.set(robot.speed * 0.7);
              robot.FrontLeftMotor.set(-robot.speed);
            }
            if (robot.YAW <= -3){
              robot.FrontRightMotor.set(robot.speed);
              robot.FrontLeftMotor.set(-robot.speed * 0.7);
            }
          }
          else{
            robot.FrontRightMotor.set(0);
            robot.FrontLeftMotor.set(0);
            // autoStep++;
          }
        }
      }
    }
  }

  public void autoF9(){
    if (robot.autoChooser == "9"){
      long now = System.currentTimeMillis();
      // dispersing cube
      if (robot.autoStep == 1){
        robot.Intake.set(-0.48);
        if (now - last >= 500){
          robot.Intake.set(0);
          robot.autoStep = 2;
        }
      }
      // 180 degree turn
      if (robot.autoStep == 2){
        if (robot.YAW <= 173){
          robot.speed = 0.15;
          robot.FrontLeftMotor.set(robot.speed);
          robot.FrontRightMotor.set(robot.speed);
        }
        else{
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
          robot.LeftEncoder.setPosition(0);
          robot.RightEncoder.setPosition(0);
          robot.autoStep = 3;
        }
      }
      // over charging station
      if (robot.autoStep == 3 && robot.AverageEncoderValue < 40){
        robot.speed = 0.3;
        robot.FrontRightMotor.set(robot.speed * 0.9);
        robot.FrontLeftMotor.set(-robot.speed);
      }
      else{
        robot.autoStep++;
      }
      // turning again
      if (robot.autoStep == 4){
        if (robot.YAW >= 7){
          robot.speed = -0.15;
          robot.FrontLeftMotor.set(robot.speed);
          robot.FrontRightMotor.set(robot.speed);
        }
        else{  
          robot.gyro.setYaw(0);
          robot.LeftEncoder.setPosition(0);
          robot.RightEncoder.setPosition(0);
          robot.autoStep++;
        }
      }
      // onto charging station
      if (robot.autoStep == 5 && robot.AverageEncoderValue < 15){
        robot.speed = 0.3;
        robot.FrontRightMotor.set(robot.speed);
        robot.FrontLeftMotor.set(-robot.speed * 0.9);
      }
      // balancing charging station
      if (robot.autoStep == 5 && robot.AverageEncoderValue >= 15){
        robot.ROLL = robot.gyro.getRoll() - 2;
        if (robot.ROLL >= -3 && robot.ROLL <= 3){
          robot.FrontLeftMotor.set(0);
          robot.FrontRightMotor.set(0);
        }
        if (robot.ROLL >= 3){
          if (robot.YAW <= 3 && robot.YAW >= -3){
            robot.speed = -0.07;
            robot.FrontLeftMotor.set(0.1);
            robot.FrontRightMotor.set(-0.09);
          }
          else if (robot.YAW >= 3){
            robot.FrontLeftMotor.set(0.1 * 0.7);
            robot.FrontRightMotor.set(-0.09);
          }
          else if (robot.YAW <= -3){
            robot.FrontLeftMotor.set(0.1);
            robot.FrontRightMotor.set(-0.09 * 0.7);
          }
        }
        else if (robot.ROLL <= -3){
          if (robot.YAW <= 3 && robot.YAW >= -3){
            robot.FrontLeftMotor.set(-0.1);
            robot.FrontRightMotor.set(0.09);
          }
          else if (robot.YAW >= 3){
            robot.FrontLeftMotor.set(-0.1 * 0.7);
            robot.FrontRightMotor.set(0.09);
          }
          else if (robot.YAW <= -3){
            robot.FrontLeftMotor.set(-0.1);
            robot.FrontRightMotor.set(0.09 * 0.7);
          }
        }
      }
    }
  }
}