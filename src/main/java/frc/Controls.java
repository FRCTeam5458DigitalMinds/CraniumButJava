package frc;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID;

public class Controls {

    Robot robot = new Robot();

    public void controls(){
        if (robot.buttonValueTwo == true){
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 1.0);
        }
        else if (robot.buttonValueTwo == false){
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
            robot.Xbox.setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
        }
    
        // 41 inches
        // 2988.303 is multiplier by inches
    
        robot.mainlimit = 122520.423;
        // Setting extension limits
        if (robot.AverageArmEncoderValue <= -15){
            // number in parameters + (number in parenthese subtracted from AverageArmValue) must equal division factor
            robot.limitfactor = (robot.AverageArmEncoderValue - 1) / -16;
            robot.maxextensionlimit = robot.mainlimit / robot.limitfactor;
        }
        else{
            // resetting limits
            robot.maxextensionlimit = robot.mainlimit;
        }
    
        // Release the piston while the arm is being extended & retracted
        if (robot.Xbox.getRawButtonPressed(2) || robot.Xbox.getRawButtonPressed(1)){
            robot.Piston.set(true);
        }
        // Stops firing piston while arm is extended & retracted
        if (robot.Xbox.getRawButtonReleased(1) || robot.Xbox.getRawButtonReleased(2)){
            robot.Piston.set(false);
        }
    
        if (robot.Xbox.getRawButton(8)){
            // arm to go
            if (robot.extensionvalue <= robot.maxextensionlimit){
            if (robot.extensionvalue <= 75000){
                robot.Piston.set(true);
                robot.ExtensionMotorOne.set(0.3);
                robot.ExtensionMotorTwo.set(0.3);
            }
            else{
                robot.ExtensionMotorOne.set(0);
                robot.ExtensionMotorTwo.set(0);
                robot.Piston.set(false);
            }
            }
            else{
                robot.ExtensionMotorOne.set(0);
                robot.ExtensionMotorTwo.set(0);
            }
    
            if (robot.AverageArmEncoderValue >= -47){
            if (robot.AverageArmEncoderValue <= -17.5){
                robot.ArmUpOne.set(-0.25);
                robot.ArmUpTwo.set(0.25);
            }
            // arm to go down
            else if (robot.AverageArmEncoderValue >= -15.5){
                robot.ArmUpOne.set(0.25);
                robot.ArmUpTwo.set(-0.25);
            }
            else{
                robot.ArmUpOne.set(0);
                robot.ArmUpTwo.set(0);
            }
            }
            else{
                robot.ArmUpOne.set(0);
                robot.ArmUpTwo.set(0);
            }
        }
        // Between ___ & 41 inches, the arm can retract & extend
        if (robot.Xbox.getRawButton(8)){
            if (robot.extensionvalue <= robot.maxextensionlimit && robot.extensionvalue >= 0){
            if (robot.Xbox.getRawButton(1)){
                robot.ExtensionMotorOne.set(0.3);
                robot.ExtensionMotorTwo.set(0.3);
                robot.currentextend = robot.ExtensionMotorOne.getSelectedSensorPosition();
            }
            else if (robot.Xbox.getRawButton(2)){
                robot.ExtensionMotorOne.set(-0.3);
                robot.ExtensionMotorTwo.set(-0.3);
            }
            else{
                robot.ExtensionMotorOne.set(0);
                robot.ExtensionMotorTwo.set(0);
                robot.Piston.set(false);
                robot.currentextend = robot.ExtensionMotorOne.getSelectedSensorPosition();
            }
            }
            // arm only retract
            if (robot.extensionvalue >= robot.maxextensionlimit){
                if (robot.Xbox.getRawButton(2)){
                    robot.ExtensionMotorOne.set(-0.3);
                    robot.ExtensionMotorTwo.set(-0.3);
                    robot.currentextend = robot.ExtensionMotorOne.getSelectedSensorPosition();
                }
                else{
                    robot.ExtensionMotorOne.set(-0.2);
                    robot.ExtensionMotorTwo.set(-0.2);
                    robot.Piston.set(true);
                }
            }
            // arm only extend
            if (robot.extensionvalue <= 0){
                if (robot.Xbox.getRawButton(1)){
                    robot.ExtensionMotorOne.set(0.3);
                    robot.ExtensionMotorOne.set(0.3);
                    robot.currentextend = robot.ExtensionMotorOne.getSelectedSensorPosition();
                }
                else{
                    robot.ExtensionMotorOne.set(0);
                    robot.ExtensionMotorTwo.set(0);
                    robot.currentextend = robot.ExtensionMotorOne.getSelectedSensorPosition();
                }
            }
    
            if ((robot.AverageArmEncoderValue >= -47) && (robot.AverageArmEncoderValue <= 0)){
                // autopreset for cube
                if (robot.Xbox.getRawButton(6)){
                    robot.ArmUpOne.set(-0.25);
                    robot.ArmUpTwo.set(0.25);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
                else if (robot.Xbox.getRawButton(5)){
                    robot.ArmUpOne.set(0.25);
                    robot.ArmUpTwo.set(-0.25);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
                else{
                    robot.ArmUpOne.set(0);
                    robot.ArmUpTwo.set(0);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
            }
    
            if (robot.AverageArmEncoderValue <= -47){
                if (robot.Xbox.getRawButton(6)){
                    robot.ArmUpOne.set(-0.2);
                    robot.ArmUpTwo.set(0.2);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
                else{
                    robot.ArmUpOne.set(0);
                    robot.ArmUpTwo.set(0);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
            }
            if (robot.AverageArmEncoderValue >= 0){
                if (robot.Xbox.getRawButton(5)){
                    robot.ArmUpOne.set(0.25);
                    robot.ArmUpTwo.set(-0.25);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
                else{
                    robot.ArmUpOne.set(0);
                    robot.ArmUpTwo.set(0);
                    robot.currentarm = robot.ArmOneEncoder.getPosition();
                }
            }
        }
        if (robot.Xbox.getRawButtonPressed(3)){
            robot.bothTake = 2;
        }
        if (robot.Xbox.getRawButtonPressed(4)){
            robot.bothTake = 3;
        }
    
        // Cone intake
        if (robot.bothTake == 1){
            if (robot.Xbox.getRawAxis(3) >= 0.1){
                robot.coneintake = true;
            }
            if (robot.coneintake == true){
                robot.SRX_1.set(1);
                robot.SRX_2.set(1);
                robot.SRX_3.set(1);
                robot.Lights.set(true);
                robot.Vent1.set(false);
                robot.Vent2.set(false);
                robot.Vent3.set(false);
                //robot.ClawMotor.set(0.3);
            }
            else{
            robot.SRX_1.set(0);
            robot.SRX_2.set(0);
            robot.SRX_3.set(0);
            robot.Lights.set(false);
            robot.Vent1.set(true);
            robot.Vent1.set(true);
            robot.Vent1.set(true);
    
            //robot.ClawMotor.set(0);
            }
        }
    
        // turning it on based on the button pressed
        if (robot.bothTake == 2){
            robot.SRX_1.set(1);
            robot.SRX_2.set(1);
            robot.SRX_3.set(1);
            robot.Vent1.set(false);
            robot.Vent2.set(false);
            robot.Vent3.set(false);
            //robot.ClawMotor.set(0.3);
        }
        // toggling the button off based on the button pressed
        else if (robot.bothTake == 3){
            robot.coneintake = false;
            long now = System.currentTimeMillis();
            if (now - robot.last <= 3000){
                //robot.ClawMotor.set(-0.15);
    
            robot.SRX_1.set(0);
            robot.SRX_2.set(0);
            robot.SRX_3.set(0);
    
            robot.Vent1.set(true);
            robot.Vent2.set(true);
            robot.Vent3.set(true);
            }
            else{
                robot.Vent1.set(false);
            robot.Vent2.set(false);
            robot.Vent3.set(false);
    
            robot.bothTake = 1;
            robot.last = System.currentTimeMillis();
            }
        }
    
        if (robot.JoyStick1.getRawButton(1)){
            robot.Intake.set(0.5);
            robot.IntakePiston.set(true);
        }
        else if (robot.JoyStick1.getRawButton(3)){
            robot.Intake.set(-0.4);
        }
        else{
            robot.Intake.set(0);
            robot.IntakePiston.set(false);
        }
    
        // setting base values for teleop
        double WheelX = -robot.Wheel.getX();
        double JoyY = robot.JoyStick1.getY();
    
        double lmol = ((WheelX * 0.5) + (0.6 * JoyY));
        double rmor = ((WheelX * 0.5) - (0.6 * JoyY)) * 0.9368259;
    
        robot.FrontLeftMotor.set(lmol);
        robot.FrontRightMotor.set(rmor);
        }
}
