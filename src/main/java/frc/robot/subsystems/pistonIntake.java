package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.TeleopCommands.pistonIntakeGrab;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class pistonIntake extends SubsystemBase {
  CANSparkMax intakeActuator;
  RelativeEncoder intakeEncoder;

  double intakePosition;

  DoubleSolenoid intake;
  DoubleSolenoid piston2;
  
  private final XboxController operatorController = new XboxController(1);
 
  public pistonIntake() {
    intakeActuator = new CANSparkMax(16, MotorType.kBrushless);
    intakeEncoder = intakeActuator.getEncoder();

    intakeActuator.setInverted(true);
    intake = new DoubleSolenoid (PneumaticsModuleType.CTREPCM, 1, 0);
  }
  final DigitalInput limitSwitch = new DigitalInput(Constants.LimitSwitchDIO);
  
  private void digitalInput() {
    if(limitSwitch.get() == true) {
    intakeActuator.set(0);
  }
  
  SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());

}
public boolean getLimitSwitchBoolean() {
  return SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());
}

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());
  }



  public boolean open() {
    intake.set(Value.kReverse);
    return true;
  }

  public boolean close() {
    intake.set(Value.kForward);
    return true;
  }
  public boolean off() {
    intake.set(Value.kOff);
    return true;
  }
  public boolean intakeDown() {
    while (intakeEncoder.getPosition() > -22 ) {
      intakeActuator.set(-.4);
    }
    intakeActuator.set(0);
    return true;
  }
  public boolean intakeUp() {
    while(intakeEncoder.getPosition() < 1) {
      intakeActuator.set(.4);
    }
    intakeActuator.set(0);
    return true;
  }


  public void joystickControl() {

    SmartDashboard.putBoolean("LimitSwitch: ", limitSwitch.get());

    
    //Opening and closing intake
    boolean xButton = operatorController.getRawButton(3);
    boolean bButton = operatorController.getRawButton(2);

    boolean yButton = operatorController.getRawButton(4);
    boolean aButton = operatorController.getRawButton(1);
    //Transfering up and down
    double rightStickY = -operatorController.getRawAxis(5);


    if(xButton) {
      intake.set(Value.kReverse);
    } 
    else if (bButton) {
      intake.set(Value.kForward);
    } 
    else {
      intake.set(Value.kOff);
    }

    if(rightStickY < 0.1) {
      intakeActuator.set(rightStickY/2);
      //intakeUp();
    }
    else if(limitSwitch.get() == false && rightStickY > 0.1) {
      intakeActuator.stopMotor();
    }
    else if((rightStickY < -0.1 || rightStickY > 0.1) && limitSwitch.get() == true) {
      intakeActuator.set(rightStickY/2);
      //intakeDown();
    } 
    else {
      intakeActuator.stopMotor();
    }
    if (yButton) {
      while ( intakeEncoder.getPosition() < -12) {
        intakeActuator.set(.4);
      }
      intakeActuator.set(0);

      //intakeActuator.set(.2);
    }
    else if (aButton) {
      while ( intakeEncoder.getPosition() > 5) {
        intakeActuator.set(-.4);
      }
      intakeActuator.set(0);

    }
    SmartDashboard.putNumber("Right Stick Y", rightStickY);

  
    intakePosition = intakeEncoder.getPosition();
    SmartDashboard.putNumber("Intake Position", intakePosition);
  }

  

}



