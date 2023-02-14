package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.net.PasswordAuthentication;

import com.revrobotics.CANSparkMax;

public class DriveTrain extends SubsystemBase {
  private final CANSparkMax frontLeft;
  private final CANSparkMax frontRight;
  private final CANSparkMax backLeft;
  private final CANSparkMax backRight;


  public DriveTrain() {
    double fl,fr,bl,br;
    //double forward;

    // double forward = -Robot.oi.getJoystick().getY();
		// double right = Robot.oi.getJoystick().getX();
		// double clockwise = Robot.oi.getJoystick().getZ();
		double K = .01;//the value that determines sensitivity of turning tweek to edit
		clockwise = K*clockwise;
		//inverse kinimatics
		fr = forward + clockwise + right;
		fl = forward - clockwise - right;
		bl = forward + clockwise - right;
		br = forward - clockwise + right;
		frontLeft.set(fl);
		frontRight.set(fr);
		backLeft.set(bl);
		backRight.set(br);
		
		//drive = new MecanumDrive(left1,left2,right1,right2);

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run



  }
}
