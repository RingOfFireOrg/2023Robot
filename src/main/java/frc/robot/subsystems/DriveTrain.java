package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveTrain extends SubsystemBase {
  private static final int FrontLeftChannel = 2;
  private static final int BackLeftChannel = 3;
  private static final int FrontRightChannel = 1;
  private static final int BackRightChannel = 0;
  //private static final int kJoystickChannel = 0;

  public RelativeEncoder frontLeftEncoder;
  public RelativeEncoder frontRightEncoder;
  public RelativeEncoder backLeftEncoder;
  public RelativeEncoder backkRightEncoder;

  private MecanumDrive m_robotDrive;
  //private Joystick m_stick;

  CANSparkMax frontLeft = new CANSparkMax(FrontLeftChannel,MotorType.kBrushless);
  CANSparkMax backLeft = new CANSparkMax(BackLeftChannel,MotorType.kBrushless);
  CANSparkMax frontRight = new CANSparkMax(FrontRightChannel,MotorType.kBrushless);
  CANSparkMax backRight = new CANSparkMax(BackRightChannel,MotorType.kBrushless);

  public DriveTrain() {
    


    frontRight.setInverted(true);
    backRight.setInverted(true);

	//  Invert Left if robots Backwards


    m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    //m_stick = new Joystick(kJoystickChannel);

  }


  public CommandBase joystickDrive(double y,double x,double z) {

    return run(() -> m_robotDrive.driveCartesian(y,x,z)).withName("joystickDrive");
  
  }






  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
  }

}
