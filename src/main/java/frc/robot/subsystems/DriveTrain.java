package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveTrain extends SubsystemBase {


  private static final int FrontLeftChannel = 2;
  private static final int BackLeftChannel = 3;
  private static final int FrontRightChannel = 1;
  private static final int BackRightChannel = 0;



  private static final int kJoystickChannel = 0;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;

  public DriveTrain() {
    CANSparkMax frontLeft = new CANSparkMax(FrontLeftChannel,MotorType.kBrushless);
    CANSparkMax backLeft = new CANSparkMax(BackLeftChannel,MotorType.kBrushless);
    CANSparkMax frontRight = new CANSparkMax(FrontRightChannel,MotorType.kBrushless);
    CANSparkMax backRight = new CANSparkMax(BackRightChannel,MotorType.kBrushless);


    frontRight.setInverted(true);
    backRight.setInverted(true);
	//  Invert Left if robots Backwards


    m_robotDrive = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    m_stick = new Joystick(kJoystickChannel);

  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_robotDrive.driveCartesian(-m_stick.getY(), -m_stick.getX(), -m_stick.getZ());
  }
}
