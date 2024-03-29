
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.SerialPort;



public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);
    
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB);



    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation2d(),
        getSwerveModulePosition(),
        new Pose2d(1, 3, new Rotation2d()));

    private final PIDController yController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
    private final PIDController xController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
    private final PIDController thetaController = new PIDController(AutoConstants.kPThetaController,0.0, 0.0);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    // public double getHeading() {
    //     if(gyro.isMoving() == true) {
    //         SmartDashboard.putNumber("Gyro Angle",gyro.getAngle());
    //     }
    //     return gyro.getAngle();
    //     //return Math.IEEEremainder(gyro.getAngle(), 360);

    // }
    public double getHeading() {
        //double temp = Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("Robot Angle in get heading", gyro.getAngle());

        double temp = (gyro.getAngle() % 360);
        if(temp < 0) {
            temp = temp + 360; 
        }
        SmartDashboard.putNumber("Gyro Angle",temp);
        return temp;
        // should be the same as: return (gyro.getAngle() % 360);

    }

    // public double gyroangle() {
    //     while(gyro.isMoving()) {
    //         return gyro.getAngle() % 360;
    //     }
    //     return gyro.getAngle();
    // }
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //     frontLeft.setDesiredState(desiredStates[0]);
    //     frontRight.setDesiredState(desiredStates[1]);
    //     backLeft.setDesiredState(desiredStates[2]);
    //     backRight.setDesiredState(desiredStates[3]);
    // }

    public void fieldCentricReset() {
        gyro.reset();
    }
    public double pitchVals() {
        return gyro.getPitch();
    }
    public double yawVals() {
        return gyro.getYaw();
    }
    public double rollVals() {
        return gyro.getRoll();
    }
    public Rotation2d getRotation2d() {
       
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getRotation2dButaDouble() {

        return getHeading();
    }

    public Pose2d getPose() {
        SmartDashboard.putString( "  Get Pose meters ", odometer.getPoseMeters().toString());
        return odometer.getPoseMeters();
    }

    void setModuleStates2(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
  
    public void resetPose(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    }
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
    }
    public boolean resetOdometry2(Pose2d pose) {
        odometer.resetPosition(getRotation2d(),getSwerveModulePosition(),pose);
        return true;
    }
    public PIDController getxController() {
        return xController;
    }

    public PIDController getyController() {
        return yController;
    }

    public PIDController getThetaController() {
        return thetaController;
    }

    @Override




    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePosition());
        SmartDashboard.putNumber("Pitch in random file idk what this is ", gyro.getPitch());
        SmartDashboard.putNumber("Roll in random file idk what this is ", gyro.getRoll());
        SmartDashboard.putNumber("Yaw in random file idk what this is ", gyro.getYaw());
        
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot Theta", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber(" Robot Tranlsation    fl", frontLeft.getDrivePosition());
        SmartDashboard.putNumber(" Robot Tranlsation    fr", frontRight.getDrivePosition());
        SmartDashboard.putNumber(" Robot Tranlsation    bl", backLeft.getDrivePosition());
        SmartDashboard.putNumber(" Robot Tranlsation    br", backRight.getDrivePosition());

    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModulePosition[] getSwerveModulePosition() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeft.getDrivePosition(), new Rotation2d(frontLeft.getTurningPosition())),
            new SwerveModulePosition(frontRight.getDrivePosition(), new Rotation2d(frontRight.getTurningPosition())),
            new SwerveModulePosition(backLeft.getDrivePosition(), new Rotation2d(backLeft.getTurningPosition())),
            new SwerveModulePosition(backRight.getDrivePosition(), new Rotation2d(backRight.getTurningPosition())),
        };
    }
    public void driveForward(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public void brake(boolean doBrake){
        if(doBrake){
            frontLeft.brake(true);
            frontRight.brake(true);
            backLeft.brake(true);
            backRight.brake(true);
        }
        else{
            frontLeft.brake(false);
            frontRight.brake(false);
            backLeft.brake(false);
            backRight.brake(false);
        }
    }

    public Rotation2d getPitchAsRotation2d() {
        // i know it gets roll and it says pitch but i kinda dont care
        return Rotation2d.fromDegrees(gyro.getRoll());
    }


    public void drive(SwerveModuleState... desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    
    public void whilePitch() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.3, 0, 0);
        SwerveModuleState[] moduleStates1 = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
        while (rollVals() < -5 || rollVals() > 5) {
          driveForward(moduleStates1);
        }
    }
    
    
}
    
    