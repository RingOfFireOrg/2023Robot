package frc.robot.Auto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;



public class newBalance extends CommandBase {
    private final SwerveSubsystem drivetrainSubsystem;
    private final Timer timer;

    public newBalance(SwerveSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;

        timer = new Timer();
        timer.start();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if(drivetrainSubsystem.getPitchAsRotation2d().getDegrees() <= -10) {
            drivetrainSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(0.5, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));
        } else if(drivetrainSubsystem.getPitchAsRotation2d().getDegrees() <= 10){
            drivetrainSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(-0.5, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));
//        } else if(drivetrainSubsystem.getPitch().getDegrees() <= -5){
//            drivetrainSubsystem.drive(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
//                    ChassisSpeeds.fromFieldRelativeSpeeds(0.3, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));
//
//    } else if(drivetrainSubsystem.getPitch().getDegrees() <= 5){
//        drivetrainSubsystem.drive(DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
//                ChassisSpeeds.fromFieldRelativeSpeeds(-0.3, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));

    }
        else if(drivetrainSubsystem.getPitchAsRotation2d().getDegrees() <= -3){
            drivetrainSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(0.1, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));

        } else if(drivetrainSubsystem.getPitchAsRotation2d().getDegrees() <= 3){
            drivetrainSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(-0.1, 0.0, 0.0, drivetrainSubsystem.getPose().getRotation())));

        } else{
            drivetrainSubsystem.stopModules();
        }


}

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(0.75)) {
            timer.reset();
            return Math.abs(drivetrainSubsystem.getPitchAsRotation2d().getDegrees()) <= 1;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
