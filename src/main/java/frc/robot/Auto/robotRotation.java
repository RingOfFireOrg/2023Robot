// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class robotRotation extends CommandBase {
  SwerveSubsystem swerve;
  public robotRotation(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 2);
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    double value = swerve.yawVals();
    while (swerve.yawVals() - value <= 180)
      swerve.setModuleStates(moduleStates);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
