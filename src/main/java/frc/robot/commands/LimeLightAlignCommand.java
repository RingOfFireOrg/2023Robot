// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;


public class LimeLightAlignCommand extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final LimeLightSubsystem limelight;

  public LimeLightAlignCommand(SwerveSubsystem swerveSubsystem, LimeLightSubsystem limelight) {
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    addRequirements();
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (!limelight.checkTarget()) {
      this.cancel();
      return;
    }
    else if (limelight.checkTarget()) {
      if (limelight.getX() > 4 && limelight.getX() < -4) {
        swerveSubsystem.drive(DriveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0,-0.3 ,swerveSubsystem.getPose().getRotation())));
      }



    }
  }

  

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
