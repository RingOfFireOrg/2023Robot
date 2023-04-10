// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;



///           I KNOW THIS SAYS PITCH BUT ITS ACTUUALY ROLL




public class whilePitchCMD extends CommandBase {
  SwerveSubsystem swerveSubsystem;
  public whilePitchCMD( SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    swerveSubsystem.whilePitch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}




///           I KNOW THIS SAYS PITCH BUT ITS ACTUUALY ROLL



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
