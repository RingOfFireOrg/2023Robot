// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.SwerveJoystickCommand;
import frc.robot.commands.robotOriented;

public class OrientationSchedular extends CommandBase  {
  private final XboxController driveController = new XboxController(0);
  private final SwerveSubsystem swerveSubsystem;
  SwerveJoystickCommand fieldOrient;
  robotOriented robotOrient;


  public OrientationSchedular(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);  

    this.swerveSubsystem = swerveSubsystem;
    this.fieldOrient = fieldOrient;
    this.robotOrient = robotOrient;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if (driveController.getRawAxis(0) >= 0.1 || driveController.getRawAxis(0) <= -0.1 || driveController.getRawAxis(1) >= 0.1 || driveController.getRawAxis(1) <= -0.1) 
    {
      CommandScheduler.getInstance().schedule(fieldOrient);
      
    }
    else {
      CommandScheduler.getInstance().schedule(robotOrient);
    }
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
