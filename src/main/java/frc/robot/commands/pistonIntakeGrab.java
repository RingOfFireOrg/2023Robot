// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake;

public class pistonIntakeGrab extends CommandBase {
  /** Creates a new pistonIntake. */

  pistonIntake pistonIntake;

  public pistonIntakeGrab(pistonIntake pistonIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pistonIntake);
    this.pistonIntake = pistonIntake;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    pistonIntake.joystickControl();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
