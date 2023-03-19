// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake;

public class PistonEncoderMovement extends CommandBase {
  pistonIntake piston;
  double pistonPos;
  public PistonEncoderMovement(pistonIntake piston, double pistonPos) {
    this.piston = piston;
    this.pistonPos = pistonPos;
    addRequirements(piston);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    piston.pistonEncoder(0);
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
