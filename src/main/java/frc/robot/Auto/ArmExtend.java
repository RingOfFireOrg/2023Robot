// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.linearSlideArm;

public class ArmExtend extends CommandBase {
  linearSlideArm arm;
  String position;
  public ArmExtend(linearSlideArm arm, String position) {
    this.arm = arm;
    this.position = position;
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (position == "high") {
      arm.highCubeHeight();
    }
    else if (position == "reset") {
      arm.resetHeight();
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
