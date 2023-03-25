// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TeleopCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.outtakeTransfer;

public class outtakeTransferMovement extends CommandBase {
  outtakeTransfer wheelie;

  public outtakeTransferMovement(outtakeTransfer wheelie) {
    addRequirements(wheelie);
    this.wheelie = wheelie;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wheelie.wheelMovement();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
