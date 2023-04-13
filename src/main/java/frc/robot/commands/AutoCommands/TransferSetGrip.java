// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.outtakeTransfer;

public class TransferSetGrip extends CommandBase {

  outtakeTransfer wheelie;
  double power;
  Timer timer;
  
  public TransferSetGrip(outtakeTransfer wheelie, double power) {
    addRequirements(wheelie);
    this.wheelie = wheelie;
    this.power = power;

  }


  // THIS COMMAND WILL NEVER END THE ONLY WAY TO END IT IS TO USE A PARRALLEL CMD DEADLINE

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    wheelie.wheelieMotorSet(power);
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return false;

  }
}
