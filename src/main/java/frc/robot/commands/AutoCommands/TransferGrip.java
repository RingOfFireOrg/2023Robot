// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.outtakeTransfer;

public class TransferGrip extends CommandBase {

  outtakeTransfer wheelie;
  String status;
  Timer timer;
  
  public TransferGrip(outtakeTransfer wheelie, String status) {
    addRequirements(wheelie);
    this.wheelie = wheelie;
    this.status = status;
    timer = new Timer();
    timer.start();
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    if (status == "open") {
      wheelie.wheelieSetSpeedOpen();
    }
    else if (status == "close") {
      wheelie.wheelieSetSpeedClose();
    }
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (timer.hasElapsed(0.6)) {
      timer.reset();
      return true;
  }
  return false;
  }
}
