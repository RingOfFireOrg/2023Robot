package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake;

public class PistonIntakeStatus extends CommandBase {
  pistonIntake piston;
  String status;
  boolean CMDStatus;
  public PistonIntakeStatus(pistonIntake piston, String status) {
    addRequirements(piston);
    this.piston = piston;
    this.status = status;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (status == "open") {
      CMDStatus = piston.open();
    }
    else if (status == "close") {
      CMDStatus = piston.close();
    }
    else if (status == "off") {
      CMDStatus = piston.off();
    }
   }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (CMDStatus == true) {
      return true;
    }
    return false;
  }
}
