package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake; 

public class PistonIntakeMovement extends CommandBase {
  pistonIntake piston;
  String status;
  boolean CMDStatus;
  public PistonIntakeMovement(pistonIntake piston, String status) {
    addRequirements(piston);
    this.piston = piston;
    this.status = status;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (status == "up") {
      CMDStatus =  piston.intakeUp();
    }
    else if (status == "down") {
      CMDStatus =  piston.intakeDown();
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
