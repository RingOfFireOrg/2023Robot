package frc.robot.commands.AutoCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.pistonIntake; 

public class PistonIntakeMovement extends CommandBase {
  pistonIntake piston;
  String status;
  boolean CMDStatus;
  Timer timer;

  public PistonIntakeMovement(pistonIntake piston, String status) {
    addRequirements(piston);
    this.piston = piston;
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
    if (status == "up") {
      CMDStatus =  piston.intakeUp();
    }
    else if (status == "down") {
      CMDStatus =  piston.intakeDown();
    }
    else if (status == "downTimed") {
      piston.wheelieMotorSet(-0.1);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if (CMDStatus == true) {
      return true;
    }
    else if (timer.hasElapsed(0.3)){
      return true;
    }
    else {
      return false;
    }
  }
}
