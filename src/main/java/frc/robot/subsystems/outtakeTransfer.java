// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class outtakeTransfer extends SubsystemBase {
  CANSparkMax outtakeMotor;
  double direction;
  private final XboxController operatorController = new XboxController(1);

  public outtakeTransfer() {
    outtakeMotor = new CANSparkMax(17, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void wheelMovement() {

    direction = operatorController.getPOV(0);

    if(direction == 0) {
      outtakeMotor.set(.55);
    } 
    else if (direction == 315 || direction == 45) {
      outtakeMotor.set(.25);
    }
    else if(direction == 180) {
      outtakeMotor.set(-.55);
    } 
    else if (direction == 225 || direction == 135) {
      outtakeMotor.set(-.25);
    }
    else {
      outtakeMotor.set(0);
    }
  }
}
