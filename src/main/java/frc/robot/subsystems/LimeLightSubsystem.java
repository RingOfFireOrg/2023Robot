// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase {
  private NetworkTableInstance table;
  public double xFinal;
  public double yFinal;

  public LimeLightSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private NetworkTableEntry getVal(String key){
    return table.getTable("limelight").getEntry(key);
  }

  public boolean checkTarget() {
    if (getVal("tv").getDouble(0) == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public double getX() {
    return getVal("tx").getDouble(0.00);
  }

  public double getY() {
    return getVal("ty").getDouble(0.00);
  }



}
