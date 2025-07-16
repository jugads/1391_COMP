// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Strategy extends SubsystemBase {
  /** Creates a new Strategy. */
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable dash;
  public Strategy() {
    dash = inst.getTable("SmartDashboard");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(getMatchStrategy());
  }
  /*public String getMatchStrategy() {
    return dash.getEntry("strategy").getString("default");
  } */
}