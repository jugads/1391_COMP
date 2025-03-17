// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.HopperConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax beltMotor;
  SparkMax wheelMotor;
  double timer = 0;
  public Hopper() {
    beltMotor = new SparkMax(kBeltMotorID, MotorType.kBrushless);
    wheelMotor = new SparkMax(kWheelMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {

  }
  public void runBeltMotor(double speed) {
    beltMotor.set(speed);
  }
  public void stopBeltMotor() {
    beltMotor.set(0);
  }
  public void runWheelMotor(double speed) {
    wheelMotor.set(speed);
  }
  public void stopWheelMotor() {
    wheelMotor.set(0);
  }
  public void runBoth(double wheelSpeed, double beltSpeed) {
    beltMotor.set(beltSpeed);
    wheelMotor.set(wheelSpeed);
  }
  public void setup() {
    timer+=1;
    if (timer == 15) {
    beltMotor.set(0.);
    timer = 0;
    }
    else if (timer > 3) {
      beltMotor.set(1.);
      wheelMotor.set(0.2);
    }
  }
}