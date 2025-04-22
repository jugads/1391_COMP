// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  SparkMax motor = new SparkMax(99, MotorType.kBrushless);
  SparkMax pivotMotor = new SparkMax(98, MotorType.kBrushless);
  DigitalInput beamBreak = new DigitalInput(6);
  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(4);
  public GroundIntake() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake Has Coral", intakeHasCoral());
  }
  public void runIntake(boolean runIn, double speed) {
    motor.set(speed * (runIn ? 1 : -1));
  }
  public boolean intakeHasCoral() {
    return !beamBreak.get();
  }
  public void runIntakePivot(double speed) {
    pivotMotor.set(speed);
  }
  public double getIntakeAngle() {
    return pivotEncoder.get();
  }
  
}
