// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  SparkMax motorOne = new SparkMax(kMotorOneID, MotorType.kBrushless);
  SparkMax motorTwo = new SparkMax(kMotorTwoID, MotorType.kBrushless);
  public Climber() {
    motorOne.getEncoder().setPosition(0.);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber encoder", getClimberPosition());
    // This method will be called once per scheduler run
  }
  public void runClimber(double speed) {
    if (getClimberPosition() < 240) {
    motorOne.set(speed);
    motorTwo.set(speed);
    }
    else {
      motorOne.stopMotor();
      motorTwo.stopMotor();
    }
  }

public double getClimberPosition() {
	return motorOne.getEncoder().getPosition();
}
  
}
