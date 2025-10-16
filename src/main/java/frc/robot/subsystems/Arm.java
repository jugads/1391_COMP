// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.ArmConstants.*;

import java.util.Map;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Arm extends SubsystemBase {
  // Hardware components for controlling the robot's arm mechanism
  SparkMax motor;
  DutyCycleEncoder encoder;
  double lastPosition;
  SimpleWidget armVal;
  boolean shouldSwing = true;
  boolean isClimbing = false;
  // Constructor initializes motor and encoder with specified ports from Constants
  public Arm() {
    motor = new SparkMax(kMotorID, MotorType.kBrushless); 
    encoder = new DutyCycleEncoder(kEncoderPort);
    lastPosition = getEncoderPosition();
    Shuffleboard.getTab("Teleoperated").addNumber("bins", () -> (getEncoderPosition()))
        .withWidget(BuiltInWidgets.kNumberBar)
        .withProperties(Map.of("min", -0.25, "max", 0.4, "step", 0.01));
  }

  // Periodic method runs repeatedly, updates dashboard with arm status
  @Override
  public void periodic() {
    // setSetpoint(getEncoderPosition());
    // This method will be called once per scheduler run
  }

  // Sets the motor speed (-1 to 1) to move the arm
  public void runMotor(double speed) {
    motor.set(speed);
  }
  public void applyVoltage(double Volts) {
    motor.setVoltage(Volts);
  }
  // Stops arm movement by setting motor speed to 0
  public void stop() {
    motor.set(0.);
  }

  public double getEncoderPosition() {
    return -(encoder.get()-0.5201);
  }
  // Checks if arm is at the transfer position (0 degrees)
  public boolean atTransferAngle() {
    return false;
  }
  public void setSwingTrue() {
    shouldSwing = true;
  }
  public void setSwingFalse() {
    shouldSwing = false;
  }
  public boolean getShouldSwing() {
    return shouldSwing;
  }
  public void setSetpoint(double encoderPosition) {
    lastPosition = encoderPosition;
  }
  public double getSetpoint() {return lastPosition;}

  public void increaseSetpoint(double d) {
    lastPosition += d; 
  }

public void setClimbing() {
	isClimbing = true;
}
public boolean isClimbing() {
  return isClimbing;
}
}