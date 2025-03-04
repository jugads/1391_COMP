// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.kElevTran;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/**
 * Controls arm position using closed-loop control:
 * - PID control for position accuracy
 * - Feedforward for gravity compensation
 * - Position limits based on elevator state
 */
public class ArmCommand extends Command {
  /** Creates a new ArmCommand. */
  // kS=0.0, kG=0.02, kV=0.0 for basic gravity compensation
  ArmFeedforward ff = new ArmFeedforward(0., 0.02, 0);
  Arm arm;
  Elevator elevator;
  // Higher P gain (2.0) for quick response, small D gain (0.1) for oscillation damping
  PIDController controller = new PIDController(2., 0, 0.1);
  
  public ArmCommand(Arm arm, Elevator elevator) {
    this.arm = arm;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Position limits:
    // Near elevator transition (±0.05 units): [-0.23, 0.25]
    // Otherwise: [0.05, 0.25]
    var armSetpoint =  MathUtil.clamp(
    arm.getSetpoint(), 
    (Math.abs((kElevTran - elevator.getElevatorPosition())) < 0.05) ? -0.23 : 0.05, 
    0.25
    );
    SmartDashboard.putNumber("Arm Setpoint", armSetpoint);
    
    // Combine PID and feedforward outputs, scaled to 85% for safety margin
    var pidSpeed = controller.calculate(arm.getEncoderPosition(), armSetpoint);
    arm.runMotor(0.85*(ff.calculate(armSetpoint, pidSpeed)+pidSpeed));
    // arm.runMotor(0.);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
