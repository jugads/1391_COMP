// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Elevator;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  // Feedforward controller to compensate for gravity and system dynamics
  // Parameters: kS (static friction), kG (gravity), kV (velocity)
  ElevatorFeedforward ff = new ElevatorFeedforward(0, 0.0625, 1.55); //1.65
  
  // PID controller for position control
  // Parameters: kP (proportional), kI (integral), kD (derivative)
  PIDController pid = new PIDController(1.55, 0, 0.05);
  
  /** Creates a new ElevatorCommand. */
  Elevator elevator;
  AlgaeScorer algae;
  // Constructor: takes elevator subsystem as parameter
  public ElevatorCommand(Elevator elevator, AlgaeScorer algae) {
    this.elevator = elevator;
    this.algae = algae;
    // Register this elevator subsystem as a requirement for this command
    // This prevents multiple commands from controlling the elevator simultaneously
    addRequirements(elevator);
  }

  // Initialization method - called once when command starts
  @Override
  public void initialize() {}

  // Main execution loop - called repeatedly while command is running
  @Override
  public void execute() {
    // Calculate motor output using feedforward and PID control
    // 1. pid.calculate gets position error and computes correction
    // 2. ff.calculate compensates for gravity and system dynamics
    // 3. MathUtil.clamp limits output between -0.6 (down) and 1.0 (up)
    elevator.runElevatorUp(MathUtil.clamp(
      ff.calculate(
        -pid.calculate(elevator.getSetpoint(), elevator.getElevatorPosition())
      ), 
      algae.hasAlgae() ? -0.2 : -0.65, algae.hasAlgae() ? 0.3 : 0.65
    ));
  }

  // Cleanup method - called when command ends
  @Override
  public void end(boolean interrupted) {}

  // Determines if command should stop running
  // Returns false to run continuously until explicitly interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
