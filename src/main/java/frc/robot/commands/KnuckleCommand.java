// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Knuckle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class KnuckleCommand extends Command {
  // Reference to the Knuckle subsystem that this command will control
  Knuckle knuckle;

  /** Creates a new KnuckleCommand */
  public KnuckleCommand(Knuckle knuckle) {
    this.knuckle = knuckle;
    // Register the knuckle subsystem as a requirement
    // This prevents multiple commands from controlling the knuckle simultaneously
    addRequirements(knuckle);
  }

  // Initialization method - called once when the command starts
  @Override
  public void initialize() {}

  // Main execution loop - called repeatedly while command is running
  @Override
  public void execute() {
    // Check if coral (game piece) is detected
    if (knuckle.hasCoral()) {
      // If coral is present, run the knuckle motor at low speed
      knuckle.setKnuckleMotorLow();
    }
    // else if (knuckle.isScoring()) {
    //   knuckle.score();
    // }
    else {
      // If no coral is detected, stop the motor
      knuckle.stopMotor();
    }
    // Commented out alternative behavior:
    // knuckle.setKnuckleMotorHigh();
  }

  // Cleanup method - called when command ends or is interrupted
  @Override
  public void end(boolean interrupted) {}

  // Determines if command should stop running
  // Returns false to run continuously until explicitly interrupted
  @Override
  public boolean isFinished() {
    return false;
  }
}
