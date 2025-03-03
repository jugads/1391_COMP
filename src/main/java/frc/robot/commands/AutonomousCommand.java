// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousCommand extends Command {
  /** Creates a new AutonomousCommand. */
  public AutonomousCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command branches3456() {
    return Commands.sequence(
      
    );
  }
}
