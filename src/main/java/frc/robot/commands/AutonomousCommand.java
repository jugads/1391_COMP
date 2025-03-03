// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ReefPoses.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ArmConstants.*;

import frc.robot.commands.TransferCommand;
import frc.robot.commands.AutoAlignCommand;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousCommand extends Command {
  /** Creates a new AutonomousCommand. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric driveRR;
  Elevator elevator;
  Arm arm;
  Hopper hopper;
  Knuckle knuckle;
  
  public AutonomousCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRR, Elevator elevator, Arm arm, Hopper hopper, Knuckle knuckle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.driveRR = driveRR;
    this.elevator = elevator;
    this.arm = arm;
    this.hopper = hopper;
    this.knuckle = knuckle;
  }

  public Command branches3_4_5_6() { 
    return Commands.sequence(
      AutoBuilder.pathfindToPose(kRED2_3, K_CONSTRAINTS),
       new ParallelCommandGroup(
        new AutoAlignCommand(drivetrain, driveRR, true),
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),
       new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
       /*new ParallelCommandGroup( // Travel height
        new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
        new InstantCommand(() -> arm.setSetpoint(0.21))
        ),
      AutoBuilder.pathfindToPose(kREDSOURCERIGHT, K_CONSTRAINTS),
       new TransferCommand(elevator, arm, knuckle, hopper),
      AutoBuilder.pathfindToPose(kRED4_5, K_CONSTRAINTS),
       new ParallelCommandGroup(
        new AutoAlignCommand(drivetrain, driveRR, false),
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),
       new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
       new ParallelCommandGroup( // Travel height
        new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
        new InstantCommand(() -> arm.setSetpoint(0.21))
        ),
      AutoBuilder.pathfindToPose(kREDSOURCERIGHT, K_CONSTRAINTS),
       new TransferCommand(elevator, arm, knuckle, hopper),
      AutoBuilder.pathfindToPose(kRED4_5, K_CONSTRAINTS),
       new ParallelCommandGroup(
        new AutoAlignCommand(drivetrain, driveRR, true),
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),
       new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
       new ParallelCommandGroup( // Travel height
        new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
        new InstantCommand(() -> arm.setSetpoint(0.21))
       ),
      AutoBuilder.pathfindToPose(kREDSOURCERIGHT, K_CONSTRAINTS),
       new TransferCommand(elevator, arm, knuckle, hopper),
      AutoBuilder.pathfindToPose(kRED6_7, K_CONSTRAINTS),
       new ParallelCommandGroup(
        new AutoAlignCommand(drivetrain, driveRR, false),
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),
       new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())*/
    );
  }
  public Command branches10_9_8_7() {
    return Commands.sequence(
      AutoBuilder.pathfindToPose(kRED10_11, K_CONSTRAINTS),
      new ParallelCommandGroup(
        new AutoAlignCommand(drivetrain, driveRR, true),
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),
      AutoBuilder.pathfindToPose(kREDSOURCELEFT, K_CONSTRAINTS)
    );
  }
}
