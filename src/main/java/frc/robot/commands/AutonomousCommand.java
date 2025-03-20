// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ReefPoses.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.OperatorConstants.kL4;
import static frc.robot.Constants.ArmConstants.*;

import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonomousCommand extends Command {
  /** Creates a new AutonomousCommand. */
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric driveRR;
  Elevator elevator;
  Arm arm;
  Hopper hopper;
  Knuckle knuckle;
  AlgaeScorer algae;
  Pose2d[] poseArrays;
  Timer timer = new Timer();

  public AutonomousCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRR, Elevator elevator,
      Arm arm, Hopper hopper, Knuckle knuckle, AlgaeScorer algae) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.driveRR = driveRR;
    this.elevator = elevator;
    this.arm = arm;
    this.hopper = hopper;
    this.algae = algae;
    this.knuckle = knuckle;
  }

  public boolean isRed() {
    return DriverStation.getAlliance().get() == Alliance.Red;
  }

  public Command branches3_4_5_6() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED2_3 : kBLUE2_3,
        isRed() ? kREDSOURCERIGHT_bargeWall : kBLUESOURCERIGHT_bargeWall,
        isRed() ? kRED4_5 : kBLUE4_5,
        isRed() ? kRED4_5 : kBLUE4_5
    };
    return 
    new ParallelCommandGroup(
    Commands.sequence(
        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.))),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new InstantCommand(() -> elevator.setSetpoint(kElevL2)),
            new InstantCommand(() -> arm.setSetpoint(kArmL2))
        ),

        // new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),

        new AutoAlignCommand(drivetrain, driveRR, true, true, elevator),

        new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
        // Piece 2
        new ParallelCommandGroup( // Transfer height
            new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
            new InstantCommand(() -> arm.setSetpoint(0.25))),
        new ParallelRaceGroup(
            AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
            new TransferCommand(elevator, arm, knuckle, hopper)),
        new WaitCommand(0.2),
        new InstantCommand(() -> timer.restart()),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 4.5),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                    new InstantCommand(() -> arm.setSetpoint(kArmL4))).until(() -> elevator.getElevatorPosition() > 0.85),
                    new AutoAlignCommand(drivetrain, driveRR, false, true, elevator)),
                    
                new RunCommand(() -> knuckle.score()).until(() -> !knuckle.hasCoral()),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper))),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral()),

        new InstantCommand(() -> timer.restart()),
        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 4.5),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),
        drivetrain.stopPathFollowState(),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                    new InstantCommand(() -> arm.setSetpoint(kArmL4))).until(() -> elevator.getElevatorPosition() > 0.85),
                    new AutoAlignCommand(drivetrain, driveRR, true, true, elevator)),
                new RunCommand(() -> knuckle.score()).until(() -> !knuckle.hasCoral()),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.2),
                drivetrain.stopPathFollowState()),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral())
            
            ),
            new ElevatorCommand(elevator, algae),
            new ArmCommand(arm, elevator)

    /*
     * new ParallelCommandGroup(
     * new AutoAlignCommand(drivetrain, driveRR, false),
     * new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
     * new InstantCommand(() -> arm.setSetpoint(kArmL4))
     * ),
     * new RunCommand(() -> knuckle.score(), knuckle).until(() ->
     * !knuckle.hasCoral()),
     * new ParallelCommandGroup( // Travel height
     * new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
     * new InstantCommand(() -> arm.setSetpoint(0.21))
     * ),
     * AutoBuilder.pathfindToPose(kREDSOURCERIGHT, K_CONSTRAINTS_Fastest),
     * new TransferCommand(elevator, arm, knuckle, hopper),
     * AutoBuilder.pathfindToPose(kRED4_5, K_CONSTRAINTS_Fastest),
     * new ParallelCommandGroup(
     * new AutoAlignCommand(drivetrain, driveRR, true),
     * new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
     * new InstantCommand(() -> arm.setSetpoint(kArmL4))
     * ),
     * new RunCommand(() -> knuckle.score(), knuckle).until(() ->
     * !knuckle.hasCoral()),
     * new ParallelCommandGroup( // Travel height
     * new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
     * new InstantCommand(() -> arm.setSetpoint(0.21))
     * ),
     * AutoBuilder.pathfindToPose(kREDSOURCERIGHT, K_CONSTRAINTS_Fastest),
     * new TransferCommand(elevator, arm, knuckle, hopper),
     * AutoBuilder.pathfindToPose(kRED6_7, K_CONSTRAINTS_Fastest),
     * new ParallelCommandGroup(
     * new AutoAlignCommand(drivetrain, driveRR, false),
     * new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
     * new InstantCommand(() -> arm.setSetpoint(kArmL4))
     * ),
     * new RunCommand(() -> knuckle.score(), knuckle).until(() ->
     * !knuckle.hasCoral())
     */
    );
  }

  public Command branches10_9_8_7() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED10_11 : kBLUE10_11,
        isRed() ? kREDSOURCELEFT_bargeWall : kBLUESOURCELEFT_bargeWall,
        isRed() ? kRED8_9 : kBLUE8_9,
        isRed() ? kRED8_9 : kRED8_9
    };
    return new ParallelCommandGroup(
        Commands.sequence(
            new WaitCommand(0.5),
            new ParallelCommandGroup(
                new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.))),
            new WaitCommand(0.1),
            new ParallelCommandGroup(
                AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
                new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
            ),
    
            // new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
    
            new AutoAlignCommand(drivetrain, driveRR, false, true, elevator),
    
            new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
            // Piece 2
            new ParallelCommandGroup( // Transfer height
                new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                new InstantCommand(() -> arm.setSetpoint(0.25))),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            new WaitCommand(0.2),
            new InstantCommand(() -> timer.restart()),
            new ParallelCommandGroup(
                new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 4.5),
                AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),
            new ConditionalCommand(
                Commands.sequence(
                    new SequentialCommandGroup(
                    new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                    new InstantCommand(() -> arm.setSetpoint(kArmL4))).until(() -> elevator.getElevatorPosition() > 0.85),
                    new AutoAlignCommand(drivetrain, driveRR, true, true, elevator)
                    ),
                    new RunCommand(() -> knuckle.score()).until(() -> !knuckle.hasCoral()),
    
                    // Piece 3
                    new ParallelCommandGroup( // Transfer height
                        new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                        new InstantCommand(() -> arm.setSetpoint(0.25))),
                    new ParallelRaceGroup(
                        AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                        new TransferCommand(elevator, arm, knuckle, hopper))
                        ,new WaitCommand(0.2)),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)),
                () -> knuckle.hasCoral()),
    
            new InstantCommand(() -> timer.restart()),
            drivetrain.setFollowingPath(),
            new ParallelCommandGroup(
                new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 4.5),
                AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),
            drivetrain.stopPathFollowState(),
            new ConditionalCommand(
                Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ).until(() -> elevator.getElevatorPosition() > 0.85),
                    new AutoAlignCommand(drivetrain, driveRR, false, true, elevator)
                ),
                    new RunCommand(() -> knuckle.score()).until(() -> !knuckle.hasCoral()),
    
                    // Piece 3
                    new ParallelCommandGroup( // Transfer height
                        new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                        new InstantCommand(() -> arm.setSetpoint(0.25))),
                    drivetrain.setFollowingPath(),
                    new ParallelRaceGroup(
                        AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                        new TransferCommand(elevator, arm, knuckle, hopper)),
                    drivetrain.stopPathFollowState()),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)),
                () -> knuckle.hasCoral())
                
                ),
                new ElevatorCommand(elevator, algae),
                new ArmCommand(arm, elevator));
}

  public Command dodge() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED0_1 : kBLUE0_1,
        isRed() ? new Pose2d(10.878, 7.48, Rotation2d.fromDegrees(180))
            : new Pose2d(6.39, 0.58, Rotation2d.fromDegrees(0)),
        isRed() ? kREDSOURCERIGHT_bargeWall : kBLUESOURCERIGHT_bargeWall
    };
    return Commands.sequence(
        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22)),
            new InstantCommand(() -> elevator.setSetpoint(0.))),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new RunCommand(() -> knuckle.setKnuckleMotorHigh()).until(() -> knuckle.hasCoral())),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))),
        new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
        new AutoAlignCommand(drivetrain, driveRR, true, true, elevator),
        new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
        new ParallelCommandGroup( // Travel height
            new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
            new InstantCommand(() -> arm.setSetpoint(0.21))),
        AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
        AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest));
  }

  public Command center0() {
    return Commands.sequence(
        new WaitCommand(1),
        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22)),
            new InstantCommand(() -> elevator.setSetpoint(0.))),
        new ParallelCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4)),
            new AutoAlignCommand(drivetrain, driveRR, false, true, elevator)),
        new RunCommand(() -> knuckle.score()).until(() -> !knuckle.hasCoral()));
  }

  public Command driveStraight() {
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
      new InstantCommand(() -> timer.restart()),
      drivetrain.applyRequest(() -> driveRR.withVelocityX(0.75)).until(() -> timer.get() > 0.5)
    );
  }
}
