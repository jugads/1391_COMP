// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ReefPoses.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.OperatorConstants.kL4;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE10_11L4;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE2_3L4;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE4_5L4;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE8_9L4;
import static frc.robot.Constants.AlignmentPoses.kAliRED10_11L4;
import static frc.robot.Constants.AlignmentPoses.kAliRED2_3L4;
import static frc.robot.Constants.AlignmentPoses.kAliRED4_5L4;
import static frc.robot.Constants.AlignmentPoses.kAliRED8_9L4;
import static frc.robot.Constants.ArmConstants.*;

import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathConstraints;

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
        isRed() ? kRED2_3 : kBLUE2_3
    };
    return new ParallelCommandGroup(
    Commands.sequence(

        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22))),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.25),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),

        new WaitCommand(0.5),
        new InstantCommand(() -> timer.restart()),
        AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE2_3L4[0] : kAliRED2_3L4[0], K_CONSTRAINTS_Barging).onlyWhile(() -> timer.get() < 4),
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
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),

                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),

                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE4_5L4[0] : kAliRED4_5L4[0], K_CONSTRAINTS_Barging).onlyWhile(() -> timer.get() < 4),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                )
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        new InstantCommand(() -> timer.restart()),

        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)
        ),
        drivetrain.stopPathFollowState(),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),
                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
                    new WaitCommand(0.2),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE4_5L4[1] : kAliRED4_5L4[1], K_CONSTRAINTS_Barging),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        //4th piece
        new InstantCommand(() -> timer.restart()),
        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[3], K_CONSTRAINTS_Fastest)),
        drivetrain.stopPathFollowState(),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ).until(() -> elevator.getElevatorPosition() > 0.85),
                    new WaitCommand(0.75),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE2_3L4[1] : kAliRED2_3L4[1], K_CONSTRAINTS_Barging),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // 4.5 piece
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral())
            
            ),
            new ElevatorCommand(elevator, algae),
            new ArmCommand(arm, elevator)
    );
  }
  public Command branches3_4_5_6_ALLLEFT() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED2_3 : kBLUE2_3,
        isRed() ? kREDSOURCERIGHT_bargeWall : kBLUESOURCERIGHT_bargeWall,
        isRed() ? kRED4_5 : kBLUE4_5,
        isRed() ? kRED2_3 : kBLUE2_3
    };
    return new ParallelCommandGroup(
    Commands.sequence(

        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22))),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.25),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),

        new WaitCommand(0.5),
        new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
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
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),

                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),

                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                )
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        new InstantCommand(() -> timer.restart()),

        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(isRed() ? kRED6_7 : kBLUE6_7, K_CONSTRAINTS_Fastest)
        ),
        drivetrain.stopPathFollowState(),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),
                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        //4th piece
        new InstantCommand(() -> timer.restart()),
        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[3], K_CONSTRAINTS_Fastest)),
        drivetrain.stopPathFollowState(),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ).until(() -> elevator.getElevatorPosition() > 0.85),
                    new WaitCommand(0.75),
                    new AutoAlignCommand(drivetrain, driveRR, false, true, elevator),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // 4.5 piece
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral())
            
            ),
            new ElevatorCommand(elevator, algae),
            new ArmCommand(arm, elevator)
    );
  }

  public Command branches10_9_8_7ALLLEFT() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED10_11 : kBLUE10_11,
        isRed() ? kREDSOURCELEFT_bargeWall : kBLUESOURCELEFT_bargeWall,
        isRed() ? kRED8_9 : kBLUE8_9,
        isRed() ? kRED10_11 : kBLUE10_11
    };
    return new ParallelCommandGroup(
    Commands.sequence(

        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22))),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.25),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),

        new WaitCommand(0.25),
        new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
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
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),

                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),

                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                )
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        new InstantCommand(() -> timer.restart()),

        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)
        ),
        drivetrain.stopPathFollowState(),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),
                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
                    new WaitCommand(0.2),
                    new AutoAlignCommand(drivetrain, driveRR, false, true, elevator),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        //4th piece
        new InstantCommand(() -> timer.restart()),
        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[3], K_CONSTRAINTS_Fastest)),
        drivetrain.stopPathFollowState(),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ).until(() -> elevator.getElevatorPosition() > 0.85),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // 4.5 piece
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral())
            
            ),
            new ElevatorCommand(elevator, algae),
            new ArmCommand(arm, elevator)
    );
}

public Command branches10_9_8_7() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED10_11 : kBLUE10_11,
        isRed() ? kREDSOURCELEFT_bargeWall : kBLUESOURCELEFT_bargeWall,
        isRed() ? kRED8_9 : kBLUE8_9,
        isRed() ? kRED10_11 : kBLUE10_11
    };
    return new ParallelCommandGroup(
    Commands.sequence(

        new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
            new InstantCommand(() -> arm.setSetpoint(0.22))),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.25),
        new ParallelCommandGroup(
            AutoBuilder.pathfindToPose(poseArrays[0], K_CONSTRAINTS_Fastest),
            new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))
        ),

        new WaitCommand(0.25),
        new InstantCommand(() -> timer.restart()),
        AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE10_11L4[1] : kAliRED10_11L4[1], K_CONSTRAINTS_Barging),
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
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),

                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),

                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE8_9L4[0] : kAliRED8_9L4[0], K_CONSTRAINTS_Barging),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                )
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        new InstantCommand(() -> timer.restart()),

        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest)
        ),
        drivetrain.stopPathFollowState(),

        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ),
                    new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.9),
                    new WaitCommand(0.2),
                    new InstantCommand(() -> timer.restart()),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE8_9L4[1] : kAliRED8_9L4[1], K_CONSTRAINTS_Barging),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),

                // Piece 3
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()
            ),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)
            ),
            () -> knuckle.hasCoral()
        ),
        //4th piece
        new InstantCommand(() -> timer.restart()),
        drivetrain.setFollowingPath(),
        new ParallelCommandGroup(
            new TransferCommand(elevator, arm, knuckle, hopper).onlyWhile(() -> timer.get() < 3.5 && !knuckle.hasCoral()).andThen(new InstantCommand(() -> arm.setSetpoint(0.25))),
            AutoBuilder.pathfindToPose(poseArrays[3], K_CONSTRAINTS_Fastest)),
        drivetrain.stopPathFollowState(),
        new ConditionalCommand(
            Commands.sequence(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                        new InstantCommand(() -> arm.setSetpoint(kArmL4))
                    ).until(() -> elevator.getElevatorPosition() > 0.85),
                    new WaitCommand(0.75),
                    new InstantCommand(() -> timer.restart()),
                    AutoBuilder.pathfindToPose(!isRed() ? kAliBLUE10_11L4[0] : kAliRED10_11L4[0], K_CONSTRAINTS_Barging),
                    // new InstantCommand(() -> arm.setSetpoint(0.14)),
                    // new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.02),
                    new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral())
                ),
                // 4.5 piece
                new ParallelCommandGroup( // Transfer height
                    new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))),
                drivetrain.setFollowingPath(),
                new ParallelRaceGroup(
                    AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                    new TransferCommand(elevator, arm, knuckle, hopper)
                ),
                new WaitCommand(0.1),
                drivetrain.stopPathFollowState()),
            new ParallelRaceGroup(
                AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
                new TransferCommand(elevator, arm, knuckle, hopper)),
            () -> knuckle.hasCoral())
            
            ),
            new ElevatorCommand(elevator, algae),
            new ArmCommand(arm, elevator)
    );
}

  public Command dodge() {
    poseArrays = new Pose2d[] {
        isRed() ? kRED0_1 : kBLUE0_1,
        isRed() ? new Pose2d(10.878, 7.48, Rotation2d.fromDegrees(180))
            : new Pose2d(6.39, 0.58, Rotation2d.fromDegrees(0)),
        isRed() ? kREDSOURCERIGHT_bargeWall : kBLUESOURCERIGHT_bargeWall
    };
    return Commands.sequence(
        new WaitCommand(1.),
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
        new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
        new InstantCommand(() -> arm.setSetpoint(0.14)),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < arm.getSetpoint()+0.03),
        new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
        new ParallelCommandGroup( // Travel height
            new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
            new InstantCommand(() -> arm.setSetpoint(0.21))),
        AutoBuilder.pathfindToPose(poseArrays[1], K_CONSTRAINTS_Fastest),
        AutoBuilder.pathfindToPose(poseArrays[2], K_CONSTRAINTS_Fastest));
  }

  public Command center0AlgaeRemoval() {
    Timer dstime = new Timer();
    return Commands.sequence(
        //Gyro and move arm
        new InstantCommand(() -> dstime.start()),
        new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
        new InstantCommand(() -> arm.setSetpoint(0.22)),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.25),

        //l4 setpoints
        new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        new InstantCommand(() -> arm.setSetpoint(kArmL4)),
        new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.85),
        new WaitCommand(0.75),
        //Both AutoAligns
        new InstantCommand(() -> timer.restart()),
new AutoAlignCommand(drivetrain, driveRR, true, true, elevator).onlyWhile(() -> timer.get() < 4),
        new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()),
        new InstantCommand(() -> knuckle.stopMotor()),
        new AutomatedAlgaeCommand(algae, drivetrain, driveRR, elevator, arm).until(() -> drivetrain.getTVLeft() && algae.hasAlgae()),
        //Reset Odometry Based on LL Left
        new InstantCommand(() -> drivetrain.resetPose(
            new Pose2d(
                drivetrain.getLeftLLPose().getX(),
                drivetrain.getLeftLLPose().getY(),
                drivetrain.getPigeon2().getRotation2d())
            )
        ),
        //Driving and elevator/arm
        new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(isRed() ? kREDBarge : kBLUEBarge, K_CONSTRAINTS_Barging),
        new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.9985)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.95),
            new InstantCommand(() -> arm.setSetpoint(0.35)),
            new WaitUntilCommand(() -> arm.getEncoderPosition() > 0.34)
        )
        ),

        //Scoring Algae
        new ParallelCommandGroup(
            new RunCommand(() -> algae.score()).until(() -> !algae.hasAlgae()),
            new InstantCommand(() -> arm.setSetpoint(0.25))
        ),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.3),

        //L3 Setpoint
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.setSetpoint(0.58)),
          new InstantCommand(() -> arm.setSetpoint(0.19))
        ),
        //Driving Back To Reef & collecting algae
        AutoBuilder.pathfindToPose(isRed() ? kRED10_11_ALGAE : kBLUE10_11_ALGAE, K_CONSTRAINTS_Fastest),
        new AutoAlignCommand(drivetrain, driveRR, false, false, elevator),
        new InstantCommand(() -> algae.resetGripper()),
        new AutomatedAlgaeCommand(algae, drivetrain, driveRR, elevator, arm).until(() -> drivetrain.getTVLeft() && algae.hasAlgae()),
        new InstantCommand(() -> drivetrain.resetPose(
            new Pose2d(
                drivetrain.getLeftLLPose().getX(),
                drivetrain.getLeftLLPose().getY(),
                drivetrain.getPigeon2().getRotation2d())
            )
        ),
        new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(isRed() ? kREDBarge : kBLUEBarge, K_CONSTRAINTS_Barging),
        new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.9985)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.95),
            new InstantCommand(() -> arm.setSetpoint(0.35)),
            new WaitUntilCommand(() -> arm.getEncoderPosition() > 0.34)
        )
        ),
    //6.8, 10.7
        //Scoring Algae
        new ParallelCommandGroup(
            new RunCommand(() -> algae.score()).until(() -> !algae.hasAlgae()),
            new InstantCommand(() -> arm.setSetpoint(0.25))
        ),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.3),
        new ParallelCommandGroup(
          new InstantCommand(() -> elevator.setSetpoint(0.58)),
          new InstantCommand(() -> arm.setSetpoint(0.19))
        ),
        //Driving Back To Reef & collecting algae
        AutoBuilder.pathfindToPose(isRed() ? kRED2_3 : kBLUE2_3, K_CONSTRAINTS_Fastest),
        new AutoAlignCommand(drivetrain, driveRR, false, false, elevator),
        new InstantCommand(() -> algae.resetGripper()),
        new AutomatedAlgaeCommand(algae, drivetrain, driveRR, elevator, arm).until(() -> drivetrain.getTVLeft() && algae.hasAlgae()),
        new InstantCommand(() -> drivetrain.resetPose(
            new Pose2d(
                drivetrain.getLeftLLPose().getX(),
                drivetrain.getLeftLLPose().getY(),
                drivetrain.getPigeon2().getRotation2d())
            )
        ),
        new ParallelCommandGroup(
        AutoBuilder.pathfindToPose(isRed() ? kREDBarge : kBLUEBarge, K_CONSTRAINTS_Barging),
        new SequentialCommandGroup(
            new InstantCommand(() -> elevator.setSetpoint(0.9985)),
            new WaitUntilCommand(() -> elevator.getElevatorPosition() > 0.95),
            new InstantCommand(() -> arm.setSetpoint(0.35)),
            new WaitUntilCommand(() -> arm.getEncoderPosition() > 0.34)
        )
        ),
    //6.8, 10.7
        //Scoring Algae
        new ParallelCommandGroup(
            new RunCommand(() -> algae.score()).until(() -> !algae.hasAlgae()),
            new InstantCommand(() -> arm.setSetpoint(0.25))
        ),
        new WaitUntilCommand(() -> arm.getEncoderPosition() < 0.3),
        AutoBuilder.pathfindToPose(kBLUE10_11, K_CONSTRAINTS_Fastest)
    ).until(() -> dstime.get() > 13.5).andThen(
        new ConditionalCommand(
            AutoBuilder.pathfindToPose(kBLUE10_11, K_CONSTRAINTS_Fastest),
            Commands.none(),
            () -> (drivetrain.getPose().getX() < 10.7 && drivetrain.getPose().getX() > 6.8)
        )
    );
  }

  public Command driveStraight() {
    return Commands.sequence(
      new InstantCommand(() -> drivetrain.getPigeon2().setYaw(isRed() ? 0. : 180.)),
      new InstantCommand(() -> timer.restart()),
      drivetrain.applyRequest(() -> driveRR.withVelocityX(0.75)).until(() -> timer.get() > 0.5)
    );
  }
}
