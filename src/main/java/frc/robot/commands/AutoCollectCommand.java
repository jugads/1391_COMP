// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCollectCommand extends Command {
  CommandSwerveDrivetrain drivetrain;
  PIDController thetaPid = new PIDController(0.01678, 0, 0);
  PIDController distancePid = new PIDController(0.0254, 0, 0);
  SwerveRequest.RobotCentric driveRR;
  /** Creates a new AutoCollectCommand. */
  public AutoCollectCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRR) {
    this.drivetrain = drivetrain;
    this.driveRR = driveRR;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancePid.setSetpoint(-2.);
    thetaPid.setSetpoint(0.);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(() -> driveRR.
    withVelocityX(
      distancePid.calculate(drivetrain.getTYIntakeLL())
    )
    .withVelocityY(0)
    .withRotationalRate(
      thetaPid.calculate(drivetrain.getTXIntakeLL())
    )
    );
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
