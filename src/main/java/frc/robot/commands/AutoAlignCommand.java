// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.kMaxSpeed;

import java.time.chrono.ThaiBuddhistDate;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

/**
 * Autonomous alignment command using vision feedback:
 * - Uses PID control for X and Y positioning
 * - Aligns to either left or right vision target
 * - Uses Limelight TX/TY values for position feedback
 */
public class AutoAlignCommand extends Command {
  /** Creates a new AutoAlignCommand */
  // X control: Higher P gain for distance, small D for stability
  PIDController distanceController = new PIDController(0.035, 0., 0.0013);
  PIDController distanceControllerRight = new PIDController(0.045, 0., 0.0013);
  // Y control: Lower gains for lateral movement
  PIDController lateralController = new PIDController(0.0065, 0., 0.0003);
  PIDController thetaController = new PIDController(0.3, 0., 0.0);
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  SwerveRequest.ApplyRobotSpeeds driveChassisSpeeds = new ApplyRobotSpeeds();
  // Determines which camera/target to use for alignment
  boolean aligningLeft;
  boolean aligningL4;
  Elevator elevator;
  public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric drive, boolean aligningLeft, boolean aligningL4, Elevator elevator) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    this.aligningLeft = aligningLeft;
    this.aligningL4 = aligningL4;
    this.elevator = elevator;
    // Register drivetrain requirement for command scheduling
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    thetaController.enableContinuousInput(-180, 180);
    // thetaController.setSetpoint(0);
    thetaController.setTolerance(2);
    // Allow 0.3m tolerance in both axes
    distanceController.setTolerance(0.5);
    lateralController.setTolerance(0.3);    
  }

  @Override
  public void execute() {
    if (!aligningL4) {
    aligningL4 = this.elevator.getElevatorPosition() > 0.9;
    }
    // Target setpoints for alignment:
    //Decrease to move closer, increase to move further
    if (getRot() < 30 && getRot() > -30) {
      thetaController.setSetpoint(0.);
    }
    else if (getRot() < 90 && getRot() > 30) {
      thetaController.setSetpoint(60.);
    }
    else if (getRot() < 150 && getRot() > 90) {
      thetaController.setSetpoint(120.);
    }
    else if (Math.abs(getRot()) > 150) {
      thetaController.setSetpoint(180.);
    }
    else if (getRot() < -90 && getRot() > -150) {
      thetaController.setSetpoint(-120.);
    }
    else {
      thetaController.setSetpoint(-60);
    }
    distanceController.setSetpoint((aligningL4 ? 3.0 : 2.75));
    distanceControllerRight.setSetpoint((aligningL4 ? 1.5 : 1.));
    lateralController.setSetpoint(aligningL4 ? 1.5 : -1.5);
    SmartDashboard.putNumber("Output", thetaController.calculate(getRot()));
    SmartDashboard.putNumber("s", thetaController.getSetpoint());
    SmartDashboard.putNumber("Rot", getRot());
    // SmartDashboard.putNumber("Rot", getRot());
    // Calculate velocities using PID and vision feedback
    // Negative maxSpeed multiplier inverts direction as needed
    drivetrain.setControl(drive
    .withVelocityX(-kMaxSpeed* (aligningLeft ?
    distanceController.calculate(drivetrain.getTYRight()) :
    distanceControllerRight.calculate(drivetrain.getTYLeft()))
    )
    .withVelocityY(-kMaxSpeed * lateralController.calculate(aligningLeft ? drivetrain.getTXRight() : drivetrain.getTXLeft()))
    .withRotationalRate(thetaController.calculate(getRot()))
    );
    // Set alignment state for status tracking
    drivetrain.setAligning(true);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all movement when command ends
    drivetrain.setControl(drive
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0.)
    );
    // Clear alignment state
    drivetrain.setAligning(false);
  }

  @Override
  public boolean isFinished() {
    // Command completes when either:
    // - X position is within tolerance
    // - Target visibility is lost for the selected camera
    return (aligningLeft ? !drivetrain.getTVRight() : !drivetrain.getTVLeft()) || DriverStation.isAutonomous() ? (Math.abs(distanceController.getSetpoint() - getMeasurement()) < 3.) : (Math.abs(distanceController.getSetpoint() - getMeasurement()) < 1.) || distanceController.atSetpoint();
  }
  public double getMeasurement() {
    return aligningLeft ? drivetrain.getTYRight() : drivetrain.getTYLeft();
  }
  public double getRot() {
    return drivetrain.getPose().getRotation().getDegrees();
  }
}