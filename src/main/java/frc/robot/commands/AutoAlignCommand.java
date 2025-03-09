// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.kMaxSpeed;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
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
  PIDController distanceController = new PIDController(0.0275, 0., 0.0013);
  // Y control: Lower gains for lateral movement
  PIDController lateralController = new PIDController(0.0065, 0., 0.0003);
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
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
    
    // Allow 0.3m tolerance in both axes
    distanceController.setTolerance(0.3);
    lateralController.setTolerance(0.3);
  }

  @Override
  public void execute() {
    aligningL4 = this.elevator.getElevatorPosition() > 0.9;
    // Target setpoints for alignment:
    distanceController.setSetpoint(aligningLeft ? (aligningL4 ? 2.5 : 0.6) : (aligningL4 ? 1.75 : 0.));
    lateralController.setSetpoint(aligningL4 ? -1 : -1.);
    SmartDashboard.putBoolean("getName()", aligningL4);
    SmartDashboard.putNumber("DSetpoint", distanceController.getSetpoint());
    // Calculate velocities using PID and vision feedback
    // Negative maxSpeed multiplier inverts direction as needed
    drivetrain.setControl(drive
    .withVelocityX(-kMaxSpeed*distanceController.calculate(aligningLeft ? drivetrain.getTYRight() : drivetrain.getTYLeft()))
    .withVelocityY(-kMaxSpeed * lateralController.calculate(aligningLeft ? drivetrain.getTXRight() : drivetrain.getTXLeft()))
    .withRotationalRate(0.)
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
    return (aligningLeft ? !drivetrain.getTVRight() : !drivetrain.getTVLeft()) || DriverStation.isAutonomous() ? (Math.abs(distanceController.getSetpoint() - getMeasurement()) < 3.) : distanceController.atSetpoint();
  }
  public double getMeasurement() {
    return aligningLeft ? drivetrain.getTYRight() : drivetrain.getTYLeft();
  }
}