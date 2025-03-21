// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomatedAlgaeCommand extends Command {
  /** Creates a new AutomatedAlgaeCommand. */
  AlgaeScorer algae;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric driveRR;
  Elevator elevator;
  Arm arm;
  Timer timer = new Timer();
  boolean isAtLeft;
  double[] algaeL2 = new double[]{17, 19, 21, 6, 8, 10};
  double[] algaeL3 = new double[]{18, 20, 22, 7, 9, 11};
  public AutomatedAlgaeCommand(AlgaeScorer algae, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric driveRR, Elevator elevator, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.algae = algae;
    this.drivetrain = drivetrain;
    this.driveRR = driveRR;
    addRequirements(algae, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double tagID = 0;
    if (drivetrain.getTVLeft() && !drivetrain.getTVRight()) {
      isAtLeft = false;
      tagID = drivetrain.getTIDLeft();
    }
    else if (drivetrain.getTVRight() && !drivetrain.getTVLeft()) {
      isAtLeft = true;
      tagID = drivetrain.getTIDRight();
    }
    else {
      if (Math.abs(drivetrain.getTXLeft()) < Math.abs(drivetrain.getTXRight())) {
        isAtLeft = true;
      }
      else if (Math.abs(drivetrain.getTXRight()) < Math.abs(drivetrain.getTXLeft())) {
        isAtLeft = true;
      }
      else {
        end(true);
      }
    }
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 0.4) {
      drivetrain.setControl(driveRR.withVelocityY(isAtLeft ? -0.75 : 0.75));
    }
    else if (!algae.hasAlgae()){
      drivetrain.setControl(driveRR.withVelocityX(0.75).withVelocityY(0.));
      algae.runAlgaeScorer(0.8);
    }
    else if (algae.hasAlgae() && timer.get() > 0.5) {
      drivetrain.setControl(driveRR.withVelocityX(-0.75).withVelocityY(0.));
    }
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
