package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Hopper;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ArmConstants.*;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/** 
 * TransferCommand coordinates multiple subsystems to transfer game pieces.
 * It extends SequentialCommandGroup to run a series of commands in order.
 */
public class TransferCommand extends SequentialCommandGroup {
  // Subsystem references needed for the transfer operation
  Elevator elevator;
  Arm arm;
  Knuckle knuckle;
  Hopper hopper;

  /** Creates a new TransferCommand. */
  public TransferCommand(Elevator elevator, Arm arm, Knuckle knuckle, Hopper hopper) {
    // Store subsystem references
    this.elevator = elevator;
    this.arm = arm;
    this.hopper = hopper;
    this.knuckle = knuckle;
    
    addCommands(
      // Move elevator to transfer position
      new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
      
      // Wait until elevator reaches position (within 0.03 units)
      new WaitUntilCommand(() -> (Math.abs(elevator.getSetpoint()-elevator.getElevatorPosition()) < 0.05)),
      
      // Move arm to transfer position and wait until it's close enough
      new InstantCommand(() -> arm.setSetpoint(kArmTran)),
      new ParallelRaceGroup(
      new WaitUntilCommand(() -> arm.getEncoderPosition() < -0.205),
      new ConditionalCommand(
        new RunCommand(() -> hopper.runBeltMotor(0.3)), Commands.none(), () -> DriverStation.isTeleop()
      )
      ),
      
      // Brief pause to ensure stability
      new WaitCommand(0.1),
      
      // Run hopper and knuckle simultaneously until coral is detected
      new ParallelCommandGroup(
          new RunCommand(() -> hopper.runBoth(0.4, 1.), hopper),
          new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle)
      ).until(() -> knuckle.hasCoral()),
      new ConditionalCommand(
      new WaitUntilCommand(() -> arm.getShouldSwing()),
      Commands.none(),
      () -> DriverStation.isTeleop()
      ),
      new InstantCommand(() -> elevator.setSetpoint(kElevTran + 0.09)).until(() -> elevator.getElevatorPosition() > (kElevTran+0.03)),
      // Final positioning after coral is acquired
      new InstantCommand(() -> arm.setSetpoint(kArmL3)),
      new WaitUntilCommand(() -> arm.getEncoderPosition() > 0.1));
  }
}
