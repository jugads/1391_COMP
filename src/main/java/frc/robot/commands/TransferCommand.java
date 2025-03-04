
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
public class TransferCommand extends SequentialCommandGroup {
  /** Creates a new TransferCommand. */
  Elevator elevator;
  Arm arm;
  Knuckle knuckle;
  Hopper hopper;
  public TransferCommand(Elevator elevator, Arm arm, Knuckle knuckle, Hopper hopper) {
    this.elevator = elevator;
    this.arm = arm;
    this.hopper = hopper;
    this.knuckle = knuckle;
    
    addCommands(
      // new InstantCommand(() -> knuckle.score(), knuckle),
      new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
      new WaitUntilCommand(() -> (Math.abs(elevator.getSetpoint()-elevator.getElevatorPosition()) < 0.03)),
      new InstantCommand(() -> arm.setSetpoint(kArmTran)).until(() -> arm.getEncoderPosition() < (arm.getSetpoint()+0.005)),/*,*/
      new WaitCommand(0.25),
      new ParallelCommandGroup(
          new RunCommand(() -> hopper.runBoth(0.4, 1.), hopper),
          new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle)
      ).until(() -> knuckle.hasCoral()),
      new ParallelCommandGroup(
      new InstantCommand(() -> arm.setSetpoint(0.15)),
      new InstantCommand(() -> elevator.setSetpoint(kElevTran+0.03)),
      new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle)
      )
    );
  }
}
