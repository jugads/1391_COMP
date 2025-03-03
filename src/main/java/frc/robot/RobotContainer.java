// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.KnuckleCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Hopper;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ReefPoses.*;
import static frc.robot.Constants.OperatorConstants.*;
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 3* Math.PI;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
            private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Knuckle knuckle = new Knuckle();
    public final Elevator elevator = new Elevator();
    public final AlgaeScorer algaeScorer = new AlgaeScorer();
    public final AutonomousCommand autos = new AutonomousCommand();
    public final Arm arm = new Arm();
    public final Leds leds = new Leds(new AddressableLED(9), new AddressableLEDBuffer(138), arm, knuckle, algaeScorer);
    public final Hopper hopper = new Hopper();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {drivetrain.getPigeon2().setYaw(0);}
   else if (DriverStation.getAlliance().get() == Alliance.Red) {drivetrain.getPigeon2().setYaw(180);}
        autoChooser.addOption("3-4-5-6", autos.branches3456());
        configureBindings();
    }

    private void configureBindings() {  
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        /*DEFAULT COMMANDS
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        */
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        elevator.setDefaultCommand(new ElevatorCommand(elevator));
        knuckle.setDefaultCommand(new KnuckleCommand(knuckle));
        algaeScorer.setDefaultCommand(new RunCommand(() -> algaeScorer.runAlgaeScorer(algaeScorer.hasAlgae() ? 0.2 : 0.), algaeScorer));
        arm.setDefaultCommand(new ArmCommand(arm));
        hopper.setDefaultCommand(new HopperCommand(hopper));
        // reset the field-centric heading on left bumper press

        /*DRIVER CONTROLLER
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        */
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.a().whileTrue(new RunCommand(() -> knuckle.setKnuckleMotorHigh()));
        joystick.b().whileTrue(new RunCommand(()-> arm.runMotor(0.1), arm));
        joystick.x().whileTrue(new RunCommand(()-> arm.runMotor(-0.1), arm));
        joystick.leftBumper().whileTrue(new RunCommand(() -> knuckle.score(), knuckle));
        // joystick.rightBumper().whileTrue(
        //     new RunCommand(() -> hopper.runBoth(0.2, 1.), hopper)
        // );
        joystick.start().whileTrue(
            new RunCommand(() -> hopper.runBeltMotor(-1.), hopper)
        );
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveRR.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.rightTrigger().whileTrue(new RunCommand(() -> elevator.increaseSetpoint(0.01)));
        joystick.leftTrigger().whileTrue(
            new SequentialCommandGroup(
                // new InstantCommand(() -> knuckle.score(), knuckle),
                new InstantCommand(() -> elevator.setSetpoint(kElevTran)),
                new WaitUntilCommand(() -> (Math.abs(elevator.getSetpoint()-elevator.getElevatorPosition()) < 0.01)),
                new InstantCommand(() -> arm.setSetpoint(kArmTran)).until(() -> arm.getEncoderPosition() < -0.225),/*,*/
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
            )
            // 
        );
        joystick.povUp().whileTrue(new RunCommand(() -> arm.setSetpoint(0.16)));
        joystick.povDown().whileTrue(new RunCommand(() -> arm.setSetpoint(0.)));
        /*OPERATOR CONTROLLER
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        */
        new JoystickButton(operator, Constants.OperatorConstants.kL1).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL1)),
                new InstantCommand(() -> arm.setSetpoint(kArmL1))
            )
        );
        new JoystickButton(operator, Constants.OperatorConstants.kL2).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL2)),
                new InstantCommand(() -> arm.setSetpoint(kArmL2))
            )
        );
        new JoystickButton(operator, Constants.OperatorConstants.kL3).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
                new InstantCommand(() -> arm.setSetpoint(kArmL3))
            )
        );
        new JoystickButton(operator, Constants.OperatorConstants.kL4).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
            )
        );
        new JoystickButton(operator, Constants.OperatorConstants.kAutoAlignLeft).whileTrue(
            new AutoAlignCommand(drivetrain, driveRR, true)
        );
        new JoystickButton(operator, Constants.OperatorConstants.kAutoAlignRight).whileTrue(
            new AutoAlignCommand(drivetrain, driveRR, false)
        );
        new JoystickButton(operator, k0degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED6_7, K_CONSTRAINTS)
        );
        new JoystickButton(operator, k60degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED4_5, K_CONSTRAINTS)
        );
        new JoystickButton(operator, k120degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED2_3, K_CONSTRAINTS)
        );
        new JoystickButton(operator, k180degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED0_1, K_CONSTRAINTS)
        );
        new JoystickButton(operator, k240degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED10_11, K_CONSTRAINTS)
        );
        new JoystickButton(operator, k300degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED8_9, K_CONSTRAINTS)
        );
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public void setStartingSetpoints() {
        arm.setSetpoint(arm.getEncoderPosition());
        elevator.setSetpoint(elevator.getElevatorPosition());
    }
}
