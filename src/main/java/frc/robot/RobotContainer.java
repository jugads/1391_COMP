// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.KnuckleDefault;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ArmConstants.*;
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final Joystick operator = new Joystick(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Knuckle knuckle = new Knuckle();
    public final Elevator elevator = new Elevator();
    public final AlgaeScorer algaeScorer = new AlgaeScorer();
    public final Arm arm = new Arm();
    public final Leds leds = new Leds(new AddressableLED(9), new AddressableLEDBuffer(138), arm, knuckle, algaeScorer);
    
    public RobotContainer() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {drivetrain.getPigeon2().setYaw(0);}
   else if (DriverStation.getAlliance().get() == Alliance.Red) {drivetrain.getPigeon2().setYaw(180);}
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
        knuckle.setDefaultCommand(new KnuckleDefault(knuckle));
        algaeScorer.setDefaultCommand(new RunCommand(() -> algaeScorer.runAlgaeScorer(algaeScorer.hasAlgae() ? 0.2 : 0.), algaeScorer));
        arm.setDefaultCommand(new ArmCommand(arm));
        // reset the field-centric heading on left bumper press

        /*DRIVER CONTROLLER
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        ***************************************************************************************
        */
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        joystick.a().whileTrue(new RunCommand(() -> knuckle.setKnuckleMotorHigh()));
        joystick.b().whileTrue(new RunCommand(()-> arm.runMotor(0.1), arm));
        joystick.x().whileTrue(new RunCommand(()-> arm.runMotor(-0.1), arm));
        joystick.leftBumper().whileTrue(new RunCommand(() -> knuckle.score(), knuckle));
        joystick.rightTrigger().whileTrue(new RunCommand(() -> elevator.runElevatorUp(0.2), elevator));
        joystick.leftTrigger().whileTrue(new RunCommand(() -> elevator.runElevatorUp(-0.1), elevator));
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
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
    public void setStartingSetpoints() {
        arm.setSetpoint(arm.getEncoderPosition());
        elevator.setSetpoint(elevator.getElevatorPosition());
    }
}
