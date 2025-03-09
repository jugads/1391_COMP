// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.KnuckleCommand;
import frc.robot.commands.HopperCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Hopper;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ReefPoses.*;

import java.nio.file.OpenOption;

import static frc.robot.Constants.OperatorConstants.*;
public class RobotContainer {
    // Drive configuration
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = 3 * Math.PI;

    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    // Telemetry
    private final Telemetry logger = new Telemetry(MaxSpeed);
    Timer timer = new Timer();
    // Controllers
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick operator = new CommandJoystick(1);
    private final CommandXboxController manual = new CommandXboxController(2);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Knuckle knuckle = new Knuckle();
    public final Elevator elevator = new Elevator();
    public final AlgaeScorer algaeScorer = new AlgaeScorer();
    public final Arm arm = new Arm();
    public final Leds leds = new Leds(new AddressableLED(9), new AddressableLEDBuffer(138), arm, knuckle, algaeScorer, drivetrain);
    public final Hopper hopper = new Hopper();
    public final Climber climber = new Climber();
    public final AutonomousCommand autos = new AutonomousCommand(drivetrain, driveRR, elevator, arm, hopper, knuckle);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {drivetrain.getPigeon2().setYaw(0);}
        else if (DriverStation.getAlliance().get() == Alliance.Red) {drivetrain.getPigeon2().setYaw(180);}
        autoChooser.addOption("3-4-5-6", autos.branches3_4_5_6());
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Set default commands for subsystems
        configureDefaultCommands();

        // Configure driver controls
        configureDriverControls();

        // Configure operator controls
        configureOperatorControls();

        // Configure manual controls
        configureManualControls();
        // Register telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureDefaultCommands() {
        // Configure drivetrain default command for field-centric control
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive
                .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
                .withVelocityX(joystick.getLeftY() * MaxSpeed)
                .withVelocityY(joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Set default commands for other subsystems
        elevator.setDefaultCommand(new ElevatorCommand(elevator));
        knuckle.setDefaultCommand(new KnuckleCommand(knuckle));
        algaeScorer.setDefaultCommand(new RunCommand(() -> algaeScorer.runAlgaeScorer(algaeScorer.hasAlgae() ? 0.2 : 0.), algaeScorer));
        arm.setDefaultCommand(new ArmCommand(arm, elevator));
        hopper.setDefaultCommand(new HopperCommand(hopper));
        climber.setDefaultCommand(new InstantCommand(() -> climber.runClimber(0.), climber));
    }

    private void configureDriverControls() {
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // joystick.().whileTrue(new RunCommand(() -> knuckle.setKnuckleMotorHigh()));
        joystick.leftBumper().onTrue(new RunCommand(() -> knuckle.score(), knuckle).until(() -> !knuckle.hasCoral()));
        // joystick.rightBumper().whileTrue(
        //     new RunCommand(() -> hopper.runBoth(0.2, 1.), hopper)
        // );
        joystick.y().whileTrue(new ParallelCommandGroup(
           new InstantCommand(() -> elevator.setSetpoint(0.3)),
           new InstantCommand(() -> arm.setSetpoint(0.25))
        ));
        joystick.b().whileTrue(
            AutoBuilder.pathfindToPose(DriverStation.getAlliance().get() == Alliance.Red ? kREDSOURCERIGHT_center : kBLUESOURCERIGHT_center, K_CONSTRAINTS)
        );
        joystick.x().whileTrue(
            AutoBuilder.pathfindToPose(DriverStation.getAlliance().get() == Alliance.Red ? kREDSOURCELEFT_center : kBLUESOURCELEFT_center, K_CONSTRAINTS)
        );
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveRR.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (DriverStation.getAlliance().get() == Alliance.Red ? -0.3 : 0.3))
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (DriverStation.getAlliance().get() == Alliance.Red ? -0.3 : 0.3))
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );
        // joystick.leftTrigger().whileTrue(
        //     new TransferCommand(elevator, arm, knuckle, hopper)
        //     // new RunCommand(() -> knuckle.setKnuckleMotorHigh())
        // );
        joystick.back().whileTrue(
            new SequentialCommandGroup(
                new InstantCommand(() -> timer.restart()),
                drivetrain.applyRequest(() -> driveRR.withVelocityX(-0.75)).until(() -> timer.get() > 0.5),
                new ParallelCommandGroup(
                    new InstantCommand(() -> elevator.setSetpoint(0.4)),
                    new InstantCommand(() -> arm.setSetpoint(0.25))
                ),
                new InstantCommand(() -> timer.stop())
            )
        );
        joystick.povLeft().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povRight().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(-0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povUp().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.povDown().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(-0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        joystick.start().whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

    private void configureOperatorControls() {
        operator.button(kL1).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL1)),
                new InstantCommand(() -> arm.setSetpoint(kArmL1))
            )
        );
        operator.button(kL2).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL2)),
                new InstantCommand(() -> arm.setSetpoint(kArmL2))
            )
        );
        operator.button(kL3).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
                new InstantCommand(() -> arm.setSetpoint(kArmL3))
            )
        );
        operator.button(kL4).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
            )
        );
        operator.button(kAutoAlignLeft).whileTrue(
            new AutoAlignCommand(drivetrain, driveRR, true, elevator.getElevatorPosition() > 0.9, elevator)
        );
        operator.button(kAutoAlignRight).whileTrue(
            new AutoAlignCommand(drivetrain, driveRR, false, elevator.getElevatorPosition()>0.9, elevator)
        );
        operator.button(k0degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED6_7, K_CONSTRAINTS)
        );
        operator.button(k60degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED4_5, K_CONSTRAINTS)
        );
        operator.button(k120degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED2_3, K_CONSTRAINTS)
        );
        operator.button(k180degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED0_1, K_CONSTRAINTS)
        );
        operator.button(k240degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED10_11, K_CONSTRAINTS)
        );
        operator.button(k300degrees).and(joystick.a()).whileTrue(
            AutoBuilder.pathfindToPose(kRED8_9, K_CONSTRAINTS)
        );
        //Algae L2
        operator.axisGreaterThan(operator.getXChannel(), 0.99).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.2)),
                new InstantCommand(() -> arm.setSetpoint(0.25)),
                new RunCommand(() -> algaeScorer.runAlgaeScorer(0.8))
            )
        );
        //Algae l3
        operator.axisLessThan(operator.getXChannel(), -0.99).whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.6)),
                new InstantCommand(() -> arm.setSetpoint(0.25)),
                new RunCommand(() -> algaeScorer.runAlgaeScorer(0.8))
            )
        );
        operator.axisGreaterThan(operator.getYChannel(), 0.99).whileTrue(
            new RunCommand(() -> System.out.println("Processor"))
        );
        operator.axisLessThan(operator.getYChannel(), -0.99).whileTrue(
            new RunCommand(() -> System.out.println("Net"))
        );
        operator.axisLessThan(operator.getYChannel(), -0.99).and(joystick.leftBumper()).whileTrue(
            new RunCommand(() -> algaeScorer.score())
        );
    } 

    private void configureManualControls() {
        // manual.a().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(kElevL1)),
        //         new InstantCommand(() -> arm.setSetpoint(kArmL1))
        //     )
        // );
        // manual.b().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(kElevL2)),
        //         new InstantCommand(() -> arm.setSetpoint(kArmL2))
        //     )
        // );
        // manual.y().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(kElevL3)),
        //         new InstantCommand(() -> arm.setSetpoint(kArmL3))
        //     )
        // );
        // manual.x().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
        //         new InstantCommand(() -> arm.setSetpoint(kArmL4))
        //     )
        // );
        // manual.leftStick().onChange(manual.getLeftTriggerAxis(), 0.9).whileTrue(
        //     elevator.increaseSetpoint(0.05)
        // );
        manual.rightBumper().whileTrue(
            new RunCommand(() -> climber.runClimber(0.2), climber)
        );
        manual.leftBumper().whileTrue(
            new RunCommand(() -> climber.runClimber(-0.2), climber)
        );
        manual.a().whileTrue(
            new RunCommand(() -> knuckle.setKnuckleMotorHigh(), knuckle)
        );
        manual.rightTrigger().whileTrue(
            new ConditionalCommand(new TransferCommand(elevator, arm, knuckle, hopper), Commands.none(), () -> !knuckle.hasCoral())
        ); 
       }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
    public void setStartingSetpoints() {
        arm.setSetpoint(arm.getEncoderPosition());
        elevator.setSetpoint(elevator.getElevatorPosition());
    }
    public void inputs() {
        System.out.println(operator.getX());
    }
    public void setCoral() {
        knuckle.setHasCoral();
    }
}
