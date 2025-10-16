// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.AutoCollectCommand;
import frc.robot.commands.AutomatedAlgaeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.IntakePivotCommand;
import frc.robot.commands.KnuckleCommand;
import frc.robot.commands.TransferCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeScorer;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Knuckle;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Strategy;
import frc.robot.subsystems.Hopper;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE4_5;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE6_7;
import static frc.robot.Constants.AlignmentPoses.kAliBLUE6_7L4;
import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.ClimberConstants.k90DegreesRotations;
import static frc.robot.Constants.ReefPoses.*;
import static frc.robot.LimelightHelpers.pose2dToArray;

import java.util.Set;

import static frc.robot.Constants.OperatorConstants.*;
public class RobotContainer {
    // Drive configuration
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = 3 * Math.PI;
    private double gyro = 1.;
    // Swerve drive requests
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed*0.1)
            .withRotationalDeadband(MaxAngularRate * 0.085)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric driveRR = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private Pose2d closestPose = new Pose2d();
    int level = 3;
    // Telemetry
    private final Telemetry logger = new Telemetry(MaxSpeed);
    Timer timer = new Timer();
    // Controllers
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);
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
    public final Strategy strat = new Strategy();
    // public final GroundIntake intake = new GroundIntake();
    public final AutonomousCommand autos = new AutonomousCommand(drivetrain, driveRR, elevator, arm, hopper, knuckle, algaeScorer);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.addOption("ONLY LEFT BRANCHES Right Side branches 3-4-5-6", autos.branches3_4_5_6_ALLLEFT());
        autoChooser.addOption("NORMAL Right Side branches 3-4-5-6", autos.branches3_4_5_6());
        autoChooser.addOption("ONLY LEFT BRANCHES LEFT Side branches 10-9-8-7", autos.branches10_9_8_7ALLLEFT());
        autoChooser.addOption("NORMAL LEFT Side branches 10-9-8-7", autos.branches10_9_8_7());
        autoChooser.addOption("Center Algae Auto", autos.center0AlgaeRemoval());
        autoChooser.addOption("Drive Straight", autos.driveStraight());
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
                .withVelocityX(-joystick.getLeftY() * MaxSpeed *gyro)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * gyro)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            )
        );

        // Set default commands for other subsystems
        elevator.setDefaultCommand(new ElevatorCommand(elevator, algaeScorer));
        knuckle.setDefaultCommand(new KnuckleCommand(knuckle));
        algaeScorer.setDefaultCommand(new RunCommand(() -> algaeScorer.runAlgaeScorer(algaeScorer.hasAlgae() ? 0.15 : 0.), algaeScorer));
        arm.setDefaultCommand(new ArmCommand(arm, elevator));
        hopper.setDefaultCommand(new RunCommand(() -> hopper.runBoth(0, 0), hopper));
        climber.setDefaultCommand(new InstantCommand(() -> climber.runClimber(0.), climber));
        // intake.setDefaultCommand(
        //     new ParallelCommandGroup(
        //         new RunCommand(() -> intake.runIntake(false, 0.)),
        //         new IntakePivotCommand(intake, true)
        //     )
        // );
    }

    private void configureDriverControls() {
        joystick.leftBumper().onTrue(new ConditionalCommand(new RunCommand(() -> knuckle.score(), knuckle), new RunCommand(() -> knuckle.scoreLowSpeed(), knuckle), () -> !(arm.getEncoderPosition() < 0)).until(() -> !knuckle.hasCoral()).andThen(new ConditionalCommand(new InstantCommand(() -> elevator.setSetpoint(kElevL1+0.04)), Commands.none(), () -> arm.getEncoderPosition() < 0.)));
        joystick.rightBumper().onTrue(new RunCommand(() -> algaeScorer.score()).until(() -> !algaeScorer.hasAlgae()));
        joystick.b().whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPoseBasedOnLL()),
            new ConditionalCommand(
              new ConditionalCommand(
                new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL3),
                new InstantCommand(() -> arm.setSetpoint(kArmL3))
                ), 
                new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL4),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
                ),
                () -> level == 3
              ), 
              new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL2),
                new InstantCommand(() -> arm.setSetpoint(kArmL2))
            ),  () -> level != 2),
              new WaitCommand(0.5),
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(level == 4 ? drivetrain.getNearestReefPoseRight() : drivetrain.getNearestReefPoseL4(false), K_CONSTRAINTS_Barging),
                Set.of() // required subsystem dependencies if any
            )
            )
        );
        joystick.x().whileTrue(
            new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetPoseBasedOnLL()),
            new ConditionalCommand(
              new ConditionalCommand(
                new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL3),
                new InstantCommand(() -> arm.setSetpoint(kArmL3))
                ), 
                new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL4),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
                ),
                () -> level == 3
              ), 
              new ParallelCommandGroup(
                elevator.setSetpointCMD(kElevL2),
                new InstantCommand(() -> arm.setSetpoint(kArmL2))
            ),  () -> level != 2),
              new WaitCommand(0.5),
            Commands.defer(
                () -> AutoBuilder.pathfindToPose(level == 4 ? drivetrain.getNearestReefPoseLeft() : drivetrain.getNearestReefPoseL4(true), K_CONSTRAINTS_Barging),
                Set.of() // required subsystem dependencies if any
            )
            )
        );
        joystick.leftTrigger().whileTrue(
            new RunCommand(() -> knuckle.runMotor(0.8))
        );
        // joystick.rightBumper().whileTrue(
        //     drivetrain.applyRequest(() ->
        //         driveRR.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );
        // joystick.povLeft().whileTrue(
        //     drivetrain.applyRequest(
        //     () ->
        //     driveRR
        //     .withVelocityX(0) // Drive forward with negative Y (forward)
        //     .withVelocityY(0.75) // Drive left with negative X (left)
        //     .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        // )
        // );
        // joystick.povRight().whileTrue(
        //     drivetrain.applyRequest(
        //     () ->
        //     driveRR
        //     .withVelocityX(0) // Drive forward with negative Y (forward)
        //     .withVelocityY(-0.75) // Drive left with negative X (left)
        //     .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        // )
        // );
        // joystick.povUp().whileTrue(
        //     drivetrain.applyRequest(
        //     () ->
        //     driveRR
        //     .withVelocityX(0.75) // Drive forward with negative Y (forward)
        //     .withVelocityY(0) // Drive left with negative X (left)
        //     .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        // )
        // );
        // joystick.povDown().whileTrue(
        //     drivetrain.applyRequest(
        //     () ->
        //     driveRR
        //     .withVelocityX(-0.75) // Drive forward with negative Y (forward)
        //     .withVelocityY(0) // Drive left with negative X (left)
        //     .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        // )
        // );
        // joystick.leftTrigger().whileTrue(new AutomatedAlgaeCommand(algaeScorer, drivetrain, driveRR, elevator, arm));
        // joystick.rightTrigger().whileTrue(AutoBuilder.pathfindToPose(kBLUESOURCERIGHT_center, K_CONSTRAINTS_Barging));

        joystick.a().whileTrue(new RunCommand(() -> algaeScorer.runAlgaeScorer(0.7)));

        joystick.start().whileTrue(new InstantCommand(() -> drivetrain.getPigeon2().reset()));
    }

    private void configureOperatorControls() {
        operator.leftTrigger().whileTrue(new TransferCommand(elevator, arm, knuckle, hopper));

        operator.povUp().onTrue(new InstantCommand(() -> level = MathUtil.clamp(level + 1, 2, 4)));
        operator.povDown().onTrue(new InstantCommand(() -> level = MathUtil.clamp(level - 1, 2, 4)));

        // operator.a().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(0.04)),
        //         new InstantCommand(() -> arm.setSetpoint(0.18))
        //     )
        // );
        // operator.y().whileTrue(
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(0.9985)).until(() -> elevator.getElevatorPosition() > 0.95),
        //         new InstantCommand(() -> arm.setSetpoint(0.3517))
        //     )
        // );

        // operator.leftBumper().whileTrue(
        //     new SequentialCommandGroup(
        //     new InstantCommand(() -> algaeScorer.resetGripper()),
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(0.31)),
        //         new InstantCommand(() -> arm.setSetpoint(0.165)),
        //         new RunCommand(() -> algaeScorer.runAlgaeScorer(0.8))
        //     )
        //     )
        // );
        // //Algae l3
        // operator.rightBumper().whileTrue(
        //     new SequentialCommandGroup(
        //     new InstantCommand(() -> algaeScorer.resetGripper()),
        //     new ParallelCommandGroup(
        //         new InstantCommand(() -> elevator.setSetpoint(0.57)),
        //         new InstantCommand(() -> arm.setSetpoint(0.19)),
        //         new RunCommand(() -> algaeScorer.runAlgaeScorer(0.8))
        //     )
        //     )
        // );

        // operator.b().whileTrue(new RunCommand(() -> climber.runClimber(0.2), climber));
    } 

    private void configureManualControls() {      
        manual.rightTrigger().whileTrue(
            new SequentialCommandGroup(
            new RunCommand(() -> climber.runClimber(1*manual.getRightTriggerAxis()), climber).until(() -> climber.getClimberPosition() > k90DegreesRotations).andThen(() -> climber.runClimber(0.)),
            new RunCommand(() -> elevator.runElevatorUp(-0.3), elevator).until(() -> elevator.getElevatorDown()).andThen(new InstantCommand(() -> elevator.setSetpoint(0.))),
            new InstantCommand(() -> arm.setClimbing()),
            new InstantCommand(() -> arm.setSetpoint(0.25))
            )
        );
        manual.leftTrigger().whileTrue(
            new ConditionalCommand(
                new TransferCommand(elevator, arm, knuckle, hopper), 
                Commands.none(), () -> !knuckle.hasCoral()
            )    
        );
        manual.back().whileTrue(
            new RunCommand(() -> climber.runClimber(0.8), climber)
        );
        manual.start().whileTrue(
            new RunCommand(() -> climber.runClimber(-0.25), climber)
        );
        manual.y().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.03)),
                new InstantCommand(() -> arm.setSetpoint(0.075)),
                new RunCommand(() -> algaeScorer.runAlgaeScorer(1.))
            )
        ); 
        manual.a().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(0.1)),
                new InstantCommand(() -> arm.setSetpoint(0.15)),
                new RunCommand(() -> algaeScorer.runAlgaeScorer(1.)) 
            )
        );      
        manual.x().whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> elevator.setSetpoint(kElevL4)),
                new InstantCommand(() -> arm.setSetpoint(kArmL4))
            )
        );
        manual.b().whileTrue(
            new RunCommand(() -> knuckle.setKnuckleMotorHigh())
        );
        manual.povLeft().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        manual.povRight().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0) // Drive forward with negative Y (forward)
            .withVelocityY(-0.75) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        manual.povUp().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        manual.povDown().whileTrue(
            drivetrain.applyRequest(
            () ->
            driveRR
            .withVelocityX(-0.75) // Drive forward with negative Y (forward)
            .withVelocityY(0) // Drive left with negative X (left)
            .withRotationalRate(0.) // Drive counterclockwise with negative X (left)
        )
        );
        operator.start().whileTrue(
            new InstantCommand(() -> knuckle.setHasCoral())
        );
        operator.y().onTrue(
            new ParallelCommandGroup(
            elevator.setSetpointCMD(kElevL4),
            new InstantCommand(() -> arm.setSetpoint(kArmL4))
            )
        );
        operator.a().onTrue(
            new ParallelCommandGroup(
            elevator.setSetpointCMD(kElevL3),
            new InstantCommand(() -> arm.setSetpoint(kArmL3))
            )
        );
        operator.b().onTrue(
            new ParallelCommandGroup(
            elevator.setSetpointCMD(kElevL2),
            new InstantCommand(() -> arm.setSetpoint(kArmL2))
            )
        );
        // maybe put arm and elevator on sticks?
        // manual.rightBumper().whileTrue(
        //     new RunCommand(() -> elevator.increaseSetpoint(0.05), elevator)
        // );
        // manual.leftBumper().whileTrue(
        //     new RunCommand(() -> elevator.increaseSetpoint(-0.05), elevator)
        // );
        // manual.povRight().whileTrue(
        //     new RunCommand(() -> arm.increaseSetpoint(0.005), arm)
        // );
        // manual.povLeft().whileTrue(
        //     new RunCommand(() -> arm.increaseSetpoint(-0.005), arm)
        // );
       }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    public void setStartingSetpoints() {
        arm.setSetpoint(arm.getEncoderPosition());
        elevator.setSetpoint(elevator.getElevatorPosition());
    }
    public void inputs() {
    }
    public void setCoral() {
        knuckle.setHasCoral();
    }
    public void setKnuckleNo() {
        knuckle.setCoralStateFalse();
    }
    public void setGyro() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {drivetrain.getPigeon2().setYaw(0);}
        else if (DriverStation.getAlliance().get() == Alliance.Red) {drivetrain.getPigeon2().setYaw(180);}
    }
    public void resetGyro() {
    gyro *= -1;
    }
    public void setPoseTarg() {
        if (closestPose == null) {
            System.out.println("WARNING: closestPose is null!");
        } else {
            System.out.println("Target set to: " + closestPose);
        }
    }
    public void dashboardUpdates() {
        SmartDashboard.putNumber("Current Level", level);
    }
}