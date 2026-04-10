// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.FeedAndShoot;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeWristIn;
import frc.robot.commands.IntakeWristOut;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.Outtake;
import frc.robot.commands.OuttakeAll;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSub;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController mech = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final AprilTagFieldLayout m_fieldLayout;

    private double shootPower = 3000;
    
// AUTO RELATED VARIABLES AND DEFS
    private SendableChooser<Command> autoSelect;

    public RobotContainer() {
        configureBindings();

        drivetrain.initPathPlannerConfig();
        registerNamedCommands();


        autoSelect = AutoBuilder.buildAutoChooser(""); // Default auto will be `Commands.none()`
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
        SmartDashboard.putData("Select Auto", autoSelect);

        // Register the commands for the autos
        // Schedule the selected auto
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
        drivetrain.swerveDriveDashboard();

        m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
    }


    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.7) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.7) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.9) // Drive counterclockwise with negative X (left)
        ));
        
        joystick.a().whileTrue(new FeedAndShoot(drivetrain));
        joystick.x().whileTrue(new ManualShoot(() -> shootPower));

        joystick.leftTrigger().whileTrue(new Intake());
        joystick.rightTrigger().whileTrue(new Outtake());
        joystick.rightBumper().whileTrue(new OuttakeAll());

        // joystick.povDown().onTrue(new InstantCommand(this::decrementShoot));
        // joystick.povUp().onTrue(new InstantCommand(this::incrementShoot));

        joystick.povUp().whileTrue(new IntakeWristIn());
        joystick.povDown().whileTrue(new IntakeWristOut());

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );


        drivetrain.registerTelemetry(logger::telemeterize);
    }

        public Command getAutonomousCommand()
    {
        return autoSelect.getSelected();
    }


     private void registerNamedCommands() //Update with Command Names
     {
        NamedCommands.registerCommand("IntakeWristOut", new IntakeWristOut());
        NamedCommands.registerCommand("IntakeWristIn", new IntakeWristIn());
        NamedCommands.registerCommand("CalculatedShoot", new FeedAndShoot(drivetrain));
        NamedCommands.registerCommand("Intake", new Intake());
        NamedCommands.registerCommand("Outtake", new Outtake());
        NamedCommands.registerCommand("OuttakeAll", new OuttakeAll());
    }

    public void incrementShoot() {
        shootPower += 50;
        System.out.println("INCREMENTED: " + shootPower);
    }

    public void decrementShoot() {
        shootPower -= 50;
        System.out.println("DECREMENTED: " + shootPower);
    }
}