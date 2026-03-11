// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.io.IOException;
import java.math.BigDecimal;
import java.util.List;
import java.util.Set;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.CalculatedShoot;
import frc.robot.commands.CalibrateActuator;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.FeedShooter;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeWristIn;
import frc.robot.commands.IntakeWristOut;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.Outtake;
import frc.robot.commands.OuttakeFromShooter;
import frc.robot.commands.SetActuators;
import frc.robot.commands.ShootAtDistance;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSub;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


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

    private double shootPower = 5000;
    
// AUTO RELATED VARIABLES AND DEFS
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        drivetrain.initPathPlannerConfig();
        registerNamedCommands();


        autoChooser = AutoBuilder.buildAutoChooser("testing"); // Default auto will be `Commands.none()`
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
        SmartDashboard.putData("Auto Chooser", autoChooser);

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
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.85) // Drive counterclockwise with negative X (left)
        ));

        Command shootCommand = Commands.sequence(
           new ManualShoot(() -> shootPower).until(() -> Robot.shooterSub.isShooterJammed()),
           new OuttakeFromShooter().withTimeout(0.5)
        ).repeatedly();

        Command calculatedShootCommand = Commands.sequence(
           new CalculatedShoot().until(() -> Robot.shooterSub.isShooterJammed()),
           new OuttakeFromShooter().withTimeout(0.5)
        ).repeatedly();
        
        joystick.a().whileTrue(calculatedShootCommand);

        joystick.rightTrigger().whileTrue(new Intake());
        joystick.leftTrigger().whileTrue(new Outtake());
        // joystick.leftBumper().whileTrue(new FeedShooter());
        joystick.rightBumper().whileTrue(new OuttakeFromShooter());

        // joystick.start().onTrue(new SetActuators(() -> 0.2));
        // joystick.back().onTrue(new SetActuators(() -> 0.3));

        // joystick.povUp().onTrue(Commands.runOnce(() -> incrementShoot()));
        // joystick.povDown().onTrue(Commands.runOnce(() -> decrementShoot()));

        joystick.povUp().whileTrue(new IntakeWristIn());
        joystick.povDown().whileTrue(new IntakeWristOut());

        // joystick.leftTrigger().whileTrue(new ManualShoot());
        // joystick.b().toggleOnTrue(new ShootAtDistance());

        // mech.start().whileTrue(new CalibrateActuator(shooterSub.getLeftServo(), shooterSub.getRightServo(), mech));
        

        // joystick.x().onTrue(
        //     new DeferredCommand(() -> driveForwardOneMeter(), Set.of(drivetrain))
        // );
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
        return autoChooser.getSelected();
    }


     private void registerNamedCommands() //Update with Command Names
    {
        NamedCommands.registerCommand("DriveToLadder", driveForwardOneMeter());
        try {
            NamedCommands.registerCommand("ReturnToPath", pathfindToNextPath("LadderToGoal"));
        } catch (FileVersionException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (ParseException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }


    public Command driveForwardOneMeter() {
        return Commands.defer(() -> {
            // 1. Setup Constants
            final double lensHeightInches = 27.0;
            final double tagHeightInches = 21.75;
            final double mountAngleDegrees = 8.1798331;
            final double targetDistanceInches = 75.0;

            // 2. Check for Target
            if (!LimelightHelpers.getTV("limelight") || LimelightHelpers.getFiducialID("limelight") != 15) {
                return Commands.none();
            }

            // 3. Trig Calculation
            double ty = LimelightHelpers.getTY("limelight");
            double tx = LimelightHelpers.getTX("limelight");
            double angleToGoalRadians = Math.toRadians(mountAngleDegrees + ty);
            double currentDistanceInches = (tagHeightInches - lensHeightInches) / Math.tan(angleToGoalRadians);

            // 4. Transform Calculation (Inches to Meters)
            double distanceToMoveMeters = (currentDistanceInches - targetDistanceInches) * 0.0254;

            Pose2d currentPose = drivetrain.getState().Pose;
            Pose2d targetPose = currentPose.transformBy(
                new Transform2d(new Translation2d(distanceToMoveMeters, 0.0), Rotation2d.fromDegrees(-tx))
            );

            // 5. Build Path
            PathPlannerPath path = new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(currentPose, targetPose),
                new PathConstraints(1.5, 1.5, Math.PI, 2 * Math.PI),
                new IdealStartingState(0.0, currentPose.getRotation()),
                new GoalEndState(0.0, targetPose.getRotation())
            );

            path.preventFlipping = true;
            return AutoBuilder.followPath(path);
        }, Set.of(drivetrain));
    }
 
    public Command pathfindToNextPath(String nextPathName) throws FileVersionException, IOException, ParseException {
    // This creates a command that pathfinds to the start of a path file 
    // and then follows it seamlessly.
     return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile(nextPathName),
            new PathConstraints(3.0, 3.0, Math.PI, 2 * Math.PI)
        ); 
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