// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
   
    

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    Constants constants = new Constants();

    double targetOffsetAngle_Vertical = 0;
    double angleToGoalDegrees = 0;
    double angleToGoalRadians = 0;
    double limelightDistanceToTarget = 0;

    public static ShooterSub shooterSub;
    public static IntakeSub intakeSub;
    public static CameraSub cameraSub;


    public Robot() {
        shooterSub = new ShooterSub();
        intakeSub = new IntakeSub();
        cameraSub = new CameraSub();
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        LimelightHelpers.SetIMUMode("limelight", 4);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        
        double robotYawDegrees = m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
    
        LimelightHelpers.SetRobotOrientation("limelight", robotYawDegrees, 0 ,0, 0, 0, 0);

        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1 != null && mt1.tagCount > 0) {
            m_robotContainer.drivetrain.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds
            );
        }
    }
    

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}