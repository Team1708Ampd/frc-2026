// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToGoal extends Command {
  /** Creates a new AlignToGoal. */
    private final CommandSwerveDrivetrain drivetrain;
    private final Constants constants = new Constants();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

    private final PIDController m_pid = new PIDController(0.03, 0.0, 0.001);

    private final double kSearchSpeed = 3;

  public AlignToGoal(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
    m_pid.setTolerance(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pid.reset();

    LimelightHelpers.LimelightResults latestResult = LimelightHelpers.getLatestResults("limelight");
    if (latestResult.targets_Fiducials.length > 0) {
      for(LimelightTarget_Fiducial tag : latestResult.targets_Fiducials) {
        System.out.println("TAG ID " + tag.fiducialID);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0;
    double tx = table.getEntry("tx").getDouble(0);

    if (hasTarget) {
      double rotationOutput = m_pid.calculate(tx, 0);   
      double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

      double rotationSpeed = rotationOutput * maxAngularRate;         
      // Apply rotation. Ensure 'false' is used for field-relative 
      // so it rotates around the robot's current center.
      // drivetrain.setControl(
        // request.withVelocityX(0)
          // .withVelocityY(0)
          // .withRotationalRate(rotationSpeed));
    } else {
      // drivetrain.setControl(
                // request.withVelocityX(0).withVelocityY(0).withRotationalRate(kSearchSpeed)
      // );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0 && table.getEntry("tid").getDouble(-1) == 10;
    return m_pid.atSetpoint() && hasTarget;
  }
}
