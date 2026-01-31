// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToDistance extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final double targetDistance;

  private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

  private final PIDController xController = new PIDController(0.6, 0, 0.05);
  // Added yController for strafing. Low P (0.02) to prevent jitter.
  private final PIDController yController = new PIDController(0.02, 0, 0);
  

  private final Constants constants = new Constants();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /** Creates a new DriveToDistance. */
public DriveToDistance(CommandSwerveDrivetrain drivetrain, double targetDistance) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;

        xController.setTolerance(1.5); 
        yController.setTolerance(1.0); // 1 degree tolerance for horizontal centering
        addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    table.getEntry("priorityid").setNumber(10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if target is visible AND if the target ID is 10
    if (table.getEntry("tv").getDouble(0) == 1 && table.getEntry("tid").getDouble(-1) == 10) {
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
        double targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);

        double angleToGoalDegrees = constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (constants.GOAL_HEIGHT - constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);

        double xSpeed = xController.calculate(distanceFromLimelightToGoalInches, targetDistance);
        // Calculate ySpeed based on tx. Negated because +tx is right, but +Y in RobotCentric is left.
        double ySpeed = -yController.calculate(targetOffsetAngle_Horizontal, 0);

        xSpeed = MathUtil.clamp(xSpeed, -2, 2);
        ySpeed = MathUtil.clamp(ySpeed, -1, 1); // Clamp strafe speed for stability

        drivetrain.setControl(request.withVelocityX(-xSpeed).withVelocityY(ySpeed));
    } else {
      // If target 10 is lost, stop moving
      drivetrain.setControl(request.withVelocityX(0).withVelocityY(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // End the command if the target is lost so it doesn't get stuck, or if we are at setpoint
    if (table.getEntry("tv").getDouble(0) == 0 || table.getEntry("tid").getDouble(-1) != 10) {
        return true;
    }
    return xController.atSetpoint() && yController.atSetpoint();
  }
}