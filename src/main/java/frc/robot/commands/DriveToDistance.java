package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter; // ADDED
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToDistance extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final double targetDistance;

  private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

  // Smoothed PID: Dropped xP slightly to stop the bounce
  private final PIDController xController = new PIDController(0.45, 0, 0); 
  private final PIDController yController = new PIDController(0.06, 0, 0);
  
  // Slew Rate Limiters: Limits acceleration to 2.5 meters/sec^2
  // This prevents the robot from "snapping" left/right and tilting the camera
  private final SlewRateLimiter xFilter = new SlewRateLimiter(2.5);
  private final SlewRateLimiter yFilter = new SlewRateLimiter(2.5);

  private final Constants constants = new Constants();
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public DriveToDistance(CommandSwerveDrivetrain drivetrain, double targetDistance) {
        this.drivetrain = drivetrain;
        this.targetDistance = targetDistance;

        xController.setTolerance(1.5); 
        yController.setTolerance(1.5); 
        addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    table.getEntry("priorityid").setNumber(10);
    // Reset filters on start to prevent old data from jumping the robot
    xFilter.reset(0);
    yFilter.reset(0);
  }

  @Override
  public void execute() {
    if (table.getEntry("tv").getDouble(0) == 1 && table.getEntry("tid").getDouble(-1) == 10) {
        double targetOffsetAngle_Vertical = table.getEntry("ty").getDouble(0);
        double targetOffsetAngle_Horizontal = table.getEntry("tx").getDouble(0);

        double angleToGoalDegrees = constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (constants.GOAL_HEIGHT - constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);

        double xSpeed = -xController.calculate(distanceFromLimelightToGoalInches, targetDistance);
        double ySpeed = yController.calculate(targetOffsetAngle_Horizontal, 0);

        // --- THE JITTER KILLERS ---

        // 1. Deadbands: Stop trying to be perfect
        if (Math.abs(distanceFromLimelightToGoalInches - targetDistance) < 2.0) xSpeed = 0;
        if (Math.abs(targetOffsetAngle_Horizontal) < 2.0) ySpeed = 0;

        // 2. Minimum Velocity: If speed is less than 0.05 m/s, just kill it 
        // to prevent the "approaching jitter"
        if (Math.abs(xSpeed) < 0.06) xSpeed = 0;
        if (Math.abs(ySpeed) < 0.06) ySpeed = 0;

        // 3. Slew Rate Limiting: Prevent the "Snap" and camera tilt
        double smoothX = xFilter.calculate(xSpeed);
        double smoothY = yFilter.calculate(ySpeed);

        // 4. Clamping
        smoothX = MathUtil.clamp(smoothX, -0.7, 0.7);
        smoothY = MathUtil.clamp(smoothY, -0.7, 0.7);

        drivetrain.setControl(request.withVelocityX(smoothX).withVelocityY(smoothY));
    } else {
      drivetrain.setControl(request.withVelocityX(0).withVelocityY(0));
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0));
  }

  @Override
  public boolean isFinished() {
    if (table.getEntry("tv").getDouble(0) == 0 || table.getEntry("tid").getDouble(-1) != 10) {
        return true;
    }
    return xController.atSetpoint() && yController.atSetpoint();
  }
}