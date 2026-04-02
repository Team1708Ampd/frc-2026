package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

public class CameraSub extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final Constants constants = new Constants();


    public CameraSub() {}

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1.0;
    }
    /**
     * @return The horizontal offset in degrees. 
     * Applies offset ONLY if 2 or more tags are being averaged by the Limelight.
     */
    public double getTX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    public double getTY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    public double getDistance() {
        if (!hasTarget()) return 0.0;
        double targetOffsetAngle_Vertical = getTY();
        double angleToGoalRadians = Math.toRadians(constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical);
        return (constants.GOAL_HEIGHT - constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
    }

    public double getDistance3d(CommandSwerveDrivetrain drivetrain) {
        // 1. Get the robot's current pose from the drivetrain
        Pose2d currentPose = drivetrain.getState().Pose;

        // 2. Determine which goal we are aiming at
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Translation2d targetGoal = isBlue ? Constants.BLUE_GOAL_CENTER : Constants.RED_GOAL_CENTER;

        // 3. Calculate the straight-line distance (Hypotenuse)
        // .getDistance() is a built-in WPILib method for Translation2d
        return Units.metersToInches(currentPose.getTranslation().getDistance(targetGoal));
    }

    public Rotation2d getAngleToGoal(CommandSwerveDrivetrain drivetrain) {
        Pose2d currentPose = drivetrain.getState().Pose;

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Translation2d targetGoal = isBlue ? Constants.BLUE_GOAL_CENTER : Constants.RED_GOAL_CENTER;

        // Vector from robot to goal
        Translation2d delta = targetGoal.minus(currentPose.getTranslation());

        // Angle of that vector in field space
        return new Rotation2d(delta.getX(), delta.getY());
    }

    @Override
    public void periodic() {}
}