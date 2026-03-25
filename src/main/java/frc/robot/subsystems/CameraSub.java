package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSub extends SubsystemBase {
    // Accessing the Limelight NetworkTable
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final Constants constants = new Constants();

    public CameraSub() {}

    /** @return True if the Limelight sees a valid target (AprilTag or Retroreflective) */
    public boolean hasTarget() {
        // "tv" is 1.0 if a target is found, 0.0 otherwise
        return limelight.getEntry("tv").getDouble(0) == 1.0;
    }

    /** @return The horizontal offset from the target in degrees (-29.8 to 29.8) */
    public double getTX() {
        return limelight.getEntry("tx").getDouble(0);
    }

    /** @return The vertical offset from the target in degrees */
    public double getTY() {
        return limelight.getEntry("ty").getDouble(0);
    }

    /** Calculates distance to the goal using trigonometry */
    public double getDistance() {
        if (!hasTarget()) return 0.0; // Avoid math on a junk "ty" value

        double targetOffsetAngle_Vertical = getTY();
        
        // Sum of the mounting angle and the offset angle seen by the camera
        double angleToGoalRadians = Math.toRadians(constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical);
        
        // Standard d = (h2 - h1) / tan(a1 + a2) formula
        double distanceFromLimelightToGoalInches = (constants.GOAL_HEIGHT - constants.LIMELIGHT_HEIGHT) 
                                                   / Math.tan(angleToGoalRadians);
                                                   
        return distanceFromLimelightToGoalInches;
    }

    @Override
    public void periodic() {
        // You could add SmartDashboard logging here to verify distance in the pits
    }
}