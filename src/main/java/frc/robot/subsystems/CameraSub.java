package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class CameraSub extends SubsystemBase {
    private final NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    private final Constants constants = new Constants();

    // Physical distance (inches) from the grouped center to the true Goal Center.
    private static final double PHYSICAL_OFFSET_INCHES = 7.0; 

    private boolean m_isGrouped = false;

    public CameraSub() {}

    public boolean hasTarget() {
        return limelight.getEntry("tv").getDouble(0) == 1.0;
    }

    /**
     * Checks if the Limelight currently sees multiple tags.
     * If 2+ tags are seen, we assume 'tx' is the average of them (Grouped).
     */
    public boolean isSeeingMultipleTags() {
        var results = LimelightHelpers.getLatestResults("limelight");
        
        // Check if the fiducial array exists and has more than one entry
        if (results != null && results.targets_Fiducials != null) {
            m_isGrouped = results.targets_Fiducials.length >= 2;
            return results.targets_Fiducials.length >= 2;
        }
        return false;
    }

    /**
     * @return The horizontal offset in degrees. 
     * Applies offset ONLY if 2 or more tags are being averaged by the Limelight.
     */
    public double getTX() {
        double rawTX = limelight.getEntry("tx").getDouble(0);
        m_isGrouped = isSeeingMultipleTags();
        
        // If we only see ONE tag, the Limelight is already centered on it.
        // We don't want to offset a single-tag target!
        if (!hasTarget() || !m_isGrouped) {
            return rawTX;
        }

        double distance = getDistance();
        if (distance < 1.0) return rawTX;

        // Dynamic offset calculation
        double angularOffsetDegrees = Math.toDegrees(Math.atan(PHYSICAL_OFFSET_INCHES / distance));

        // Adjust based on your 'too far left' observation
        return rawTX - angularOffsetDegrees;
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

    @Override
    public void periodic() {
        // Cache the grouping status once per loop
        m_isGrouped = isSeeingMultipleTags();
    }
}