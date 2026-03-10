// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSub extends SubsystemBase {
  /** Creates a new CameraSub. */
  public CameraSub() {}

  
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");

    Constants constants = new Constants();

    public Double getDistance() {
        double targetOffsetAngle_Vertical = limelight.getEntry("ty").getDouble(0);
        double angleToGoalDegrees = constants.LIMELIGHT_ANGLE + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        double distanceFromLimelightToGoalInches = (constants.GOAL_HEIGHT - constants.LIMELIGHT_HEIGHT) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
