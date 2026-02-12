// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  TalonFX leftShooter;
  TalonFX middleShooter;
  TalonFX rightShooter;

  public ShooterSub() {
    leftShooter = new TalonFX(8);
    middleShooter = new TalonFX(9);
    rightShooter = new TalonFX(10);
  }

  public void setAllShooters(double power) {
    leftShooter.set(power);
    middleShooter.set(power);
    rightShooter.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
