// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  
  TalonFX intakeMotor;
  TalonFX hopperMotor;

  public IntakeSub() {
    intakeMotor = new TalonFX(11);
    hopperMotor = new TalonFX(12);
  }

  public void setBothIntakes(double power) {
    intakeMotor.set(power);
    hopperMotor.set(power);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public void setHopperPower(double power) {
    hopperMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
