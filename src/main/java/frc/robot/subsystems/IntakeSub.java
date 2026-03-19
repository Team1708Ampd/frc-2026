// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  
  TalonFX intakeMotor;
  TalonFX intakeMotor2;
  TalonFX hopperMotor;
  TalonFX feederMotor;
  TalonFX wristMotor; //

  public IntakeSub() {
    intakeMotor = new TalonFX(12);
    intakeMotor2 = new TalonFX(20);
    hopperMotor = new TalonFX(13);
    feederMotor = new TalonFX(8);
    wristMotor = new TalonFX(10);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Voltage.PeakForwardVoltage = 10.0;
    config.Voltage.PeakReverseVoltage = -10.0;

    intakeMotor.getConfigurator().apply(config);
    hopperMotor.getConfigurator().apply(config);
    feederMotor.getConfigurator().apply(config);
    wristMotor.getConfigurator().apply(config);
  }

  public void setBothIntakes(double power) {
    intakeMotor.set(power);
    intakeMotor2.set(power);
    hopperMotor.set(power);
    feederMotor.set(power);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
    intakeMotor2.set(power);
  }

  public void setHopperPower(double power) {
    hopperMotor.set(power);
  }

  public void setIntakeWristPower(double power){
    wristMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
