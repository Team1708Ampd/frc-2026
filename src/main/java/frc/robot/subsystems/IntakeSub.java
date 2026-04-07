// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSub;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ShooterSub;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
  /** Creates a new IntakeSub. */
  
  TalonFX intakeMotor;
  TalonFX intakeMotor2;
  TalonFX hopperMotor;
  TalonFX wristMotor; //
  

  public IntakeSub() {
    intakeMotor = new TalonFX(12);
    intakeMotor2 = new TalonFX(20);
    hopperMotor = new TalonFX(13);
    wristMotor = new TalonFX(10);
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Voltage.PeakForwardVoltage = 11.0;
    config.Voltage.PeakReverseVoltage = -11.0;
    config.CurrentLimits.SupplyCurrentLimit = 35.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeMotor.getConfigurator().apply(config);
    intakeMotor2.getConfigurator().apply(config);

    wristMotor.getConfigurator().apply(config);

    intakeMotor.setControl(new Follower(intakeMotor2.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public void setAllIntakes(double power) {
    intakeMotor2.set(power);
    hopperMotor.set(power);
  }

  public void setIntakePower(double power) {
    intakeMotor2.set(power);
  }

  public void setHopperPower(double power) {
    hopperMotor.set(power);
  }

  public void setWristPower(double power) {
    wristMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
