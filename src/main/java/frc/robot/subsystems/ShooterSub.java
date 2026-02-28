// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  TalonFX leftShooter;
  TalonFX middleShooter;
  TalonFX rightShooter;

  Servo leftServo;
  Servo rightServo;

  TalonFXConfiguration config;
  private final VelocityVoltage m_velocityRequest;
/* * QUADRATIC CONSTANTS (Inches Version)
     * Values are placeholders. Note: kA will likely be a very small decimal.
     */
    private final double kA = 0.0001; 
    private final double kB = 0.02;   
    private final double kC = 3.5;

  public ShooterSub() {
    leftShooter = new TalonFX(11);
    middleShooter = new TalonFX(14);
    rightShooter = new TalonFX(15);

    leftServo = new Servo(0);
    leftServo.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    rightServo = new Servo(1);
    rightServo.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40; // Limit to 40 Amps
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.Slot0.kV = 0.115;
    config.Slot0.kP = 0.2;

    config.Voltage.PeakForwardVoltage = 10.0;
    config.Voltage.PeakReverseVoltage = -10.0;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftShooter.getConfigurator().apply(config);
    middleShooter.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooter.getConfigurator().apply(config);

    m_velocityRequest = new VelocityVoltage(0);
  }

  public void runShooter(DoubleSupplier rpm) {
    leftShooter.setControl(m_velocityRequest.withVelocity(rpm.getAsDouble() / 60));
    middleShooter.setControl(m_velocityRequest.withVelocity(rpm.getAsDouble() / 60));
    rightShooter.setControl(m_velocityRequest.withVelocity(rpm.getAsDouble() / 60));
  }

  public boolean isShooterReady(DoubleSupplier targetRPM) {
    double currentLeft = leftShooter.getVelocity().getValueAsDouble();
    double currentMiddle = middleShooter.getVelocity().getValueAsDouble();
    double currentRight = rightShooter.getVelocity().getValueAsDouble();

    double leftError = Math.abs(currentLeft - (targetRPM.getAsDouble() / 60));
    double middleError = Math.abs(currentMiddle - (targetRPM.getAsDouble() / 60));
    double rightError = Math.abs(currentRight - (targetRPM.getAsDouble() / 60));

    System.out.print("LEFT ERROR: " + leftError + ", CURRENT: " + currentLeft);

    
    // Only allow feeding if the shooter is within 2 rotations per second of target
    return (middleError < 3.0) &&
      (leftError < 3.0) &&
      (rightError < 3.0); 
}

  public void setTargetVelocity() {

  }

  public void setHoodPos(double pos) {
    leftServo.setPosition(pos);
    rightServo.setPosition(pos);
  }

  public boolean isShooterJammed() {
    double leftCurrent = leftShooter.getStatorCurrent().getValueAsDouble();
    double leftVelocity = leftShooter.getVelocity().getValueAsDouble();

    double middleCurrent = middleShooter.getStatorCurrent().getValueAsDouble();
    double middleVelocity = middleShooter.getVelocity().getValueAsDouble();

    double rightCurrent = rightShooter.getStatorCurrent().getValueAsDouble();
    double rightVelocity = rightShooter.getVelocity().getValueAsDouble();

    return (Math.abs(leftVelocity) < 1 && leftCurrent > 40) ||
           (Math.abs(middleVelocity) < 1 && middleCurrent > 40) ||
           (Math.abs(rightVelocity) < 1 && rightCurrent > 40);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
