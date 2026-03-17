// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
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
    leftServo.setBoundsMicroseconds(2400, 2000, 1500, 1000, 600);
    rightServo = new Servo(1);
    rightServo.setBoundsMicroseconds(2400, 2000, 1500, 1000, 600);

    config = new TalonFXConfiguration();

    config.Slot0.kV = 0.115;
    config.Slot0.kP = 0.2;

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
    return (middleError <= 7.0) &&
      (leftError <= 7.0) &&
      (rightError <= 7.0); 
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

    return (Math.abs(leftVelocity) < 3 && leftCurrent > 40) ||
           (Math.abs(middleVelocity) < 3 && middleCurrent > 40) ||
           (Math.abs(rightVelocity) < 3 && rightCurrent > 40);
  }

  double hoodDistanceOffset = 1;

  public int getHoodPosition(double distance) {
    if(distance < 57 + hoodDistanceOffset) {
      setHoodPos(0.258);
      return 2;
    } else if (distance < 102.7 + hoodDistanceOffset) {
      setHoodPos(0.333);
      return 3;
    } else {
      setHoodPos(0.41);
      return 4;
    }
}

  public double calculateTargetRPS(double distance) {
    int pos = getHoodPosition(distance); // Updates and returns the current state
    double rps;
    if (distance <= 57.0) {
        rps = (0.18882 * distance) + 60.41405; // Pos 2
    } else if (distance <= 102.7) {
        rps = (0.21168 * distance) + 53.79669; // Pos 3
    } else {
        rps = (0.07534 * distance) + 63.20878; // Pos 4
    }

    return MathUtil.clamp(rps, 60.0, 100.0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
