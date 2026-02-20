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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterSub extends SubsystemBase {
  /** Creates a new ShooterSub. */

  TalonFX leftShooter;
  TalonFX middleShooter;
  TalonFX rightShooter;

  Servo leftServo;
  Servo rightServo;

  TalonFXConfiguration config;
  VoltageOut voltageRequest;

/* * QUADRATIC CONSTANTS (Inches Version)
     * Values are placeholders. Note: kA will likely be a very small decimal.
     */
    private final double kA = 0.0001; 
    private final double kB = 0.02;   
    private final double kC = 3.5;

  public ShooterSub() {
    leftShooter = new TalonFX(15);
    middleShooter = new TalonFX(14);
    rightShooter = new TalonFX(11);

    // leftServo = new Servo(0);
    // rightServo = new Servo(1);

    config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 40; // Limit to 40 Amps
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftShooter.getConfigurator().apply(config);
    middleShooter.getConfigurator().apply(config);
    rightShooter.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftShooter.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightShooter.getConfigurator().apply(config);
    middleShooter.getConfigurator().apply(config);

    voltageRequest = new VoltageOut(0);
  }

  public void setAllShooters(double voltage) {
    leftShooter.setControl(voltageRequest.withOutput(voltage));    
    middleShooter.setControl(voltageRequest.withOutput(voltage));
    rightShooter.setControl(voltageRequest.withOutput(voltage));
  }

  public void setShooterVoltageByRegression(double distanceInches) {
        // V = ax^2 + bx + c
        double targetVoltage = (kA * Math.pow(distanceInches, 2)) 
                             + (kB * distanceInches) 
                             + kC;

        // Clamp between 0V and 12V for safety
        targetVoltage = Math.max(0.0, Math.min(targetVoltage, 12.0));

        leftShooter.setControl(voltageRequest.withOutput(targetVoltage));
        middleShooter.setControl(voltageRequest.withOutput(targetVoltage));
        rightShooter.setControl(voltageRequest.withOutput(targetVoltage));


        SmartDashboard.putNumber("Shooter/Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Shooter/Distance Inches", distanceInches);
    }

    public Servo getLeftServo() {
      return leftServo;
    }

    public Servo getRightServo() {
      return rightServo;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
