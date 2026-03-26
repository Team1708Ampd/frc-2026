package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterSub extends SubsystemBase {

  // Shooter Motors
  TalonFX leftShooter;
  TalonFX middleShooter;
  TalonFX rightShooter;

  // Progressive Feeder Motors
  TalonFX feederBottomAndMiddle;
  TalonFX feederTop;

  // Hood Components
  // TalonFX shooterHood;
  CANcoder hoodEncoder;
  
  // Control Requests
  MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VoltageOut m_voltageRequest = new VoltageOut(0);

  private double m_activeHoodTarget = 0.0;
  private int m_currentZone = 2;

  public ShooterSub() {
    leftShooter = new TalonFX(11);
    middleShooter = new TalonFX(14);
    rightShooter = new TalonFX(15);
    
    feederBottomAndMiddle = new TalonFX(8); // Example IDs
    feederTop = new TalonFX(19);

    // shooterHood = new TalonFX(18);
    // hoodEncoder = new CANcoder(20);

    /* --- Shooter Configuration --- */
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kV = 0.115;
    shooterConfig.Slot0.kP = 0.2;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    
    // Voltage Comp & Current Limits for stability
    shooterConfig.Voltage.PeakForwardVoltage = 11.0;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftShooter.getConfigurator().apply(shooterConfig);
    middleShooter.getConfigurator().apply(shooterConfig);

    shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooter.getConfigurator().apply(shooterConfig);

    /* --- Feeder Configuration --- */
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();
    feederConfig.Voltage.PeakForwardVoltage = 11.0;
    feederConfig.CurrentLimits.SupplyCurrentLimit = 35.0;
    feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    feederConfig.CurrentLimits.StatorCurrentLimit = 60.0; // Higher for dual-roller friction
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    feederBottomAndMiddle.getConfigurator().apply(feederConfig);
    feederTop.getConfigurator().apply(feederConfig);

    /* --- Hood Configuration --- */
    // TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
    // hoodConfig.Slot0.kP = 12.0; // High P for precision positioning
    // hoodConfig.Slot0.kD = 0.1;
    
    // // Link CANcoder to the Hood Motor
    // hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
    // hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    
    // hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 5; 
    // hoodConfig.MotionMagic.MotionMagicAcceleration = 10;
    
    // shooterHood.getConfigurator().apply(hoodConfig);
  }

  /** Runs the three shooter motors at target RPM */
  public void runShooter(double rpm) {
    double rps = rpm / 60.0;
    leftShooter.setControl(m_velocityRequest.withVelocity(rps));
    middleShooter.setControl(m_velocityRequest.withVelocity(rps));
    rightShooter.setControl(m_velocityRequest.withVelocity(rps));
  }

  public void runProgressiveFeeders(double shooterTargetRPS) {
    feederTop.setControl(m_voltageRequest.withOutput(11.0 * (shooterTargetRPS * 1.7 / 100.0)));
    feederBottomAndMiddle.setControl(m_voltageRequest.withOutput(11.0 * (shooterTargetRPS * 0.6 / 100.0)));
  }

  public void stopFeeders() {
    feederTop.setControl(m_voltageRequest.withOutput(0));
    feederBottomAndMiddle.setControl(m_voltageRequest.withOutput(0));
  }

  public void setHoodPosition(double targetRotations) {
    m_activeHoodTarget = targetRotations;
    // shooterHood.setControl(hoodRequest.withPosition(targetRotations));
  }

  // public double getCurrentHoodPosition() {
  //   return shooterHood.getPosition().getValueAsDouble();
  // }

  // public boolean isHoodAtPosition() {
  //   return Math.abs(getCurrentHoodPosition() - m_activeHoodTarget) < 0.01;
  // }

  /** Logic to handle zone switching with hysteresis and busy-guard */
  public int updateHoodZone(double distance) {
    double newTargetRotation = m_activeHoodTarget;
    double hysteresis = 2.0; 

    // Zone Logic with Hysteresis
    if (m_currentZone == 2 && distance > (57.0 + hysteresis)) {
        m_currentZone = 3;
    } else if (m_currentZone == 3) {
        if (distance < (57.0 - hysteresis)) m_currentZone = 2;
        else if (distance > (102.7 + hysteresis)) m_currentZone = 4;
    } else if (m_currentZone == 4 && distance < (102.7 - hysteresis)) {
        m_currentZone = 3;
    }

    // Set rotation based on current zone
    if (m_currentZone == 2) newTargetRotation = 0.15;
    else if (m_currentZone == 3) newTargetRotation = 0.30;
    else if (m_currentZone == 4) newTargetRotation = 0.45;

    // // Only update if reached previous target
    // if (isHoodAtPosition() && newTargetRotation != m_activeHoodTarget) {
    //     setHoodPosition(newTargetRotation);
    // }

    return m_currentZone;
  }

  /** Synchronizes RPS formula with the physical hood zone */
  public double calculateTargetRPS(double distance) {
    int zone = updateHoodZone(distance);
    double rps;

    if (zone == 2) rps = (0.18882 * distance) + 60.41405;
    else if (zone == 3) rps = (0.21168 * distance) + 53.79669;
    else rps = (0.07534 * distance) + 63.20878;

    return MathUtil.clamp(rps, 60.0, 100.0);
  }

  public boolean isShooterReady(double targetRPS) {
    double tolerance = 2.0; // Adjust this based on how much "dip" you can tolerate
    
    double leftVel = leftShooter.getVelocity().getValueAsDouble();
    double rightVel = rightShooter.getVelocity().getValueAsDouble();
    double centerVel = middleShooter.getVelocity().getValueAsDouble();

    // Check if every motor is within the "Acceptable Zone"
    boolean leftReady = Math.abs(leftVel - targetRPS) < tolerance;
    boolean rightReady = Math.abs(rightVel - targetRPS) < tolerance;
    boolean centerReady = Math.abs(centerVel - targetRPS) < tolerance;

    return leftReady && rightReady && centerReady;
}

  public Command getFeederDebugCommand() {
    return this.runEnd(
        () -> {
            // Using a static test speed of 60 RPS
            double testRPS = 60.0;
            this.runProgressiveFeeders(testRPS);
            
            // Log the "Actual" vs "Target" to see if 11V is enough
            SmartDashboard.putNumber("Debug/Stage3 Target", testRPS * 1.7);
            SmartDashboard.putNumber("Debug/Stage3 Actual", feederTop.getVelocity().getValueAsDouble());
        },
        () -> this.stopFeeders()
    ).withName("FeederDebug");
}

public void outtakeFromShooter() {
    double reverseVoltage = -11.0; 

    feederTop.setControl(m_voltageRequest.withOutput(reverseVoltage));
    feederBottomAndMiddle.setControl(m_voltageRequest.withOutput(reverseVoltage));

    leftShooter.setControl(m_voltageRequest.withOutput(-6));
    middleShooter.setControl(m_voltageRequest.withOutput(-6));
    rightShooter.setControl(m_voltageRequest.withOutput(-6));
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Hood/Position", getCurrentHoodPosition());
    SmartDashboard.putNumber("Hood/Target", m_activeHoodTarget);
    // SmartDashboard.putBoolean("Hood/At Target", isHoodAtPosition());
    SmartDashboard.putNumber("Shooter/Left Current", leftShooter.getSupplyCurrent().getValueAsDouble());
  }
}