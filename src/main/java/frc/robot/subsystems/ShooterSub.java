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
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterSub extends SubsystemBase {

    // Shooter Motors
    private final TalonFX leftShooter;
    private final TalonFX middleShooter;
    private final TalonFX rightShooter;

    // Progressive Feeder Motors
    private final TalonFX feederBottomAndMiddle;
    private final TalonFX feederTop;

    // Hood Components
    private final TalonFX shooterHood;
    private final CANcoder hoodEncoder;
  
    // Control Requests
    private final MotionMagicVoltage hoodRequest = new MotionMagicVoltage(0);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final VoltageOut m_voltageRequest = new VoltageOut(0);

    private double m_activeHoodTarget = 0.0;
    
    // Simplified Zone Tracking
    private boolean m_isFarZone = false; 
    private final double ZONE_THRESHOLD = 65.0;
    private final double HYSTERESIS = 2.0;

    public ShooterSub() {
        leftShooter = new TalonFX(11);
        middleShooter = new TalonFX(14);
        rightShooter = new TalonFX(15);
        
        feederBottomAndMiddle = new TalonFX(8); 
        feederTop = new TalonFX(19);

        shooterHood = new TalonFX(21);
        hoodEncoder = new CANcoder(5);

        /* --- Shooter Configuration --- */
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.Slot0.kV = 0.115;
        shooterConfig.Slot0.kP = 0.2;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
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
        feederConfig.CurrentLimits.StatorCurrentLimit = 60.0; 
        feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        feederBottomAndMiddle.getConfigurator().apply(feederConfig);
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederTop.getConfigurator().apply(feederConfig);

        /* --- Hood Configuration --- */
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = 12.0; 
        hoodConfig.Slot0.kD = 0.1;
        hoodConfig.Feedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 5; 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 10;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //reversed rotation

        
        shooterHood.getConfigurator().apply(hoodConfig);
    }

    public void moveShooterHood(double power) {
        shooterHood.set(power);
        System.out.println("HOOD POSITION: " + hoodEncoder.getAbsolutePosition());
    }

    public void runShooter(double rpm) {
        double rps = rpm / 60.0;
        leftShooter.setControl(m_velocityRequest.withVelocity(rps));
        middleShooter.setControl(m_velocityRequest.withVelocity(rps));
        rightShooter.setControl(m_velocityRequest.withVelocity(rps));
    }

    public void runProgressiveFeeders(double shooterTargetRPS) {
        // Updated to your latest tuned ratios
        feederTop.setControl(m_voltageRequest.withOutput(11.0 * (shooterTargetRPS * 1.9 / 100.0)));
        feederBottomAndMiddle.setControl(m_voltageRequest.withOutput(11.0 * (shooterTargetRPS * 1.8 / 100.0)));
    }

    public void stopFeeders() {
        feederTop.setControl(m_voltageRequest.withOutput(0));
        feederBottomAndMiddle.setControl(m_voltageRequest.withOutput(0));
    }

    public void setHoodPosition(double targetRotations) {
        // Busy-guard: only update if the target is actually different
        if (targetRotations != m_activeHoodTarget) {
            m_activeHoodTarget = targetRotations;
            shooterHood.setControl(hoodRequest.withPosition(targetRotations));
        }
    }

    public boolean isHoodAtPosition() {
        return Math.abs(shooterHood.getPosition().getValueAsDouble() - m_activeHoodTarget) < 0.01;
    }

    /** * Core logic: Updates hood position and returns target RPS.
     * Uses a 2-inch hysteresis gap around the 65-inch mark.
     */
    public double calculateTargetRPS(double distance) {
        // Hysteresis Logic
        if (!m_isFarZone && distance > (ZONE_THRESHOLD + HYSTERESIS)) {
            m_isFarZone = true;
        } else if (m_isFarZone && distance < (ZONE_THRESHOLD - HYSTERESIS)) {
            m_isFarZone = false;
        }

        double rps;
        if (!m_isFarZone) {
            // STANDARD ZONE: y = 0.23x + 40.4
            rps = (0.23 * distance) + 40.4;
            setHoodPosition(0.15); 
        } else {
            // FAR ZONE: placeholder slope
            rps = (0.15 * distance) + 55.0; 
            setHoodPosition(0.45); 
        }

        return MathUtil.clamp(rps, 45.0, 100.0);
    }

    public boolean isShooterReady(double targetRPS) {
        double tolerance = 3.0; 
        boolean leftReady = Math.abs(leftShooter.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
        boolean rightReady = Math.abs(rightShooter.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
        boolean centerReady = Math.abs(middleShooter.getVelocity().getValueAsDouble() - targetRPS) < tolerance;
        return leftReady && rightReady && centerReady;
    }

    public void outtakeFromShooter() {
        m_voltageRequest.withOutput(-11.0);
        feederTop.setControl(m_voltageRequest);
        feederBottomAndMiddle.setControl(m_voltageRequest);
        leftShooter.setControl(m_voltageRequest.withOutput(-6));
        middleShooter.setControl(m_voltageRequest.withOutput(-6));
        rightShooter.setControl(m_voltageRequest.withOutput(-6));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Hood Target", m_activeHoodTarget);
        SmartDashboard.putBoolean("Shooter/Is Far Zone", m_isFarZone);
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady(m_activeHoodTarget)); // Note: this is an estimate in periodic
    }
}