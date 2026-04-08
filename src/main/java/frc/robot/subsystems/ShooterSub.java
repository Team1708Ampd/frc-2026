package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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
    
    // Simplified Zone Tracking
    private boolean m_isFarZone = false; 
    private final double ZONE_THRESHOLD = 65.0;
    private final double HYSTERESIS = 2.0;

    DigitalInput limitSwitch;

    private double m_currentShooterTargetRPS = 0.0;

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

        limitSwitch = new DigitalInput(9);
    }

    public void moveShooterHood(double power) {
        shooterHood.set(power);
        System.out.println("HOOD POSITION: " + hoodEncoder.getAbsolutePosition().getValueAsDouble());
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

    private final VoltageOut m_homingRequest = new VoltageOut(0);
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);

    private double m_activeHoodTarget = 0.0;
    private boolean m_isHoming = false;

    public double calculateTargetRPS(double distance) {
        double threshold = 90.4;
        double gap = 3.0; 

        if (!m_isFarZone && distance > (threshold + gap)) {
            m_isFarZone = true;
        } else if (m_isFarZone && distance < (threshold - gap)) {
            m_isFarZone = false;
        }

        double rpm;
        if (!m_isFarZone) {
            rpm = (13.75 * distance) + 2000;
            m_activeHoodTarget = 0.0; // Standardize 0.0 as the "Bottom" target
            System.out.println("CLOSE SHOT");
        
            if (!getHoodLimitSwitch()) { 
                shooterHood.set(-0.1);
                System.out.println("MOVE HOOD DOWN");
            } else {
                shooterHood.set(0);
                shooterHood.setPosition(0);
                hoodEncoder.setPosition(0);
                System.out.println("STOPPED HOOD");
            }
        } else {
           rpm = (12.25 * distance) + 1750;
        m_activeHoodTarget = 0.27;
        double currentPos = hoodEncoder.getAbsolutePosition().getValueAsDouble();

        // Use a "Deadband" of 0.01 to prevent jittering
            if (Math.abs(currentPos - 0.27) < 0.02) {
                shooterHood.set(0);
                System.out.println("FAR POSITION: AT TARGET");
            } else if (currentPos > 0.27) {
                // Need to go DOWN
                shooterHood.set(-0.07);
                System.out.println("FAR POSITION: MOVING DOWN TO 0.27");
            } else {
                // Need to go UP
                shooterHood.set(0.07);
                System.out.println("FAR POSITION: MOVING UP TO 0.27");
            }
        }

        double rps = rpm / 60.0;
        this.m_currentShooterTargetRPS = rps; 
        return rps;
    }

    public boolean isHoodAtPosition() {
        if (!m_isFarZone) {
            return getHoodLimitSwitch(); // Ready if switch is pressed
        }
        // Ready if within 0.01 rotations of 0.27
        System.out.println("AT POSITION?" + (Math.abs(hoodEncoder.getAbsolutePosition().getValueAsDouble() - 0.25) < 0.02));
        return Math.abs(hoodEncoder.getAbsolutePosition().getValueAsDouble() - 0.27) < 0.02;
    }

    public boolean isShooterReady(double targetRPS) {
        double tolerance = 5.0; 
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

    public boolean getHoodLimitSwitch() {
        return limitSwitch.get();
    }

    public void zeroHoodEncoder() {
        hoodEncoder.setPosition(0);
    }

    public void setHoodToPass() {
        double currentPos = hoodEncoder.getAbsolutePosition().getValueAsDouble();

        if (Math.abs(currentPos - 0.38) < 0.02) {
            shooterHood.set(0);
        } else if (currentPos > 0.38) {
            shooterHood.set(-0.07);
        } else {
            shooterHood.set(0.07);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Hood Target", m_activeHoodTarget);
        SmartDashboard.putBoolean("Shooter/Is Far Zone", m_isFarZone);
        SmartDashboard.putBoolean("Shooter/Ready", isShooterReady(m_activeHoodTarget)); // Note: this is an estimate in periodic
    }
}