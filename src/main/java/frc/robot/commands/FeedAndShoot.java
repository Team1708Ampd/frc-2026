package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class FeedAndShoot extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    
    // Tuning: Adjusted P for snappier AprilTag alignment
    private final PIDController m_turnPid = new PIDController(0.04, 0.0, 0.002);
    private final Debouncer m_aimDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);

    private double m_targetRPS;

    public FeedAndShoot(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Require the shooter, camera, and drive. 
        // Note: Intake is usually separate, but included here for the "Feeder" control.
        addRequirements(Robot.shooterSub, Robot.cameraSub, drivetrain);
        
        m_turnPid.setTolerance(4.0); // Degree tolerance for Limelight/AprilTag
    }

    @Override
    public void initialize() {
        m_turnPid.reset();
        // Initial distance check to prevent a 0-value jump
        double distance = Robot.cameraSub.getDistance();
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
    }

    @Override
    public void execute() {
        // 1. UPDATE TARGETS
        double distance = Robot.cameraSub.getDistance();
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
        
        // 2. RUN SHOOTER & HOOD
        // Shooter spools up immediately; Hood moves to zone based on distance
        Robot.shooterSub.runShooter(m_targetRPS * 60);

        // 3. AUTO-ALIGN DRIVETRAIN
        boolean hasTarget = Robot.cameraSub.hasTarget(); // Assuming cameraSub has a helper
        double tx = Robot.cameraSub.getTX();
        boolean isAimed = false;

        if (hasTarget) {
            double rotationOutput = m_turnPid.calculate(tx, 0);
            isAimed = m_turnPid.atSetpoint();

            // Apply rotation, keep X and Y at 0 for a stable stationary shot
            drivetrain.setControl(request.withRotationalRate(rotationOutput * 5.0)); 
        } else {
            drivetrain.setControl(request.withRotationalRate(0));
        }

        // 4. SMART FEEDER GATE (The "Rule of Thirds" Spray)
        // We only feed if: Aimed + Hood Ready + Shooter at RPM
        boolean shooterReady = Robot.shooterSub.isShooterReady(m_targetRPS);        
        // boolean hoodReady = Robot.shooterSub.isHoodAtPosition();
        boolean readyToFire = m_aimDebouncer.calculate(isAimed && shooterReady);
        // boolean readyToFire = m_aimDebouncer.calculate(isAimed && shooterReady && hoodReady);

        if (readyToFire) {
            // New 3-motor logic: Hopper, Linked Stage 1/2, and Stage 3
            Robot.shooterSub.runProgressiveFeeders(m_targetRPS);
            Robot.intakeSub.setIntakePower(1);
            Robot.intakeSub.setHopperPower(5);
        } else {
            Robot.shooterSub.stopFeeders();
            Robot.intakeSub.setHopperPower(0);
            Robot.intakeSub.setIntakePower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooterSub.runShooter(0);
        Robot.shooterSub.stopFeeders();
        Robot.intakeSub.setIntakePower(0);
        Robot.intakeSub.setHopperPower(0);
        drivetrain.setControl(request.withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        // Usually returns false so it runs while button is held
        return false;
    }
}