package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class FeedAndShoot extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    
    private final PIDController m_turnPid = new PIDController(0.04, 0.0, 0.002);
    private final Debouncer m_aimDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
    
    // 0.1s debouncer ensures we are stable for 5 cycles (20ms * 5 = 100ms)
    private final Debouncer m_speedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private double m_startTime;
    private double m_targetRPS;

    private boolean toSpeed = false;
    private int atSpeed = 0; // Keeping for logic compatibility

    public FeedAndShoot(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(Robot.shooterSub, Robot.cameraSub, drivetrain);
        m_turnPid.setTolerance(4.0); 
    }

    @Override
    public void initialize() {
        m_turnPid.reset();
        double distance = Robot.cameraSub.getDistance();
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
        Robot.shooterSub.runShooter(m_targetRPS * 60);
        m_startTime = Timer.getFPGATimestamp();

        toSpeed = false;
        atSpeed = 0;
    }

    @Override
    public void execute() {
            System.out.println("DISTANCE: " + Robot.cameraSub.getDistance());
            System.out.println("POWER: " + m_targetRPS * 60);
        // 1. UPDATE TARGETS
        double distance = Robot.cameraSub.getDistance();
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
        
        // 2. RUN SHOOTER
        Robot.shooterSub.runShooter(m_targetRPS * 60);

        // 3. AUTO-ALIGN DRIVETRAIN
        boolean hasTarget = Robot.cameraSub.hasTarget();
        double tx = Robot.cameraSub.getTX();
        boolean isAimed = false;

        double time = Timer.getFPGATimestamp() - m_startTime;
        double frequency = 1.5; 
        double speed = Math.sin(time * 2 * Math.PI * frequency) * 0.3;

        if (hasTarget) {
            double rotationOutput = m_turnPid.calculate(tx, 0);
            isAimed = m_turnPid.atSetpoint();
            drivetrain.setControl(request.withRotationalRate(rotationOutput * 5.0)); 
        } else {
            drivetrain.setControl(request.withRotationalRate(0));
        }

        // 4. SPEED LATCH LOGIC
        // We only want to latch 'true' once we've been at speed for 5 cycles
        boolean shooterReady = Robot.shooterSub.isShooterReady(m_targetRPS); 
        
        if (!toSpeed) {
            // Calculate returns true only after 5 consecutive cycles of shooterReady
            toSpeed = m_speedDebouncer.calculate(shooterReady);
            if (toSpeed) {
                atSpeed = 5; // Latch established
            }
        }

        boolean hoodReady = true; // Placeholder as requested
        boolean readyToFire = m_aimDebouncer.calculate(isAimed && toSpeed && hoodReady);

        // 5. FEEDER CONTROL
        if (readyToFire) {
            Robot.shooterSub.runProgressiveFeeders(m_targetRPS);
            Robot.intakeSub.setIntakePower(1);
            Robot.intakeSub.setHopperPower(1);
            Robot.intakeSub.setWristPower(speed);
        } else {
            Robot.shooterSub.stopFeeders();
            Robot.intakeSub.setHopperPower(0);
            Robot.intakeSub.setIntakePower(0);
            Robot.intakeSub.setWristPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Robot.shooterSub.runShooter(0);
        Robot.shooterSub.stopFeeders();
        Robot.intakeSub.setIntakePower(0);
        Robot.intakeSub.setHopperPower(0);
        Robot.intakeSub.setWristPower(0);
        drivetrain.setControl(request.withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}