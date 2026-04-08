package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class FeedAndShoot extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();
    
    private final PIDController m_turnPid = new PIDController(0.04, 0.0, 0.002);
    private final Debouncer m_aimDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
    private final Debouncer m_speedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    // GOAL COORDINATES (Meters)
    private final Translation2d BLUE_GOAL = Constants.BLUE_GOAL_CENTER;
    private final Translation2d RED_GOAL = Constants.RED_GOAL_CENTER;

    private double m_startTime;
    private double m_targetRPS;
    private boolean toSpeed = false;

    public FeedAndShoot(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(Robot.shooterSub, Robot.cameraSub, drivetrain);
        m_turnPid.setTolerance(2.0); 
        // Enable continuous input so the PID understands -180 and 180 are the same point
        m_turnPid.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        m_turnPid.reset();
        double distance = Robot.cameraSub.getDistance3d(drivetrain);
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
        Robot.shooterSub.runShooter(m_targetRPS * 60);
        m_startTime = Timer.getFPGATimestamp();
        toSpeed = false;
    }

    @Override
    public void execute() {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        // if (mt1 == null || mt1.tagCount == 0) {
        //     drivetrain.setControl(request.withRotationalRate(0));
        //     Robot.shooterSub.stopFeeders();
        //     Robot.intakeSub.setHopperPower(0);
        //     Robot.intakeSub.setIntakePower(0);
        //     Robot.intakeSub.setWristPower(0);
        //     return;
        // }

        // 1. UPDATE TARGETS & SHOOTER
        double distance = Robot.cameraSub.getDistance3d(drivetrain);
        System.out.println("DISTANCE: " + distance);
        m_targetRPS = Robot.shooterSub.calculateTargetRPS(distance);
        Robot.shooterSub.runShooter(m_targetRPS * 60);

        var currentPose = drivetrain.getState().Pose;
        
        Rotation2d targetHeading = Robot.cameraSub.getAngleToGoal(drivetrain);

        double rotationOutput = m_turnPid.calculate(
            currentPose.getRotation().getDegrees(),
            targetHeading.getDegrees()
        );
        boolean isAimed = m_turnPid.atSetpoint();

        // Apply rotation to Swerve
        drivetrain.setControl(request.withRotationalRate(rotationOutput * 3.0));

        // 3. SHOOTER SPEED LATCH and get wrist agitation speed
        double time = Timer.getFPGATimestamp() - m_startTime;
        double wristSpeed = Math.sin(time * 2 * Math.PI * 1.5) * 0.3;
        boolean shooterReady = Robot.shooterSub.isShooterReady(m_targetRPS); 
        if (!toSpeed) {
            toSpeed = m_speedDebouncer.calculate(shooterReady);
        }

        // 4. FINAL READINESS CHECK
        boolean hoodReady = true; 
        boolean readyToFire = m_aimDebouncer.calculate(toSpeed);
        // boolean readyToFire = true;

        // 5. FEEDER CONTROL
        // if (readyToFire) {
            Robot.shooterSub.runProgressiveFeeders(m_targetRPS);
            Robot.intakeSub.setIntakePower(1);
            Robot.intakeSub.setHopperPower(1);
            Robot.intakeSub.setWristPower(wristSpeed);
        // } else {
            Robot.shooterSub.stopFeeders();
            Robot.intakeSub.setHopperPower(0);
            Robot.intakeSub.setIntakePower(0);
            Robot.intakeSub.setWristPower(0);
        // }
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