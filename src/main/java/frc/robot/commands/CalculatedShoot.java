// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalculatedShoot extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Constants constants = new Constants();
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    private final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric();

    private final PIDController m_pid = new PIDController(0.03, 0.0, 0.001);

    private final double kSearchSpeed = 3;
  /** Creates a new CalculatedShoot. */
  public CalculatedShoot(CommandSwerveDrivetrain drivetrain) {
      this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooterSub, Robot.intakeSub, Robot.cameraSub, drivetrain);
    m_pid.setTolerance(3);
  }

    DoubleSupplier targetRPM;
    private final Debouncer debouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
    double distance;
    double hoodPos;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = Robot.cameraSub.getDistance();
    hoodPos = Robot.shooterSub.getHoodPosition(distance);
    targetRPM = () -> Robot.shooterSub.calculateTargetRPS(distance) * 60;

    System.out.println("DISTANCE: " + distance);
    System.out.println("HOOD POSITION: " + hoodPos);
    System.out.println("RPM: " + targetRPM); 
    
    m_pid.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = Robot.cameraSub.getDistance();
    hoodPos = Robot.shooterSub.getHoodPosition(distance);
    targetRPM = () -> Robot.shooterSub.calculateTargetRPS(distance) * 60;

    // 1. Always spool the shooter to the calculated target
    Robot.shooterSub.runShooter(() -> targetRPM.getAsDouble());
    boolean rawReady = Robot.shooterSub.isShooterReady(targetRPM);

    boolean hasTarget = table.getEntry("tv").getDouble(0) == 1.0;
    double tx = table.getEntry("tx").getDouble(0);
    
    boolean isAimed = false;

    if (hasTarget) {
      double rotationOutput = m_pid.calculate(tx, 0);
      isAimed = m_pid.atSetpoint(); // Check if we are within the 0.5 degree tolerance

      if (isAimed) {
        // If we reached the target, stop rotating to stay stable for the shot
        drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      } else {
        // If not aimed, keep rotating
        double maxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        double rotationSpeed = rotationOutput * maxAngularRate;

        drivetrain.setControl(
          request.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed));
      }
    } else {
      // If we lose the target, stop moving for safety
      drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }

    // 2. The SHOOTING Logic
    // Only fire if the shooter is at the right RPM AND the drivetrain is aimed
    boolean smoothReady = debouncer.calculate(rawReady && isAimed);

    if (smoothReady) {
      Robot.intakeSub.setAllIntakes(0.7);
    } else {
      Robot.intakeSub.setAllIntakes(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSub.runShooter(() -> 0);
    Robot.intakeSub.setAllIntakes(0);
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
