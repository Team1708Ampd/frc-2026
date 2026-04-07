// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualShoot extends Command {

  DoubleSupplier power;
  private final Debouncer m_aimDebouncer = new Debouncer(0.05, Debouncer.DebounceType.kBoth);
  private double m_startTime;

  private boolean toSpeed = false;
  private int atSpeed = 0;

  CommandSwerveDrivetrain drivetrain;

  /** Creates a new ManualShoot. */
  public ManualShoot(DoubleSupplier power, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.power = power;
    this.drivetrain = drivetrain;
    addRequirements(Robot.shooterSub, Robot.cameraSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DISTANCE: " + Robot.cameraSub.getDistance3d(drivetrain));
    System.out.println("POWER: " + power.getAsDouble());
    m_startTime = Timer.getFPGATimestamp();
    atSpeed = 0;
    toSpeed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("DISTANCE: " + Robot.cameraSub.getDistance3d(drivetrain));
    System.out.println("POWER: " + power.getAsDouble());
    Robot.shooterSub.runShooter(power.getAsDouble());
    boolean shooterReady = Robot.shooterSub.isShooterReady(power.getAsDouble() / 60);     
    
    double time = Timer.getFPGATimestamp() - m_startTime;
    double frequency = 2.0; // Oscillations per second
    double speed = Math.sin(time * 2 * Math.PI * frequency) * 0.35;

    if (!toSpeed || atSpeed < 25) {
      toSpeed = shooterReady;
      if (toSpeed) {
        atSpeed++;
      }
    }

    if(toSpeed && atSpeed >= 3) {
      Robot.shooterSub.runProgressiveFeeders(power.getAsDouble() / 60);
      Robot.intakeSub.setIntakePower(1);
      Robot.intakeSub.setHopperPower(1);
      Robot.intakeSub.setWristPower(speed);
    } else {
      Robot.shooterSub.stopFeeders();
      Robot.intakeSub.setHopperPower(0);
      Robot.intakeSub.setWristPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooterSub.runShooter(0);
    Robot.shooterSub.stopFeeders();
    Robot.intakeSub.setIntakePower(0);
    Robot.intakeSub.setHopperPower(0);
    Robot.intakeSub.setWristPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
