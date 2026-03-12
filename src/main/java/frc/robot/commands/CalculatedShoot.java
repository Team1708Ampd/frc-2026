// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CalculatedShoot extends Command {
  /** Creates a new CalculatedShoot. */
  public CalculatedShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooterSub, Robot.intakeSub, Robot.cameraSub);
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
    System.out.println("RPM: " + targetRPM.getAsDouble()); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = Robot.cameraSub.getDistance();
    hoodPos = Robot.shooterSub.getHoodPosition(distance);
    targetRPM = () -> Robot.shooterSub.calculateTargetRPS(distance) * 60;

    Robot.shooterSub.runShooter(() -> targetRPM.getAsDouble());
    boolean rawReady = Robot.shooterSub.isShooterReady(targetRPM);
    
    // The debouncer "smooths" the rawReady signal
    boolean smoothReady = debouncer.calculate(rawReady);

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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
