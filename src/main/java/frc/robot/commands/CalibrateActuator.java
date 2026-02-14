package frc.robot.commands;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CalibrateActuator extends Command {
    private final Servo left;
    private final Servo right;
    private final CommandXboxController controller;
    private int currentMicros = 1500;

    public CalibrateActuator(Servo left, Servo right, CommandXboxController controller) {
        this.left = left;
        this.right = right;
        this.controller = controller;
    }

    @Override
    public void execute() {
      int pov = controller.getHID().getPOV();
      if (pov == 0) currentMicros += 10;
      else if (pov == 180) currentMicros -= 10;
      left.setPulseTimeMicroseconds(currentMicros);
      right.setPulseTimeMicroseconds(currentMicros);
      System.out.println("Dual PWM: " + currentMicros + "us");
  }
}