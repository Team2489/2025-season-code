package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIn extends Command {
  CoralIntake coralIntake;
  double power;
  DigitalInput digitalInput;

  public IntakeIn(CoralIntake coralIntake, double power) {
    this.coralIntake = coralIntake;
    this.power = power;
   // this.digitalInput = digitalInput;
    addRequirements(coralIntake);
  }

  @Override
  public void initialize() {
    coralIntake.intakeRun(0.0);
  }

  @Override
  public void execute() {
    // if (digitalInput.get()) {
    //   coralIntake.stop();
    // } else {
    //   coralIntake.intakeRun(power);
    // }

    coralIntake.intakeRun(-power);
  }

  @Override
  public void end(boolean interrupted) {
    coralIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
