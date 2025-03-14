package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class IntakeOut extends Command {
  CoralIntake coralIntake;
  double power;

  public IntakeOut(CoralIntake coralIntake, double power) {
    this.coralIntake = coralIntake;
    this.power = power;
    addRequirements(coralIntake);
  }

  @Override
  public void initialize() {
    coralIntake.intakeRun(0.0);
  }

  @Override
  public void execute() {
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
