package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ResetElevatorPosition extends Command {
  Elevator elevator;
  DigitalInput limitSwitch;

  public ResetElevatorPosition(Elevator elevator, DigitalInput limitSwitch) {
    this.elevator = elevator;
    this.limitSwitch = limitSwitch;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  @Override
  public void execute() {
    elevator.setMotors(0.1, -0.1);
    if (limitSwitch.get()) {
      elevator.handleLimitSwitch();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
