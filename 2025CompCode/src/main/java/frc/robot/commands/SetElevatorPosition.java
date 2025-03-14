package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class SetElevatorPosition extends Command {
  Elevator elevator;
  double height;
  public SetElevatorPosition(Elevator elevator, double height) {
    this.elevator = elevator;
    this.height = height;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  @Override
  public void execute() {
    elevator.setElevatorPosition(height);
  
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
