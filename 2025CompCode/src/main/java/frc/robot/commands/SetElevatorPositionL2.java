package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.subsystems.Elevator;

public class SetElevatorPositionL2 extends Command {
  Elevator elevator;
  double power;
  public SetElevatorPositionL2(Elevator elevator, double power) {
    this.elevator = elevator;
    this.power = power;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  @Override
  public void execute() {
    if (elevator.getElevatorPosition() < ElevatorReefPositions.L2.height) {
      double pwr = (((ElevatorReefPositions.L2.height - 10) < elevator.getElevatorPosition()) && ((elevator.getElevatorPosition()) < (ElevatorReefPositions.L2.height))) ? 0.25 * power : power;
      elevator.setMotors(pwr, -pwr);
    } else {
      elevator.stop();
    }
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
