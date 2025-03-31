package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.subsystems.Elevator;

public class SetElevatorPositionL4 extends Command {
  Elevator elevator;
  double power;
  public SetElevatorPositionL4(Elevator elevator, double power) {
    this.elevator = elevator;
    this.power = power;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0.022, -0.022);
  }

  @Override
  public void execute() {
    if (elevator.getElevatorPosition() < ElevatorReefPositions.L4.height) {
      double pwr = (((ElevatorReefPositions.L4.height - 10) < elevator.getElevatorPosition()) && ((elevator.getElevatorPosition()) < (ElevatorReefPositions.L4.height))) ? 0.25 * power : power;
      elevator.setMotors(pwr, -pwr);
    } else {
      elevator.stop();
    }
    if (((ElevatorReefPositions.L4.height - 1) <= elevator.getElevatorPosition())) {
      elevator.ledLightsGreen();
    } else {
      elevator.ledLightRed();
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setMotors(0.022, -0.022);
    if (((ElevatorReefPositions.L4.height - 1) <= elevator.getElevatorPosition())) {
      elevator.ledLightsGreen();
    } else {
      elevator.ledLightRed();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
