package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ResetElevatorPosition extends Command {
  Elevator elevator;
  double power;
  DigitalInput limitSwitch;

  public ResetElevatorPosition(Elevator elevator, double power, DigitalInput limitSwitch) {
    this.elevator = elevator;
    this.power = power;
    this.limitSwitch = limitSwitch;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  @Override
  public void execute() {
    if (limitSwitch.get()) {
      double pwr = (elevator.getElevatorPosition() > 10) ? power : 0.25 * power;
      elevator.setMotors(-pwr, pwr);
    } else {
      elevator.stop();
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
