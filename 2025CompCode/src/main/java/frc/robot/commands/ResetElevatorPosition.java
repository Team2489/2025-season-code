package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;


public class ResetElevatorPosition extends Command {
  Elevator elevator;
  DigitalInput limitSwitch;
  double power;

  public ResetElevatorPosition(Elevator elevator, DigitalInput limitSwitch, double power) {
    this.elevator = elevator;
    this.limitSwitch = limitSwitch;
    this.power = power;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setMotors(0, 0);
  }

  @Override
  public void execute() {
    double elevatorPosition = elevator.getElevatorPosition() * Constants.kCountsPerRev / Constants.kCountsPerInch;
    elevator.setMotors(power, -power);
    if (elevatorPosition <= 1) {
      elevator.setMotors(elevatorPosition, -elevatorPosition);
    }
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
