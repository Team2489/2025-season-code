package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class ScoreLevelHeight extends SequentialCommandGroup {
  public ScoreLevelHeight(CoralIntake coralIntake, Elevator elevator, double height, double intakePower, double elevatorPower, DigitalInput limitSwitch) {
    addCommands(
      new SetElevatorPosition(elevator, height),
      new IntakeOut(coralIntake, intakePower),
      new ResetElevatorPosition(elevator, limitSwitch, elevatorPower)
    );
  }
}
