package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

public class ScoreReef extends SequentialCommandGroup {
  public ScoreReef(Elevator elevator, CoralIntake coralIntake, double intakePower, DigitalInput limitSwitch) {
    addCommands(
      new IntakeOut(coralIntake, intakePower)
      //new ResetElevatorPosition(elevator, limitSwitch)
    );
  }
}
