package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MecanumDriveTrain;

public class MoveLeft extends SequentialCommandGroup {
  public MoveLeft(MecanumDriveTrain driveTrain) {
    addCommands(
      new A_DriveMec(driveTrain, 0, 0.3, 0).withTimeout(0.5)
    //  new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L4.height, intakeOutPower)
      );
      //driving backwards to the coral station
      //intake coral for like 3 seconds
      // go back to reef to score another like L4
  }
}
