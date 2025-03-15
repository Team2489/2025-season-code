package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorReefPositions;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MecanumDriveTrain;

public class A_RightL4 extends SequentialCommandGroup {
  public A_RightL4(MecanumDriveTrain driveTrain, CoralIntake coralIntake, Elevator elevator, double intakeOutPower) {
    addCommands(
      new A_DriveMec(driveTrain, 0.55, 0.7, 0).withTimeout(1)
    //  new ScoreLevelHeight(coralIntake, elevator, ElevatorReefPositions.L4.height, intakeOutPower)
      );
      //driving backwards to the coral station
      //intake coral for like 3 seconds
      // go back to reef to score another like L4
  }
}
