package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.MecanumDriveTrain;

public class A_Center extends SequentialCommandGroup {
  public A_Center(MecanumDriveTrain driveTrain, Elevator elevator, CoralIntake coralIntake, DigitalInput limitSwitch) {
    addCommands(
      new A_DriveMec(driveTrain, 0.15, 0, 0).withTimeout(4.5),
      new SetElevatorPositionL4(elevator, 0.4).withTimeout(4.8),
      new IntakeOut(coralIntake, -0.55).withTimeout(2),
      new ResetElevatorPosition(elevator, 0.6, limitSwitch)
    );
  }
}
