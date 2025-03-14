package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MecanumDriveTrain;

public class A_Center extends SequentialCommandGroup {
  public A_Center(MecanumDriveTrain driveTrain) {
    addCommands(
      new A_DriveMec(driveTrain, 0.55, 0.7, 0).withTimeout(0.6)
    );
  }
}
