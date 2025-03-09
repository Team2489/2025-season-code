// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A_LeftL4 extends SequentialCommandGroup {
  /** Creates a new A_LeftL4. */
  public A_LeftL4(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new A_DriveMec(driveTrain, 0.55, 0.7, 0).withTimeout(1)
      //some sort of sequential command for scoring L4
      //driving backwards to the coral station
      //intake coral for like 3 seconds
      // go back to reef to score another like L4
    );
  }
}
