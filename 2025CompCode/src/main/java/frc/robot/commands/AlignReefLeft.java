// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.MecanumDriveTrain;

public class AlignReefLeft extends SequentialCommandGroup {
  public AlignReefLeft(MecanumDriveTrain mDrive, LimeLight limeLight) {
    addCommands(
      new AlignToAprilTag(mDrive, limeLight),
      new MoveLeft(mDrive)
    );
  }
}
