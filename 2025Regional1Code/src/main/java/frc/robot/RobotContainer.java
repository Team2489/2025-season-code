// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CenterAuton;
import frc.robot.subsystems.MecanumDriveTrain;
import frc.robot.commands.DriveMecanumCartesian;

public class RobotContainer {
  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);

  MecanumDriveTrain mDrive = new MecanumDriveTrain();

  SendableChooser<Command> chooser = new SendableChooser<>();

  CenterAuton centerAuton = new CenterAuton(mDrive, 0, 0, 0);


  public RobotContainer() {
    configureBindings();
    mDrive.setDefaultCommand(new DriveMecanumCartesian(mDrive, -xboxController.getLeftY(), -xboxController.getLeftX(), -xboxController.getRightX()));
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
