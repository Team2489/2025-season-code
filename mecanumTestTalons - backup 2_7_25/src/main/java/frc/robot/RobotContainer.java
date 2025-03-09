// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.A_Center;
import frc.robot.commands.A_RightL4;
import frc.robot.commands.A_LeftL4;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.AprilTagNewAlignment;
import frc.robot.commands.MoveYForTime;
import frc.robot.commands.driveMecanum;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);
  DriveTrain dDrive = new DriveTrain();
  LimeLight limeLight = new LimeLight();

  SendableChooser<Command> chooser = new SendableChooser<>();

  A_Center a_Center = new A_Center(dDrive);
  A_RightL4 a_RightL4 = new A_RightL4(dDrive);
  A_LeftL4 a_Left4 = new A_LeftL4(dDrive);

  public RobotContainer() {
    configureBindings();

    dDrive.setDefaultCommand(new driveMecanum(dDrive, xboxController));
    chooser.setDefaultOption("Auton Center", a_Center);
    SmartDashboard.putData(chooser);
  }

  private void configureBindings() {
    new JoystickButton(xboxController, XboxController.Button.kB.value).whileTrue(new AlignToAprilTag(dDrive, limeLight));
    new JoystickButton(xboxController, XboxController.Button.kX.value).whileTrue(new MoveYForTime(dDrive, 0.5));
    // new JoystickButton(xboxController, XboxController.Button.kY.value).whileTrue(new AprilTagNewAlignment(dDrive, limeLight));
  }

  public Command getAutonomousCommand() {
        return chooser.getSelected();
  }
}
