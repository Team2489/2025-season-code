// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.MecanumDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.Elevator;

import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;

public class RobotContainer {
  XboxController xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  XboxController xboxController2 = new XboxController(Constants.XBOX_CONTROLLER2_PORT);

  MecanumDrivetrain mDrive = new MecanumDrivetrain();
  CoralIntake coralIntake = new CoralIntake();
  Elevator elevator = new Elevator();
  DigitalInput digitalInput = new DigitalInput(Constants.LINE_BREAKER_PORT);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xboxController, Button.kRightBumper.value).whileTrue(new IntakeIn(coralIntake, 1, digitalInput));
    new JoystickButton(xboxController, Button.kLeftBumper.value).whileTrue(new IntakeOut(coralIntake, 1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
