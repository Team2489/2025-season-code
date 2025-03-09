package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class MoveYForTime extends Command {
    private final DriveTrain driveTrain;
    private final double ySpeed; // The speed at which to move along the Y-axis
    private final double timeLimit = 2.0; // Time for how long to move the robot
    private Timer timer;

    public MoveYForTime(DriveTrain driveTrain, double ySpeed) {
        this.driveTrain = driveTrain;
        this.ySpeed = ySpeed;
        addRequirements(driveTrain); // Declare subsystem dependencies
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.reset();
        timer.start(); // Start the timer
    }

    @Override
    public void execute() {
        // Move the robot with the specified ySpeed for the duration of the timer
        driveTrain.driveCartesian(0, ySpeed, 0); // No xSpeed, no zRotation
    }

    @Override
    public boolean isFinished() {
        // End the command after 5 seconds
        return timer.get() >= timeLimit;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends
        driveTrain.stopMotors();
    }
}