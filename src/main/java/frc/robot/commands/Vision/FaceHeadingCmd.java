package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class FaceHeadingCmd extends Command {
    private final Swerve swerveDrive;

    double rotationPidOutput = 0;

    private PIDController rotationPidController;

    // double desiredz = 1.92; // in meters

    boolean xPos = false;
    boolean rotationPos = false;
    // boolean zPos = false;

    // Sets up variables that are used elsewhere to be used here and PIDs
    public FaceHeadingCmd(Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;

        this.rotationPidController = new PIDController(0.008, 0.00, 0.0005);

        addRequirements();
    }

    @Override
    public void initialize() {
        rotationPidController.reset();
    }

    @Override
    public void execute() {
        double robotHeading = swerveDrive.getGyroYaw().getDegrees();

        if (Math.abs(robotHeading) > 0.1) { // Adjust tolerance as needed
            rotationPidOutput = rotationPidController.calculate(robotHeading, 0);
            rotationPidOutput = rotationPidOutput * 1; // Speed multiplier
            rotationPos = false;
        } else {
            rotationPidOutput = 0;
            rotationPos = true;
        }

        swerveDrive.visionRotationVal(rotationPidOutput, true);
    }

    @Override
    public boolean isFinished() {
        return false; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.visionRotationVal(0, false);
    }
}

// potential upgrade; once yaw hits within 0 degree range 3 times in a row
// ignore all inputs until degree has reached 3 degrees 3 times in a row