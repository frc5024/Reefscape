package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TurnToApriltagCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 3;

    private double pidOutput = 0;

    private PIDController pidController;

    public TurnToApriltagCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;
        this.pidController = new PIDController(0.005, 0.00, 0.0006);

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        pidController.reset();
    }

    @Override
    public void execute() {

        // checks to see if april tag seen is required tag
        if (limelight.getAprilTagID() == targetID) {
            double x = limelight.getX(); // Get the X offset from the target

            if (Math.abs(x) > 0.6) { // Tolerance level
                pidOutput = pidController.calculate(x, 0); // 0 changes offset
                pidOutput = pidOutput * 1;
            } else {
                pidOutput = 0;
            }

            // rotates to face the apriltag while leaving the other movement to user input
            swerveDrive.visionRotationVal(pidOutput, true);

        } else {
            swerveDrive.visionRotationVal(0, false);
        }

        // System.out.println(swerveDrive.getGyroYaw());
        // System.out.println(swerveDrive.getHeading());
        // Rotation2d test = swerveDrive.getHeading();

        // get yaw from LL and yaw from gyro.
        // gyro yaw = LL yaw +/- Degree from wanted gyro 0 (idk if + or - yet)
        // if gyro yaw doesnt = desired gyro 0
        // rest gyro using difference in desired and actual to rest to proper 0
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.visionRotationVal(0, false);
    }
}
