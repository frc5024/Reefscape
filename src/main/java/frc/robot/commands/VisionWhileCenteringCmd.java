package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class VisionWhileCenteringCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 3;

    private boolean shot = false;
    private boolean newWarm = false;

    double strafePidOutput = 0;
    double rotationPidOutput = 0;
    double translationPidOutput = 0;

    private PIDController strafePidController;
    private PIDController rotationPidController;
    private PIDController translationPidController;

    // double desiredz = 1.92; // in meters

    boolean xPos = false;
    boolean rotationPos = false;
    // boolean zPos = false;

    // Sets up variables that are used elsewhere to be used here and PIDs
    public VisionWhileCenteringCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;

        this.strafePidController = new PIDController(0.3, 0, 0.0007);
        this.translationPidController = new PIDController(0.1, 0, 0.0007);

        this.rotationPidController = new PIDController(0.004, 0.00, 0.0001);

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        strafePidController.reset();
        translationPidController.reset();
        rotationPidController.reset();
    }

    @Override
    public void execute() {

        // checks apriltag in view
        if (limelight.getAprilTagID() == targetID) {
            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");

            double x = limelight.getX(); // Get X offset
            double yawDeg = botPose[4];

            double zDis = botPose[2];

            double atDeg = yawDeg - x;
            double atRad = Math.toRadians(atDeg);
            double atXDis = zDis * (Math.tan(atRad));

            // double zDiff = desiredz - zDis;

            // System.out.println(zDiff);

            // Chceks to see if the desired distance is withing the tolerance and adjusts
            // location
            if (Math.abs(yawDeg) > 1) { // Adjust tolerance as needed
                rotationPidOutput = rotationPidController.calculate(yawDeg / 2, 0);
                rotationPidOutput = rotationPidOutput * 1; // Speed multiplier
                rotationPos = false;
            } else {
                rotationPidOutput = 0;
                rotationPos = true;
            }

            // ~~
            if (Math.abs(atXDis) > 0.05) { // In meters
                strafePidOutput = strafePidController.calculate(atXDis, 0);
                strafePidOutput = -strafePidOutput * 1; // Speed multiplier
                xPos = false;
            } else {
                strafePidOutput = 0;
                xPos = true;
            }

            // moves the robot in proper direction and locks controller input
            swerveDrive.visionStrafeVal(strafePidOutput, true);
            swerveDrive.visionRotationVal(rotationPidOutput, true);
            // swerveDrive.visionTranslationalVal(0, true);

            System.out.println(yawDeg);
        } else {
            swerveDrive.visionTranslationalVal(0, false);
            swerveDrive.visionStrafeVal(0, false);
            swerveDrive.visionRotationVal(0, false);

        }
    }

    @Override
    public boolean isFinished() {
        return false; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
        // Stop all motion
        swerveDrive.visionTranslationalVal(0, false);
        swerveDrive.visionStrafeVal(0, false);
        swerveDrive.visionRotationVal(0, false);
    }
}

// potential upgrade; once yaw hits within 0 degree range 3 times in a row
// ignore all inputs until degree has reached 3 degrees 3 times in a row