package frc.robot.commands.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class DistanceTestingCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 5;

    double translationPidOutput = 0;
    private PIDController translationPidController;

    double desiredz = 0.97; // in meters

    boolean zPos = false;

    // Sets up variables that are used elsewhere to be used here and PIDs
    public DistanceTestingCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;

        this.translationPidController = new PIDController(0.7, 0, 0.05);

        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        translationPidController.reset();
    }

    @Override
    public void execute() {

        if (limelight.getAprilTagID() == targetID) {
            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");
            double robotHeading = normalizeHeading(SmartDashboard.getNumber("Heading", 0)); // 0-360 range
            double Dis = botPose[2];

            double zDis = Dis * Math.cos(Math.toRadians(robotHeading));

            double zDiff = desiredz - zDis;

            System.out.println(zDiff);

            if (Math.abs(zDiff) > 0.01) { // In meters
                translationPidOutput = translationPidController.calculate(zDiff, 0);
                translationPidOutput = translationPidOutput * 0.5; // Speed multiplier
                zPos = false;
            } else {
                translationPidOutput = 0;
                zPos = true;
            }

            swerveDrive.visionTranslationalVal(translationPidOutput, true);
        } else {
            swerveDrive.visionTranslationalVal(0, false);
            swerveDrive.visionStrafeVal(0, false);
            swerveDrive.visionRotationVal(0, false);

        }
    }

    private double normalizeHeading(double heading) {
        return (heading >= 0) ? heading : (360 + heading);
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