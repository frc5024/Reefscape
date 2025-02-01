package frc.robot.commands.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class TestCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    double targetID = 5;

    public TestCmd(Limelight limelight, Swerve swerveDrive) {
        this.limelight = limelight;
        this.swerveDrive = swerveDrive;

        addRequirements(limelight);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (limelight.getAprilTagID() == targetID) {
            Pose3d botPose = LimelightHelpers.getBotPose3d_wpiBlue("");
            // LimelightHelpers.

        }
    }

    @Override
    public boolean isFinished() {
        return false; // Stop when aligned
    }

    @Override
    public void end(boolean interrupted) {
    }
}