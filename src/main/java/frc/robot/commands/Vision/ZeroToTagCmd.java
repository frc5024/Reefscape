package frc.robot.commands.Vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class ZeroToTagCmd extends Command {
    private final Limelight limelight;
    private final Swerve swerveDrive;

    private final double targetID = 5;

    public ZeroToTagCmd(Limelight limelight, Swerve swerveDrive) {
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
            double[] botPose = LimelightHelpers.getTargetPose_CameraSpace("");

            double yawDeg = botPose[4];

            swerveDrive.zeroHeadingWithOffset(yawDeg);
        }
        cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
