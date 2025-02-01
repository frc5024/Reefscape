package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class PoseEstimator extends SubsystemBase {

    private SwerveDrivePoseEstimator m_poseEstimator;
    private Swerve swerveDrive;

    private PoseEstimator() {

    }

    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation("limelight",
                m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        boolean doRejectUpdate = false;

        if (Math.abs(swerveDrive.gyro.getRate()) > 720) { // if our angular velocity is greater than 720 degrees per
                                                          // second, ignore vision updates
            doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) { // If no tags seen dont update
            doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }

    }

}
