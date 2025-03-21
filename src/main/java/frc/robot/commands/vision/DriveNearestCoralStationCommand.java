package frc.robot.commands.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * Drives to nearest coral station based on pathplanner path
 */
public class DriveNearestCoralStationCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command followPathCommand;

    /**
     * 
     */
    public DriveNearestCoralStationCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(this.swerveDriveSubsystem);
    }

    @Override
    public void end(boolean interrupted) {
        if (this.followPathCommand != null) {
            this.followPathCommand.end(interrupted);
            Logger.recordOutput("Commands/Active Command", "");
        }

        this.swerveDriveSubsystem.resetDrivePID();

    }

    @Override
    public void execute() {
        if (this.followPathCommand != null) {
            this.followPathCommand.execute();
        }
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop
        this.swerveDriveSubsystem.zeroDrivePID();

        this.followPathCommand = getFollowPathCommand();

        if (this.followPathCommand != null) {
            this.followPathCommand.initialize();
            Logger.recordOutput("Commands/Active Command", this.getName());
        }
    }

    @Override
    public boolean isFinished() {
        return this.followPathCommand != null ? this.followPathCommand.isFinished() : true;
    }

    /**
     * 
     */
    private Command getFollowPathCommand() {
        try {
            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestCoralStation());

            return AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * 
     */
    private String getNearestCoralStation() {
        Pose2d currentPose = this.swerveDriveSubsystem.getPose();
        Pose2d leftStationPose = AllianceFlipUtil.apply(FieldConstants.CORAL_STATION_POSES[0]);
        Pose2d rightStationPose = AllianceFlipUtil.apply(FieldConstants.CORAL_STATION_POSES[1]);

        double leftStationDistance = Math.hypot(currentPose.getX() - leftStationPose.getX(),
                currentPose.getY() - leftStationPose.getY());
        double rightStationDistance = Math.hypot(currentPose.getX() - rightStationPose.getX(),
                currentPose.getY() - rightStationPose.getY());

        if (AllianceFlipUtil.shouldFlip()) {
            return leftStationDistance < rightStationDistance ? "DriveRightStation - Path" : "DriveLeftStation - Path";
        } else {
            return leftStationDistance < rightStationDistance ? "DriveLeftStation - Path" : "DriveRightStation - Path";
        }
    }
}
