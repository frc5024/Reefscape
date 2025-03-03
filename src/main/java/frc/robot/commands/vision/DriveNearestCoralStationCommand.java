package frc.robot.commands.vision;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.AllianceFlipUtil;

/**
 * Drives to nearest coral station based on pathplanner path
 */
public class DriveNearestCoralStationCommand extends Command {
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private Command commandGroup;
    private Command followPathCommand;

    /**
     * 
     */
    public DriveNearestCoralStationCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (this.commandGroup != null)
            this.commandGroup.cancel();
        this.swerveDriveSubsystem.resetDrivePID();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
    }

    @Override
    public void initialize() {
        // zero drive pid since we are driving closed loop
        this.swerveDriveSubsystem.zeroDrivePID();

        schedulePathCommand();

        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    @Override
    public boolean isFinished() {
        try {

            return followPathCommand.isFinished();

        } catch (Exception e) {
            return true;
        }
    }

    /**
     * 
     */
    public void schedulePathCommand() {
        try {

            PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile(getNearestCoralStation());

            this.followPathCommand = AutoBuilder.pathfindThenFollowPath(pathPlannerPath,
                    frc.robot.autonomous.AutoBuilder.CONSTRAINTS);
            this.commandGroup = Commands.sequence(this.followPathCommand);
            this.commandGroup.schedule();

        } catch (Exception e) {
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
