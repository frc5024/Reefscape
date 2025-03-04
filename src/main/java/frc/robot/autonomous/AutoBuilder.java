package frc.robot.autonomous;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.tuning.ElevatorFeedForwardCharacterizationCommand;
import frc.robot.commands.tuning.FeedForwardCharacterizationCommand;
import frc.robot.commands.tuning.WheelRadiusCharacterizationCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LocalADStarAK;

/**
 * 
 */
public class AutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {
    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    /* Constraints */
    public static final PathConstraints CONSTRAINTS = new PathConstraints(
            4.5, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

    // PathPlanner Config Constants
    private static final double ROBOT_MOI = 6.883;
    public static final RobotConfig PP_CONFIG = new RobotConfig(
            RobotConstants.MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    SwerveConstants.cotsDriveConstants.wheelDiameter / 2,
                    SwerveConstants.maxLinearSpeed,
                    RobotConstants.WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(SwerveConstants.cotsDriveConstants.driveGearRatio),
                    120,
                    1),
            SwerveConstants.moduleTranslations);

    /* Autonomous Chooser */
    private LoggedDashboardChooser<Command> autonomousChooser;

    /**
     * 
     */
    public AutoBuilder(SwerveDriveSubsystem swerveDriveSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
    }

    /**
     * 
     */
    public void configureAutonomous() {
        double[] driveXPIDs = frc.robot.Constants.PIDConstants.getDriveXPIDs();
        double[] driveOmegaPIDs = frc.robot.Constants.PIDConstants.getDriveOmegaPIDs();
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this.swerveDriveSubsystem::getPose,
                this.swerveDriveSubsystem::setPose,
                this.swerveDriveSubsystem::getChassisSpeeds,
                this.swerveDriveSubsystem::drive,
                new PPHolonomicDriveController(new PIDConstants(driveXPIDs[0], driveXPIDs[1], driveXPIDs[2]),
                        new PIDConstants(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2])),
                PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this.swerveDriveSubsystem);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });

        // Setup the chooser
        this.autonomousChooser = new LoggedDashboardChooser<Command>("Auto Routine", AutoBuilder.buildAutoChooser());

        if (RobotConstants.TUNING_MODE) {
            this.autonomousChooser.addOption("Drive Characterization",
                    new FeedForwardCharacterizationCommand(this.swerveDriveSubsystem).get());
            this.autonomousChooser.addOption("Wheel Radius Characterization",
                    new WheelRadiusCharacterizationCommand(this.swerveDriveSubsystem).get());
            this.autonomousChooser.addOption("Elevator Characterization",
                    new ElevatorFeedForwardCharacterizationCommand(this.elevatorSubsystem).get());
        }

        // Load Game Autos
        this.autonomousChooser.addOption("Drive Away", new PathPlannerAuto("DriveAway"));
    }

    /**
     * 
     */
    public LoggedDashboardChooser<Command> getAutonomousChooser() {
        return this.autonomousChooser;
    }

    /**
     * 
     */
    public static Command getPathFindingCommand(Pose2d targetPose) {
        Command pathFindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                CONSTRAINTS,
                0.0);

        return pathFindingCommand;
    }
}
