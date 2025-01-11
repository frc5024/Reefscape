package frc.robot.autonomous;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.FeedForwardCharacterizationCommand;
import frc.robot.commands.WheelRadiusCharacterizationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.utils.LocalADStarAK;

/**
 * 
 */
public class AutoBuilder extends com.pathplanner.lib.auto.AutoBuilder {
    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    TunerConstants.FrontLeft.WheelRadius,
                    TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                    WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1)
                            .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                    TunerConstants.FrontLeft.SlipCurrent,
                    1),
            SwerveDriveSubsystem.getModuleTranslations());

    /* Subsystems */
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    /* Autonomous Chooser */
    private LoggedDashboardChooser<Command> autonomousChooser;

    /**
     * 
     */
    public AutoBuilder(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    /**
     * 
     */
    public void configureAutonomous() {
        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this.swerveDriveSubsystem::getPose,
                this.swerveDriveSubsystem::setPose,
                this.swerveDriveSubsystem::getChassisSpeeds,
                this.swerveDriveSubsystem::runVelocity,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
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

        if (Constants.TUNING_MODE) {
            this.autonomousChooser.addOption("Drive Characterization",
                    FeedForwardCharacterizationCommand.get(this.swerveDriveSubsystem));
            this.autonomousChooser.addOption("Wheel Radius Characterization",
                    WheelRadiusCharacterizationCommand.get(this.swerveDriveSubsystem));
        }
    }

    /**
     * 
     */
    public LoggedDashboardChooser<Command> getAutonomousChooser() {
        return this.autonomousChooser;
    }
}
