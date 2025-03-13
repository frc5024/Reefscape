package frc.robot;

import java.util.NoSuchElementException;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.RobotConstants;
import frc.robot.containers.BealtovenRobotContainer;
import frc.robot.containers.MapleSimRobotContainer;
import frc.robot.containers.RobotContainer;
import frc.robot.controls.GameData;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.utils.LoggedTracer;
import frc.robot.utils.PhoenixUtil;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    // Variables used in simulation to set robot starting position
    private Alliance alliance = Alliance.Blue;
    private int location = 0;

    private Timer gcTimer = new Timer();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        gcTimer.start();
    }

    @Override
    public void robotInit() {
        // AdvantageKit Record metadata
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (RobotConstants.currentMode) {
            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());

                this.robotContainer = new MapleSimRobotContainer();
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;

            case REAL:
            default:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());

                this.robotContainer = new BealtovenRobotContainer();
                // this.robotContainer = new ReefscapeRobotContainer();
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        // Setup Limelight port forwarding - be sure to match against camera constants
        // https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/best-practices
        for (int port = 5800; port <= 5809; port++) {
            PortForwarder.add(port, "limelight.local", port);
            // PortForwarder.add(port + 10, "limelight-two.local", port);
        }

        DriverStation.silenceJoystickConnectionWarning(true);
        checkDriverStationUpdate();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Refresh all Phoenix signals
        LoggedTracer.reset();
        PhoenixUtil.refreshAll();
        LoggedTracer.record("PhoenixRefresh");

        CommandScheduler.getInstance().run();
        LoggedTracer.record("Commands");

        this.robotContainer.updateAlerts();
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

        // Record cycle time
        LoggedTracer.record("RobotPeriodic");
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        LEDSubsystem.getInstance().solidRed();
    }

    @Override
    public void disabledPeriodic() {
        LEDSubsystem.getInstance().solidRed();
        checkDriverStationUpdate();
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        checkDriverStationUpdate();

        this.autonomousCommand = this.robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (this.autonomousCommand != null) {
            this.autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (this.autonomousCommand != null) {
            this.autonomousCommand.cancel();
        }

        checkDriverStationUpdate();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        Logger.recordOutput("GameData/Game Piece Mode", GameData.getInstance().getGamePieceModeAsString());
        Logger.recordOutput("GameData/Pole Position", GameData.getInstance().getCoralPoleAsString());
        Logger.recordOutput("GameData/Reef Position", GameData.getInstance().getReefStationIndexAsString());
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        this.robotContainer.displaySimFieldToAdvantageScope();
    }

    /**
     * Checks the driverstation alliance. We have have to check repeatedly because
     * we don't know when the
     * driverstation/FMS will connect, and the alliance can change at any time in
     * the shop.
     */
    private void checkDriverStationUpdate() {
        // https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
        try {

            Alliance currentAlliance = DriverStation.getAlliance().get();

            int currentLocation = DriverStation.getLocation().getAsInt();

            // If we have data, and have a new alliance from last time
            if (DriverStation.isDSAttached()
                    && (currentAlliance.compareTo(this.alliance) != 0 || currentLocation != this.location)) {
                this.robotContainer.onAllianceChanged(currentAlliance, currentLocation);
                this.alliance = currentAlliance;
                this.location = currentLocation;
            }

        } catch (NoSuchElementException e) {
        }
    }
}
