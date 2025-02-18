package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotConstants;
import frc.robot.modules.gyro.GyroIOInputsAutoLogged;
import frc.robot.modules.gyro.GyroModuleIO;
import frc.robot.modules.swerve.SwerveModule;
import frc.robot.modules.swerve.SwerveModuleConstants;
import frc.robot.modules.swerve.SwerveModuleIO;
import frc.robot.utils.PhoenixOdometryThread;

/**
 * 
 */
public class SwerveDriveSubsystem extends SubsystemBase implements VisionSubsystem.VisionConsumer {
    public static final Lock odometryLock = new ReentrantLock();

    private final GyroModuleIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModule[] swerveModules = new SwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
            SwerveModuleConstants.swerveDriveKinematics,
            rawGyroRotation,
            lastModulePositions, new Pose2d());

    private ChassisSpeeds desiredChassisSpeeds;
    private SwerveModuleState[] desiredModuleStates = new SwerveModuleState[] {};
    private boolean isFieldRelative;
    private boolean isOpenLoop;

    /**
     * 
     */
    public SwerveDriveSubsystem(GyroModuleIO gyroIO, SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO,
            SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        this.swerveModules[0] = new SwerveModule(flModuleIO, 0);
        this.swerveModules[1] = new SwerveModule(frModuleIO, 1);
        this.swerveModules[2] = new SwerveModule(blModuleIO, 2);
        this.swerveModules[3] = new SwerveModule(brModuleIO, 3);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        (state) -> Logger.recordOutput("SwerveDrive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

        this.isFieldRelative = true;
        this.isOpenLoop = false;
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        this.poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * 
     */
    public void disableFieldRelative() {
        setIsFieldRelative(false);
    }

    public void enableFieldRelative() {
        setIsFieldRelative(true);
    }

    @Override
    public void periodic() {
        // Set the swerve module states
        if (this.desiredChassisSpeeds != null) {
            this.desiredModuleStates = SwerveModuleConstants.swerveDriveKinematics
                    .toSwerveModuleStates(this.desiredChassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(this.desiredModuleStates, SwerveModuleConstants.maxLinearSpeed);
            for (SwerveModule swerveModule : this.swerveModules) {
                if (this.isOpenLoop) {
                    swerveModule.runVelocity(this.desiredModuleStates[swerveModule.getIndex()]);
                } else {
                    swerveModule.runSetpoint(this.desiredModuleStates[swerveModule.getIndex()]);
                }
            }
        }

        odometryLock.lock(); // Prevents odometry updates while reading data
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.updateInputs();
        }

        this.gyroIO.updateInputs(this.gyroInputs);
        Logger.processInputs("SwerveDrive/Gyro", this.gyroInputs);
        odometryLock.unlock();

        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.periodic();
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : this.swerveModules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps = this.swerveModules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = this.swerveModules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - this.lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                this.lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (this.gyroInputs.connected) {
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = SwerveModuleConstants.swerveDriveKinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && RobotConstants.currentMode != RobotConstants.Mode.SIM);

        Logger.recordOutput("Subsystems/SwerveDrive/IsFieldOriented", this.isFieldRelative);
        Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/Setpoints", this.desiredModuleStates);
        // Logger.recordOutput("Subsystems/SwerveDrive/SwerveChassisSpeeds/Setpoints",
        // discreteSpeeds);
        Logger.recordOutput("Subsystems/SwerveDrive/SwerveStates/SetpointsOptimized", this.desiredModuleStates);

        // Always reset desiredChassisSpeeds to null to prevent latching to the last
        // state (aka motor safety)!!
        this.desiredChassisSpeeds = null;
    }

    /**
     * Used for autonomous driving in AutoBuilder - chassis speeds are robot
     * relative
     */
    public void drive(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
        this.desiredChassisSpeeds = chassisSpeeds;
        this.isOpenLoop = false;
    }

    /**
     * 
     */
    public void drive(double xVelocity, double yVelocity, double rVelocity) {
        drive(xVelocity, yVelocity, rVelocity,
                DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                        ? getRotation().plus(new Rotation2d(Math.PI))
                        : getRotation(),
                false);
    }

    /**
     * 
     */
    public void drive(double xVelocity, double yVelocity, double rVelocity, Rotation2d angle, boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = null;
        xVelocity = xVelocity * SwerveModuleConstants.maxLinearSpeed;
        yVelocity = yVelocity * SwerveModuleConstants.maxLinearSpeed;
        rVelocity = rVelocity * SwerveModuleConstants.maxAngularSpeed;

        if (isFieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rVelocity, angle);
        } else {
            chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rVelocity);
        }

        this.desiredChassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.LOOP_PERIOD_SECS);
        this.isOpenLoop = isOpenLoop;
    }

    /**
     * 
     */
    public void resetPosition(Pose2d pose2d) {
        this.poseEstimator.resetPosition(getRotation(), getModulePositions(), pose2d);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            this.swerveModules[i].runCharacterization(output);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        drive(0.0, 0.0, 0.0);
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will
     * return to their normal orientations the next time a nonzero velocity is
     * requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = SwerveModuleConstants.moduleTranslations[i].getAngle();
        }
        SwerveModuleConstants.swerveDriveKinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "Subsystems/SwerveDrive/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule swerveModule : this.swerveModules) {
            states[swerveModule.getIndex()] = swerveModule.getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = this.swerveModules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "Subsystems/SwerveDrive/SwerveChassisSpeeds/Measured")
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveModuleConstants.swerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = this.swerveModules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native
     * units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += this.swerveModules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Resets the current odometry pose.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * 
     */
    public void resetDrivePID() {
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.resetDrivePID();
        }
    }

    /**
     * 
     */
    public void updateDrivePID(double kP, double kI, double kD) {
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.setDrivePID(kP, kI, kD);
        }
    }

    /**
     * 
     */
    public void updateTurnPID(double kP, double kI, double kD) {
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.setTurnPID(kP, kI, kD);
        }
    }

    /**
     * 
     */
    public void zeroDrivePID() {
        for (SwerveModule swerveModule : this.swerveModules) {
            swerveModule.setDrivePID(0.0, 0.0, 0.0);
        }
    }

    /** Getters and Setters */
    public boolean isFieldRelative() {
        return this.isFieldRelative;
    }

    public void setIsFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
    }
}
