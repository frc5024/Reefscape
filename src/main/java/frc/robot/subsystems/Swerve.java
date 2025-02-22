package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public double speedModifier;
    public AHRS gyro;

    double translationVal = 0;
    double strafeVal = 0;
    double rotationVal = 0;

    boolean lockRotation = false;
    boolean lockStrafe = false;
    boolean lockTranslation = false;

    boolean fieldRelative = true;

    public boolean isSlowMode = false;

    public final double scaleValue = 3600.0 / 3831.020004272461;

    private static Swerve mInstance;
    private Elevator elevatorSubsystem;

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve();
        }
        return mInstance;
    }

    private Swerve() {
        LimelightHelpers.setFiducial3DOffset("", 0, 0, 0);

        gyro = new AHRS(SPI.Port.kMXP);
        gyro.zeroYaw();

        speedModifier = 1;

        elevatorSubsystem = Elevator.getInstance();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT
                                                                      // RELATIVE ChassisSpeeds. Also optionally outputs
                                                                      // individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                                // holonomic drive trains
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    // takes priority over controller input - call lockController to remove
    // controller access to movement value
    public void visionTranslationalVal(double translationSpeed, boolean lockController) {
        if (lockController) {
            translationVal = translationSpeed;
        }
        lockTranslation = lockController;
    }

    public void visionRotationVal(double rotationSpeed, boolean lockController) {
        if (lockController) {
            rotationVal = rotationSpeed;
        }
        lockRotation = lockController;
    }

    public void visionStrafeVal(double strafeSpeed, boolean lockController) {
        if (lockController) {
            strafeVal = strafeSpeed;
        }
        lockStrafe = lockController;
    }

    // Controller called values - only call when lockController isnt true
    public void controllerTranslationalVal(double translationOutput) {
        if (!lockTranslation) {
            translationVal = translationOutput;
        }
    }

    public void controllerRotationVal(double rotationOutput) {
        if (!lockRotation) {
            rotationVal = rotationOutput;
        }
    }

    public void controllerStrafeVal(double strafeOutput) {
        if (!lockStrafe) {
            strafeVal = strafeOutput;
        }
    }

    public void setFieldRelative(boolean isFieldRelative) {
        fieldRelative = isFieldRelative;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    public void setSpeedModifier() {
        speedModifier = speedModifier - elevatorSubsystem.getElevatorPercent();

        if (isSlowMode) {
            speedModifier = 0.3 * speedModifier;
        }
    }

    /**
     * 
     */
    public void drive(boolean isOpenLoop) {
        ChassisSpeeds chassisSpeeds = null;

        setSpeedModifier();

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translationVal * Constants.Swerve.maxSpeed * speedModifier,
                    strafeVal * Constants.Swerve.maxSpeed * speedModifier,
                    rotationVal * Constants.Swerve.maxAngularVelocity * speedModifier, getGyroYaw());
        } else {
            System.out.println("translationVal");
            chassisSpeeds = new ChassisSpeeds(translationVal * Constants.Swerve.maxSpeed * speedModifier,
                    strafeVal * Constants.Swerve.maxSpeed * speedModifier,
                    rotationVal * Constants.Swerve.maxAngularVelocity * speedModifier);
        }

        drive(chassisSpeeds, isOpenLoop);
    }

    /**
     * 
     */
    public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }

    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    // public void zeroHeading() {
    // swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
    // new Pose2d(getPose().getTranslation(), new Rotation2d()));
    // }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getGyroYaw() {
        // return Rotation2d.fromDegrees(-gyro.getYaw());
        // return Rotation2d.fromDegrees(gyro.getAngle() * scaleValue);
        double angle = (gyro.getAngle() * scaleValue) % 360.0;
        if (angle > 180) {
            angle = angle - 360;
        }
        return Rotation2d.fromDegrees(-angle);
    }

    public Rotation2d getGyroYaw360() {
        double angle = (gyro.getAngle() * scaleValue) % 360.0;
        return Rotation2d.fromDegrees(-angle);
    }

    public void zeroHeadingWithOffset(double degOffset) {
        swerveOdometry.resetPosition(getGyroYawWithOffset(degOffset), getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYawWithOffset(double degOffset) {
        // negative to fix field relative
        return Rotation2d.fromDegrees((-gyro.getYaw()) - degOffset);

    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
        setModuleStates(states);
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for (SwerveModule mod : mSwerveMods) {
            mod.update();
        }

        SmartDashboard.putNumber("Pose X", getPose().getX());
        SmartDashboard.putNumber("Pose Y", getPose().getY());
        SmartDashboard.putNumber("Gyro", -getGyroYaw().getDegrees());
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());

        SmartDashboard.putNumber("RawAngle", gyro.getAngle());

        SmartDashboard.putNumber("scale value", (scaleValue));

        // Log subsystem to AK
        double[] acceleration = new double[] {
                this.gyro.getWorldLinearAccelX(), this.gyro.getWorldLinearAccelY()
        };

        double tagAngle = 0;

        Pose3d botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
        double robotHeading = getGyroYaw().getDegrees();

        double Dis = -botPose3D.getZ(); // Distance from LL to tag

        double rotationToTag = robotHeading + tagAngle;

        double zDis = Dis * Math.cos(Math.toRadians(rotationToTag));
        double xDis = zDis * (Math.tan(Math.toRadians(rotationToTag)));

        SmartDashboard.putNumber("PotentialForwardOffest", zDis);
        SmartDashboard.putNumber("PotentialSideOffest", xDis);
    }
}