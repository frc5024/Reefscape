package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.modules.swerve.SwerveModuleConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation
 * velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis
 * runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall,
 * away from the alliance wall is positive.
 */
public class TeleopDriveCommand extends Command {
    private final SwerveDriveSubsystem swerveDrive;
    private final Supplier<Rotation2d> robotAngleSupplier;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(TeleopConstants.X_RATE_LIMIT);
    private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(TeleopConstants.Y_RATE_LIMIT);
    private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(TeleopConstants.ROTATION_RATE_LIMIT);

    /* These are used for Path Planner autonomous mode in the DrivePathCommand */
    private final ProfiledPIDController omegaController;

    private double rotationalAngle;

    /**
     * Constructor
     * 
     * @param swerveDrive          drivetrain
     * @param robotAngleSupplier   supplier for the current angle of the robot
     * @param translationXSupplier supplier for translation X component, in meters
     *                             per second
     * @param translationYSupplier supplier for translation Y component, in meters
     *                             per second
     * @param rotationSupplier     supplier for rotation component, in radians per
     *                             second
     */
    public TeleopDriveCommand(SwerveDriveSubsystem swerveDrive, Supplier<Rotation2d> robotAngleSupplier,
            DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.robotAngleSupplier = robotAngleSupplier;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        double[] driveOmegaPIDs = PIDConstants.getDriveOmegaPIDs();
        this.omegaController = new ProfiledPIDController(driveOmegaPIDs[0], driveOmegaPIDs[1], driveOmegaPIDs[2],
                TeleopConstants.OMEGA_CONSTRAINTS);

        this.rotationalAngle = 360.0; // this value is used to determine if the driverPov has been pressed

        addRequirements(swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        this.swerveDrive.stop();

        Logger.recordOutput("Commands/Active Command", "");
    }

    @Override
    public void execute() {
        Rotation2d angle = this.robotAngleSupplier.get();

        double xVelocity = -modifyAxis(this.translationXSupplier.getAsDouble() * SwerveModuleConstants.maxLinearSpeed);
        double yVelocity = -modifyAxis(this.translationYSupplier.getAsDouble() * SwerveModuleConstants.maxLinearSpeed);
        double rVelocity = -modifyAxis(this.rotationSupplier.getAsDouble() * 10.0 / 2);

        xVelocity = this.translateXRateLimiter.calculate(xVelocity);
        yVelocity = this.translateYRateLimiter.calculate(yVelocity);
        rVelocity = this.rotationRateLimiter.calculate(rVelocity);

        if (rVelocity == 0.0) {
            if (this.rotationalAngle != 360.0) {
                double goalRotation = Units.degreesToRadians(this.rotationalAngle);
                this.omegaController.setGoal(goalRotation);
                rVelocity = this.omegaController.calculate(this.robotAngleSupplier.get().getRadians());
            }

            if (this.omegaController.atGoal()) {
                rVelocity = 0;
                this.rotationalAngle = 360.0;
            }
        }

        this.swerveDrive.runVelocity(xVelocity, yVelocity, rVelocity, angle);

        Logger.recordOutput("Commands/TeleopDriveCommand/xVelocity", xVelocity);
        Logger.recordOutput("Commands/TeleopDriveCommand/yVelocity", yVelocity);
        Logger.recordOutput("Commands/TeleopDriveCommand/rVelocity", rVelocity);
        Logger.recordOutput("Commands/TeleopDriveCommand/angle", angle.getDegrees());
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Commands/Active Command", this.getName());
    }

    /**
     * 
     */
    private double modifyAxis(double value) {
        value = MathUtil.applyDeadband(value, 0.2);
        value = Math.copySign(value, value);

        return value;
    }
}