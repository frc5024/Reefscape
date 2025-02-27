package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class SwerveDriveCommands {
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;

    /**
     * 
     */
    private SwerveDriveCommands() {
    }

    /**
     * 
     */
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), TeleopConstants.DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /**
     * 
     */
    public static double getOmegaFromJoysticks(double driverOmega) {
        double omega = MathUtil.applyDeadband(driverOmega, TeleopConstants.DEADBAND);
        return omega * omega * Math.signum(omega);
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command drive(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, boolean isOpenLoop) {
        return Commands.run(
                () -> {
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());
                    double omega = getOmegaFromJoysticks(omegaSupplier.getAsDouble());

                    swerveDriveSubsystem.drive(linearVelocity.getX(), linearVelocity.getY(), omega,
                            swerveDriveSubsystem.getRotation(), isOpenLoop);
                },
                swerveDriveSubsystem);
    }

    /**
     * 
     */

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(SwerveDriveSubsystem swerveDriveSubsystem, DoubleSupplier xSupplier,
            DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(ANGLE_KP, 0.0, ANGLE_KD,
                new TrapezoidProfile.Constraints(SwerveConstants.maxAngularSpeed,
                        SwerveConstants.maxAngularAcceleration));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Calculate angular speed
                    double omega = angleController.calculate(swerveDriveSubsystem.getRotation().getRadians(),
                            rotationSupplier.get().getRadians());

                    swerveDriveSubsystem.drive(linearVelocity.getX(), linearVelocity.getY(), omega, false);
                },
                swerveDriveSubsystem)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(swerveDriveSubsystem.getRotation().getRadians()));
    }
}
