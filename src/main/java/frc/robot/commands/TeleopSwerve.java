package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private SlewRateLimiter translationFilter = new SlewRateLimiter(5);
    private SlewRateLimiter strafeFilter = new SlewRateLimiter(5);
    private SlewRateLimiter rotationFilter = new SlewRateLimiter(5);
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Slew rate limiter */
        double translationOutput = translationFilter.calculate(translationVal);
        double strafeOutput = strafeFilter.calculate(strafeVal);
        double rotationOutput = rotationFilter.calculate(rotationVal);

        /* Drive */
        s_Swerve.controllerTranslationalVal(translationOutput);
        s_Swerve.controllerStrafeVal(strafeOutput);
        s_Swerve.controllerRotationVal(rotationOutput);

        s_Swerve.drive(true);
    }
}
