package frc.robot.commands.tuning;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * 
 */
public class CharacterizationCommands {
    private final SwerveDriveSubsystem swerveDriveSubsystem;
    private final SysIdRoutine driveRoutine;
    private final SysIdRoutine turnRoutine;

    /**
     * 
     */
    public CharacterizationCommands(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        /*
         * SysId routine for characterizing translation. This is used to find PID gains
         * for the drive motors.
         */
        this.driveRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Default ramp rate is acceptable
                        Volts.of(3.5),
                        Seconds.of(3.5),
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> swerveDriveSubsystem.runDriveCharacterizationVolts(volts.in(Volts)),
                        null,
                        swerveDriveSubsystem));

        /*
         * SysId routine for characterizing steer. This is used to find PID gains for
         * the steer motors.
         */
        this.turnRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // Default ramp rate is acceptable
                        Volts.of(3.5),
                        Seconds.of(3.5),
                        // Log state with Phoenix SignalLogger class
                        (state) -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
                new SysIdRoutine.Mechanism(
                        (volts) -> swerveDriveSubsystem.runTurnCharacterizationVolts(volts.in(Volts)),
                        null,
                        swerveDriveSubsystem));
    }

    /**
     * 
     */
    public Command runDriveCharacterizationCmd() {
        return Commands.sequence(
                runOnce(() -> SignalLogger.start()),
                this.driveRoutine.quasistatic(Direction.kForward),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.driveRoutine.quasistatic(Direction.kReverse),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.driveRoutine.dynamic(Direction.kForward),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.driveRoutine.dynamic(Direction.kReverse),
                runOnce(() -> SignalLogger.stop()));
    }

    /**
     * 
     */
    public Command runTurnCharacterizationCmd() {
        return Commands.sequence(
                runOnce(() -> SignalLogger.start()),
                this.turnRoutine.quasistatic(Direction.kForward),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.turnRoutine.quasistatic(Direction.kReverse),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.turnRoutine.dynamic(Direction.kForward),
                runOnce(() -> this.swerveDriveSubsystem.stop()).withTimeout(1.0),
                this.turnRoutine.dynamic(Direction.kReverse),
                runOnce(() -> SignalLogger.stop()));
    }
}
