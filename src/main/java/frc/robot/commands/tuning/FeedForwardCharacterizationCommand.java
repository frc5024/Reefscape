package frc.robot.commands.tuning;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDriveSubsystem;

/**
 * Measures the velocity feedforward constants for the drive motors.
 *
 * <p>
 * This command should only be used in voltage control mode.
 */
public class FeedForwardCharacterizationCommand {
    private final double FF_START_DELAY = 2.0; // Secs
    private final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    /**
     * 
     */
    public FeedForwardCharacterizationCommand(SwerveDriveSubsystem swerveDriveSubsystem) {
        this.swerveDriveSubsystem = swerveDriveSubsystem;
    }

    /**
     * 
     */
    public Command get() {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient
                Commands.run(() -> {
                    this.swerveDriveSubsystem.runDriveCharacterizationVolts(0.0);
                },
                        this.swerveDriveSubsystem)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            this.swerveDriveSubsystem.runDriveCharacterizationVolts(voltage);
                            velocitySamples.add(this.swerveDriveSubsystem.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        },
                        this.swerveDriveSubsystem)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

}
