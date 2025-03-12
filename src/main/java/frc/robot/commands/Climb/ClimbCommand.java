package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDs;

public class ClimbCommand extends Command {

    private Climb climbSubsystem;

    public ClimbCommand(Climb climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
    }

    // moves motor only if the limit switch is false
    public void execute() {
        if (climbSubsystem.extended && !climbSubsystem.isLimitSwitchBroken()) {
            climbSubsystem.retractArm();
        } else {
            climbSubsystem.stopMotor();
        }
    }

    // Celebrates then stops
    @Override
    public void end(boolean interrupted) {
        LEDs.getInstance().setCommand(LEDPreset.Rainbow.kConfetti).schedule();
        climbSubsystem.stopMotor();
    }

    // Stops when the motors are at climbed position
    @Override
    public boolean isFinished() {
        return climbSubsystem.isLimitSwitchBroken();
    }
}
