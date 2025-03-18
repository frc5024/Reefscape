package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDs;

public class ClimbCommand extends Command {

    private Climb climbSubsystem;
    public Timer climbTimer = new Timer();

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

    // slows climb to 30% for 1 second before stopping
    @Override
    public void end(boolean interrupted) {
        climbTimer.reset();
        LEDs.getInstance().setCommand(LEDPreset.Rainbow.kConfetti).schedule();

        if (climbTimer.get() <= 1) {
            climbSubsystem.stopMotor();
        } else {
            climbSubsystem.slowClimb();
        }
    }

    // Stops when the motors are at climbed position
    @Override
    public boolean isFinished() {
        return climbSubsystem.isLimitSwitchBroken();
    }
}
