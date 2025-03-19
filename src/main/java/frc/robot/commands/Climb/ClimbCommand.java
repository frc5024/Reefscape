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

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
    }

    public void execute() {
        /*
         * Only starts climbing if the cage hits the limit switch -- no else in case the
         * cage bounces off the switch
         */
        if (climbSubsystem.extended && !climbSubsystem.isLimitSwitchBroken()) {
            climbSubsystem.retractArm();
        } else {
            climbSubsystem.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        /* Celebrates then stops */
        LEDs.getInstance().setCommand(LEDPreset.Rainbow.kConfetti).schedule();
        climbSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Stops when the motors are at climbed position
        return climbSubsystem.isLimitSwitchBroken();
    }
}
