package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.leds.LEDPreset;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.LEDs;

public class ClimbCommand extends Command {

    private Climb climbSubsystem;
    private LEDs ledSubsystem;

    public ClimbCommand(Climb climbSubsystem, LEDs ledSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.ledSubsystem = ledSubsystem;

        addRequirements(climbSubsystem, ledSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
    }

    public void execute() {
        // Stops motor when Ultrasonic sensor and Encoder detect end position
        if (climbSubsystem.overThreshold() && climbSubsystem.isClimbPosition()) {
            ledSubsystem.setLEDS(LEDPreset.Rainbow.kConfetti);
        } else {
            climbSubsystem.climbing();
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopMotor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
