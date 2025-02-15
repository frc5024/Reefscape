package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class LowerRampCommand extends Command {

    private final Coral coralSubsystem;
    // private double startingValue;
    private Timer timer = new Timer();

    public LowerRampCommand(Coral coralSubsystem) {

        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // when start, rotate -plopspeed, lowers the ramp
    public void initialize() {
        coralSubsystem.setBottom(Constants.coralConstants.rampSpeed);
        timer.reset();
        timer.start();
    }

    // when button pressed, rotate servo 90 degrees (lowers the ramp)
    public void execute() {
    }

    // when command ends, set idle
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return coralSubsystem.isLineBroken() || timer.get() > 0.5;
    }
}