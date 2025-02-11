package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralTest;

public class CoralTestCommand extends Command {
    private final CoralTest coralTest;

    public CoralTestCommand(CoralTest coralTest) {
        this.coralTest = coralTest;

        addRequirements(coralTest);
    }

    @Override
    public void initialize() {

    }

    public void execute() {
        coralTest.start();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        coralTest.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
