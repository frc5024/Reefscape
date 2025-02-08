package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class TestCommand extends Command {
    public TestCommand() {

    }

    @Override
    public void initialize() {
        System.out.println("dye!");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
