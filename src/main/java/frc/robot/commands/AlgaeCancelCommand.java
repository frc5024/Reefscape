package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeCommandBased;

// Used to set motors to idle (off)
public class AlgaeCancelCommand extends Command {
    private final AlgaeCommandBased m_AlgaeCommandBased;

    public AlgaeCancelCommand(AlgaeCommandBased algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Interrupts any ongoing commands and sets motor speeds to idleSpeed (off)
        m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);

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
