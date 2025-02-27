package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

// Intakes 
public class AlgaeIntakeCommand extends Command {
    private final Algae m_AlgaeCommandBased;
    boolean hasAlgae = false;

    public AlgaeIntakeCommand(Algae algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        // If linebreak is false, set hasAlgae to false and set motors to intakeSpeed
        if (!m_AlgaeCommandBased.getAlgaeLinebreak()) {
            hasAlgae = false;
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.intakeSpeed);
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // If linebreak is triggered, set motors to
        // idleSpeed and set hasAlgae to true
        if (m_AlgaeCommandBased.getAlgaeLinebreak()) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
            hasAlgae = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hasAlgae;
    }
}
