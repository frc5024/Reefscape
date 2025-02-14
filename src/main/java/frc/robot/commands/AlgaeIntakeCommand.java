package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeCommandBased;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeCommandBased m_AlgaeCommandBased;
    boolean brokenLine = false;

    public AlgaeIntakeCommand(AlgaeCommandBased algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        brokenLine = false;

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // If the linebreak sensor has not been triggered, set the motors to intake mode
        if (!m_AlgaeCommandBased.getLinebreak() && !brokenLine) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.intakeSpeed);
        } else {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
            brokenLine = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
