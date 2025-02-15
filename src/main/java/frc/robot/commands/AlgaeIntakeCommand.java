package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeCommandBased;

// Intakes 
public class AlgaeIntakeCommand extends Command {
    private final AlgaeCommandBased m_AlgaeCommandBased;
    boolean hasAlgae = false;

    public AlgaeIntakeCommand(AlgaeCommandBased algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        System.out.println("Initializing Intake");

        // If linebreak is false, set hasAlgae to false and set motors to intake speed
        // otherwise, set hasAlgae to true and set motors to idle speed
        if (!m_AlgaeCommandBased.getLinebreak()) {
            hasAlgae = false;
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.intakeSpeed);
            System.out.println("setted the motors");
        } else {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
            hasAlgae = true;
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        System.out.println("Executing Intake");

        // If linebreak is triggered, set motors to
        // idlespeed and set hasAlgae to true
        if (m_AlgaeCommandBased.getLinebreak()) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
            hasAlgae = true;
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("End");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return hasAlgae;
    }
}
