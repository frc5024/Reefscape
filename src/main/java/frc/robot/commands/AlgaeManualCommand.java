package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class AlgaeManualCommand extends Command {
    private final Algae m_algae;

    public AlgaeManualCommand(Algae algae, DoubleSupplier outPower, DoubleSupplier inPower) {
        this.m_algae = algae;

        addRequirements(m_algae);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
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
