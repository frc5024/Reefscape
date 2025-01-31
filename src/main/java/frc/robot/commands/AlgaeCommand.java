package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class AlgaeCommand extends Command {
    private final Algae m_algae;
    private DoubleSupplier outPower;
    private DoubleSupplier inPower;

    public AlgaeCommand(Algae algae, DoubleSupplier outPower, DoubleSupplier inPower) {
        m_algae = algae;
        this.outPower = outPower;
        this.inPower = inPower;
        addRequirements(m_algae);

    }
    // Reads values from triggers and assigns them to variables

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double algaeMotorSpeed = (outPower.getAsDouble() * 0.5) + (-inPower.getAsDouble() * 0.5);
        m_algae.setSpeed(algaeMotorSpeed);

    }
    // Halfes the trigger values and adds them, then assigns them to m_algae as
    // setSpeed

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
