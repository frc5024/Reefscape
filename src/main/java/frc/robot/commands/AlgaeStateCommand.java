package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Algae;

public class AlgaeStateCommand extends Command {
    private final Algae m_algae;
    private DoubleSupplier buttonAxis;
    // private DoubleSupplier servobuttonaxis;

    private int presstime = 0;

    public AlgaeStateCommand(Algae algae, DoubleSupplier buttonAxis) {
        this.m_algae = algae;
        this.buttonAxis = buttonAxis;
        // this.servobuttonaxis = servobuttonaxis;
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
        // Sets presstime to number of frames button has been held for
        presstime = m_algae.checkPressTime(presstime, buttonAxis.getAsDouble());
        m_algae.motorSpeedStateMachine(presstime);
        // m_algae.setServoSpeed(servobuttonaxis);

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
