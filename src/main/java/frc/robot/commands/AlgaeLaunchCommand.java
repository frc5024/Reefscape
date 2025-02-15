package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeCommandBased;

public class AlgaeLaunchCommand extends Command {
    private final AlgaeCommandBased m_AlgaeCommandBased;

    Timer launchTimer = new Timer();

    public AlgaeLaunchCommand(AlgaeCommandBased algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        launchTimer.reset();
        launchTimer.start();

        if (!m_AlgaeCommandBased.getLinebreak()) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set the motors to launch mode (outtake) if there is something in the intake
        // system and timer is not elasped
        if (launchTimer.hasElapsed(Constants.Algaes.outtaketimer)) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
        } else if (m_AlgaeCommandBased.getLinebreak()) {
            // Set motors to idle if timer has passed timer time
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.launchSpeed);
        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

        // Sets speed to idle (off) if command is interrupted
        m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
