package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeCommandBased;

public class AlgaeLaunchCommand extends Command {
    private final AlgaeCommandBased m_AlgaeCommandBased;
    boolean hasAlgae = false;

    Timer launchTimer = new Timer();

    public AlgaeLaunchCommand(AlgaeCommandBased algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        // Reset dropTimer (set to 0) and then start timer
        launchTimer.reset();
        launchTimer.start();

        // If there is something in the intake system, set motors to launchSpeed
        if (m_AlgaeCommandBased.getLinebreak()) {
            hasAlgae = true;
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.launchSpeed);
        } else {
            // If there is nothing in the intake system, set motors to idle and end command
            hasAlgae = false;
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
        }

        m_AlgaeCommandBased.setSpeed(Constants.Algaes.launchSpeed);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set the motors to idleSpeed and set hasAlgae to false (ending the command) if
        // timer has elapsed outtaketimer time (1.5 seconds)
        // if (launchTimer.hasElapsed(Constants.Algaes.outtaketimer)) {
        // m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
        // hasAlgae = false;
        // }

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
