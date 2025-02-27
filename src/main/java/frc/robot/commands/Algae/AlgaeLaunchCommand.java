package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;

public class AlgaeLaunchCommand extends Command {
    private final Algae m_AlgaeCommandBased;
    boolean hasAlgae = false;

    Timer launchTimer = new Timer();

    public AlgaeLaunchCommand(Algae algaeCommandBased) {
        this.m_AlgaeCommandBased = algaeCommandBased;
        addRequirements(m_AlgaeCommandBased);
    }

    @Override
    public void initialize() {

        // Reset dropTimer (set to 0) and then start timer
        launchTimer.reset();
        launchTimer.start();

        hasAlgae = true;

        m_AlgaeCommandBased.setSpeed(Constants.Algaes.launchSpeed);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Set the motors to idleSpeed and set hasAlgae to false (ending the command) if
        // timer has elapsed outtaketimer time (1.5 seconds)
        if (launchTimer.hasElapsed(Constants.Algaes.outtakeTimer)) {
            m_AlgaeCommandBased.setSpeed(Constants.Algaes.idleSpeed);
            hasAlgae = false;
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
