package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class L1Command extends Command {

    private final Coral coralSubsystem;

    boolean isAuto = false;
    double speed;

    // constructor for OuttakeCommand
    public L1Command(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        coralSubsystem.setTop(-Constants.coralConstants.L1Speed);
        coralSubsystem.setBottom(0);
    }

    // execute, startOuttake() once button is pressed
    @Override
    public void execute() {
    }

    // end, when command ends, set activeOuttake to false and set state to IDLE
    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setIdle();

    }

    // if line is not broken, return true, else return false
    @Override
    public boolean isFinished() {
        return !coralSubsystem.isLineBroken();
    }
}
