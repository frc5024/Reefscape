package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class OuttakeCommand extends Command {

    private final Coral coralSubsystem;

    boolean isAuto = false;
    double speed;

    // constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem, boolean isAuto) {
        this.coralSubsystem = coralSubsystem;
        this.isAuto = isAuto;

        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!coralSubsystem.isLineBroken()) {
            cancel();
        }

        if (!isAuto)
            speed = Constants.coralConstants.outtakeSpeed;
        else
            speed = Constants.coralConstants.outtakeAutoSpeed;

        coralSubsystem.set(speed);
    }

    // execute, startOuttake() once button is pressed
    @Override
    public void execute() {
        if (!coralSubsystem.isLineBroken()) {
            cancel();
        }
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