package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Rumble;

public class OuttakeCommand extends Command {

    private final Coral coralSubsystem;
    private Elevator elevatorSubsystem;
    Rumble rumble = new Rumble();

    // constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (!coralSubsystem.isLineBroken()) {
            cancel();
        }
    }

    // execute, startOuttake() once button is pressed
    @Override
    public void execute() {
        if (!coralSubsystem.isLineBroken()) {
            rumble.doubleRumble(true);
            cancel();
        }

        coralSubsystem.set(CoralConstants.outtakeSpeed);

        // if (elevatorSubsystem.getElevatorPosition() ==
        // Constants.elevatorConstants.L1position) {
        // coralSubsystem.set(Constants.coralConstants.L1Speed);
        // } else {
        // coralSubsystem.set(Constants.coralConstants.outtakeSpeed);
        // }

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