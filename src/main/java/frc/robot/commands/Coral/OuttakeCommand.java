package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class OuttakeCommand extends Command {

    private final Coral coralSubsystem;
    private final Elevator elevatorSubsystem;

    boolean isAuto = false;
    double speed;

    // constructor for OuttakeCommand
    public OuttakeCommand(Coral coralSubsystem, Elevator elevatorSubsystem, boolean isAuto) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.isAuto = isAuto;

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
            cancel();
        }

        if (elevatorSubsystem.getElevatorPosition() == Constants.elevatorConstants.L1Position) {
            coralSubsystem.setTop(-Constants.coralConstants.L1Speed);
            coralSubsystem.setBottom(0);
        } else {
            coralSubsystem.set(Constants.coralConstants.outtakeSpeed);
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