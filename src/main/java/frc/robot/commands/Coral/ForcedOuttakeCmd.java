package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class ForcedOuttakeCmd extends Command {

    private final Coral coralSubsystem;
    private final Elevator elevatorSubsystem;

    // constructor for OuttakeCommand
    public ForcedOuttakeCmd(Coral coralSubsystem, Elevator elevatorSubsystem) {
        this.coralSubsystem = coralSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(coralSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // execute, startOuttake() once button is pressed
    @Override
    public void execute() {
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
        return false;
    }
}