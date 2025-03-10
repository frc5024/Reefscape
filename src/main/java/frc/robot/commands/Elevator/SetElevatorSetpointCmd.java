package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class SetElevatorSetpointCmd extends Command {
    public final Elevator elevatorSubsystem;

    // creates a variable called setpoint
    public double setpoint;

    public SetElevatorSetpointCmd(Elevator elevatorSubsystem, double setpoint) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.setpoint = setpoint;

        addRequirements(elevatorSubsystem);

    }

    // once controller button is pressed, "initialize" is called
    @Override
    public void initialize() {
        elevatorSubsystem.resetPID();
        // sets the setpoint to the position that is assigned to the button
        elevatorSubsystem.setGoal(setpoint);
        elevatorSubsystem.togglePID(true);
        if (setpoint != Constants.elevatorConstants.rootPosition) {
            elevatorSubsystem.setElevatorPosition(setpoint);
        }
    }

    // "end" is called when the button is no longer being pressed
    @Override
    public void end(boolean interrupted) {
        // elevatorSubsystem.resetPID();
        // elevatorSubsystem.togglePID(false);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.targetReached();
    }

}