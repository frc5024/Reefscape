package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class SetElevatorModeCmd extends Command {
    public final Elevator elevatorSubsystem;

    // creates a variable called setpoint
    public double setpoint;

    public SetElevatorModeCmd(Elevator elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);

    }

    // once controller button is pressed, "initialize" is called
    @Override
    public void initialize() {
        setpoint = elevatorSubsystem.getSetpoint();
        elevatorSubsystem.resetPID();

        System.out.println("Setpoint final; " + setpoint);

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