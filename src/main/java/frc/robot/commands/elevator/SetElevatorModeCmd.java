package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class SetElevatorModeCmd extends Command {
    public final Elevator elevatorSubsystem;

    public double mode;

    public SetElevatorModeCmd(Elevator elevatorSubsystem, double mode) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.mode = mode;

        addRequirements(elevatorSubsystem);

    }

    // stores the elevator desired mode as its degree form
    @Override
    public void initialize() {
        if (mode == ElevatorConstants.L1Position) {
            elevatorSubsystem.setElevatorMode(mode);
            SmartDashboard.putString("ElevatorMode", "L1");
        } else if (mode == ElevatorConstants.L2Position) {
            elevatorSubsystem.setElevatorMode(mode);
            SmartDashboard.putString("ElevatorMode", "L2");
        } else if (mode == ElevatorConstants.L3Position) {
            elevatorSubsystem.setElevatorMode(mode);
            SmartDashboard.putString("ElevatorMode", "L3");
        } else if (mode == ElevatorConstants.L4Position) {
            elevatorSubsystem.setElevatorMode(mode);
            SmartDashboard.putString("ElevatorMode", "L4");
        }

    }

    @Override
    public void execute() {
    }

    // "end" is called when the button is no longer being pressed
    @Override
    public void end(boolean interrupted) {
    }

}