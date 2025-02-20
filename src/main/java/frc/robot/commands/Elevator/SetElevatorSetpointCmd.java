package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
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
        // sets the setpoint to the position that is assigned to the button
        elevatorSubsystem.setSetPoint(setpoint);

    }

    // will use PID calculatioins to set the speed. Execute is used because it is
    // called only when the command is running
    @Override
    public void execute() {
        elevatorSubsystem.pidMotor();
    }

    // "end" is called when the button is no longer being pressed
    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.controlMotor(0);
        elevatorSubsystem.resetPID();
    }

}