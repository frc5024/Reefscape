package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class ElevatorPID extends PIDSubsystem{
    super (new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD));
}
