package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.elevatorConstants;

public class ElevatorPID extends PIDSubsystem{
    TalonFX elevatorMotor;

    public ElevatorPID(){
        super (new PIDController(elevatorConstants.kP,elevatorConstants.kI,elevatorConstants.kD));
        elevatorMotor = new TalonFX (1);
        
    }

    

    @Override
    protected void useOutput(double output, double setpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'useOutput'");
    }

    @Override
    protected double getMeasurement() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMeasurement'");
    }
}
