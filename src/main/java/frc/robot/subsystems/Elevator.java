package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;;


public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor;
    
    private PIDController PID;

    public Elevator() {
        elevatorMotor = new TalonFX(0);
        PID = new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD);
        elevatorMotor.setPosition(0);


    }

    @Override
    public void periodic(){
        if(enabled) {
            double speed = PID.calculate(elevatorMotor.getPosition().getValueAsDouble());
        } else {
            double speed = 0;
        }
    }

    public void setSetPoint(double position) {
        PID.setSetpoint(position);
    }

    public void motorOn(boolean enabled) {
        
    }


}



