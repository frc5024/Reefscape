package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.elevatorConstants;;


public class Elevator extends SubsystemBase{
    private TalonFX elevatorMotor;
    private boolean enabled;
    private PIDController PID;

    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry pEntry = tab.add("SET P", elevatorConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", elevatorConstants.kD).getEntry();
    GenericEntry maxSpeedEntry = tab.add("SET Max Speed", (5)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();

    public Elevator() {
        elevatorMotor = new TalonFX(0);
        PID = new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD);
        elevatorMotor.setPosition(0);
        enabled = false;
        

    }

    @Override
    public void periodic(){
        if(enabled) {
            double speed = PID.calculate(elevatorMotor.getPosition().getValueAsDouble());
            elevatorMotor.set(speed);
        } else {
            double speed = 0;
            elevatorMotor.set(speed);
        }
    }

    public void setSetPoint(double position) {
        PID.setSetpoint(position);
    }

    public void motorOn(boolean enabled) {
        this.enabled = enabled;
    }




}



