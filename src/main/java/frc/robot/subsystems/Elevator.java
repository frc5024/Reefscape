package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
    GenericEntry iEntry = tab.add("SET I", elevatorConstants.kI).getEntry();
    GenericEntry maxSpeedEntry = tab.add("SET Max Speed", (0.3)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();

    public Elevator() {
        elevatorMotor = new TalonFX(21);
        PID = new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD);
        elevatorMotor.setPosition(0);
        enabled = false;
        
        tab.addDouble("encoder value", () ->elevatorMotor.getPosition().getValueAsDouble()); //callback loop. calls the function everytime it wants a value. Constantly checks the value.
 
    }

    @Override
    public void periodic(){
        PID.setP(pEntry.getDouble(elevatorConstants.kP));
        PID.setD(dEntry.getDouble(elevatorConstants.kD));
        PID.setI(dEntry.getDouble(elevatorConstants.kI));
        double speed = 0;
        if(enabled) {
            speed = PID.calculate(elevatorMotor.getPosition().getValueAsDouble());
        }

        //max speed
        if (speed >= maxSpeedEntry.getDouble(0.3)) {
            elevatorMotor.set(maxSpeedEntry.getDouble(0.3));
        } else {
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



