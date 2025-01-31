package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase{
     //created and named the motor controller
    private SparkMax elevatorMotor; 

    private boolean enabled;

    //created and named the PID controller
    private PIDController PID;

    //created and named the limit switches
    private static DigitalInput zeroingLimitSwitch;
    private static DigitalInput stoppingLimitSwitch;

    //added shuffleboard tabs to change the different values in the shuffle board app
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry pEntry = tab.add("SET P", elevatorConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", elevatorConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", elevatorConstants.kI).getEntry();
    GenericEntry maxSpeedEntry = tab.add("SET Max Speed", (elevatorConstants.elevatorMaxSpeed)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();

    //constructor
    public Elevator() {
        //assigning the ID and values
        elevatorMotor = new SparkMax(elevatorConstants.motorID, SparkLowLevel.MotorType.kBrushless);

        //assigning values to the P, I and D
        PID = new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD);
        elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);

        //elevatorMotor.setPosition(0);
        enabled = false;
        
       tab.addDouble("encoder value", () ->elevatorMotor.getEncoder().getPosition()); //callback loop. calls the function everytime it wants a value. Constantly checks the value.
       tab.addDouble("encoder valueDEG", () ->Units.radiansToDegrees(elevatorMotor.getEncoder().getPosition()));
    }


    @Override
    public void periodic(){

        //getting the PID values and showing them on the shuffle board ("getDouble" constantly checks the value)
        PID.setP(pEntry.getDouble(elevatorConstants.kP));
        PID.setD(dEntry.getDouble(elevatorConstants.kD));
        PID.setI(dEntry.getDouble(elevatorConstants.kI));

        double speed = 0;

        //if the boolean enabled is true then run this command
        if(enabled) {
            speed = PID.calculate(elevatorMotor.getEncoder().getPosition());
        }

        //max speed
        if (speed >= maxSpeedEntry.getDouble(elevatorConstants.elevatorMaxSpeed)) {
            elevatorMotor.set(maxSpeedEntry.getDouble(elevatorConstants.elevatorMaxSpeed));
        } else {
            elevatorMotor.set(speed);
        }
    }

    //gets the position from the SetElevatorSetpointCmd
    public void setSetPoint(double position) {
        PID.setSetpoint(position);
    }

    //get the condition of enabled from the elevator command
    public void motorOn(boolean enabled) {
        this.enabled = enabled;
    }


    //creating a boolean method which returns the condition of both limit switches
    public boolean isBottomLimitSwitchBroken() {
        return zeroingLimitSwitch.get();
    }

    public boolean isTopLimitSwitchBroken() {
        return stoppingLimitSwitch.get();
    }


    //encoder value will reset to 0 once the bottom limit switch is triggered
    public void zeroingEncoder () {
        if (isBottomLimitSwitchBroken()) {
            elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);
        }
    }

    //stop the motor if the the top limit switch is triggered
    public void stopMotor () {
        if (isTopLimitSwitchBroken()) {
            elevatorMotor.set(0);
        }
    }







}



