package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.elevatorConstants;

public class Elevator extends SubsystemBase {
    // created and named the motor controller
    public SparkMax elevatorMotor; // LEAD R CHANGE THIS
    public SparkMax elevatorMotor2; // FOLLOWER L
    private final SparkBaseConfig elevatorMotorConfig = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake) // sets the motors to break mode
            .inverted(true);
    private final SparkBaseConfig elevatorMotor2Config = new SparkMaxConfig()
            .idleMode(IdleMode.kBrake)
            // .inverted(true)
            .follow(elevatorConstants.motorID1, false);

    // created and named the PID controller
    private PIDController PID;
    private double gConstant;

    // made it so "speed" is able to be accessed by the whole class
    private double speed;

    public double elevatorMode;
    public double elevatorPosition;

    // created and named the limit switches
    private static DigitalInput zeroingLimitSwitch;
    private static DigitalInput stoppingLimitSwitch;

    // added shuffleboard tabs to change the different values in the shuffle board
    // app
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry pEntry = tab.add("SET P", elevatorConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", elevatorConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", elevatorConstants.kI).getEntry();
    GenericEntry gEntry = tab.add("SET G", elevatorConstants.G).getEntry();
    GenericEntry maxUpSpeedEntry = tab.add("SET Max Speed", (elevatorConstants.elevatorMaxUpSpeed)).getEntry();
    GenericEntry maxDownSpeedEntry = tab.add("SET Max Down Speed", (elevatorConstants.elevatorMaxDownSpeed)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();
    GenericEntry motor1ManualEntry = tab.add("SET MANUAL SPEED", 0.0).getEntry();

    // constructor
    public Elevator() {
        // assigning the ID and values
        elevatorMotor = new SparkMax(elevatorConstants.motorID1, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(elevatorConstants.motorID2, SparkLowLevel.MotorType.kBrushless);
        this.elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        this.elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        zeroingLimitSwitch = new DigitalInput(8);
        stoppingLimitSwitch = new DigitalInput(1);

        // assigning values to the P, I and D
        PID = new PIDController(elevatorConstants.kP, elevatorConstants.kI, elevatorConstants.kD);
        elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);

        // elevatorMotor.setPosition(0);

        tab.addDouble("encoder value", () -> elevatorMotor.getEncoder().getPosition()); // callback loop. calls the
                                                                                        // function everytime it wants a
                                                                                        // value. Constantly checks the
                                                                                        // value.
        tab.addDouble("encoder valueDEG", () -> Units.radiansToDegrees(elevatorMotor.getEncoder().getPosition()));
        tab.addDouble("encoder2 valueDEG", () -> Units.radiansToDegrees(elevatorMotor2.getEncoder().getPosition()));
        tab.addDouble("PID Speed Value", () -> speed);
        tab.addBoolean("bottom limitswitch", () -> isBottomLimitSwitchBroken());
        tab.addBoolean("toplimitswitch", () -> isTopLimitSwitchBroken());

    }

    @Override
    public void periodic() {

        // getting the PID values and showing them on the shuffle board ("getDouble"
        // constantly checks the value)
        PID.setP(pEntry.getDouble(elevatorConstants.kP));
        PID.setD(dEntry.getDouble(elevatorConstants.kD));
        // using "I" value when going down because I is needed to get to a position
        // going down, but isn't needed going up
        if (speed < 0) {
            PID.setI(iEntry.getDouble(elevatorConstants.kI));
        } else {
            PID.setI(0);
        }

        gConstant = gEntry.getDouble(elevatorConstants.G);

        // //if the boolean enabled is true then run this command

        if (speed >= maxUpSpeedEntry.getDouble(elevatorConstants.elevatorMaxUpSpeed)) {
            elevatorMotor.set(maxUpSpeedEntry.getDouble(elevatorConstants.elevatorMaxUpSpeed));
        } else if (speed <= -maxDownSpeedEntry.getDouble(elevatorConstants.elevatorMaxDownSpeed)) {
            elevatorMotor.set(-maxDownSpeedEntry.getDouble(elevatorConstants.elevatorMaxDownSpeed));
        } else {
            elevatorMotor.set(speed);
        }

        // //safety precaution to prevent the motor from trying to go past the bottom
        // stop
        if (speed < 0 && encoderValue() <= elevatorConstants.minimumBottomValue) {
            elevatorMotor.set(0);
        }

        checkTopLimitSwitch();
        zeroingEncoder();
    }

    // creates a command for calculating speed through PID which will be used in the
    // command instead of the periodic
    public void pidMotor() {
        speed = PID.calculate(elevatorMotor.getEncoder().getPosition()) + gConstant;
    }

    // gets the position from the SetElevatorSetpointCmd
    public void setSetPoint(double position) {
        PID.setSetpoint(position);
        resetPID();
    }

    // makes it accesible to the SetElevatorSetpointCmd
    public void resetPID() {
        PID.reset();
    }

    // setting the manual target speed that can be controlled by the driver to the
    // speed used in calculations and safety checks
    public void controlMotor(double targetSpeed) {
        speed = targetSpeed;
    }

    // creating a boolean method which returns the condition of both limit switches
    public boolean isBottomLimitSwitchBroken() {
        return !zeroingLimitSwitch.get();
    }

    public boolean isTopLimitSwitchBroken() {
        return !stoppingLimitSwitch.get();
    }

    // encoder value will reset to 0 once the bottom limit switch is triggered
    public void zeroingEncoder() {
        if (isBottomLimitSwitchBroken()) {
            elevatorMotor.getEncoder().setPosition(elevatorConstants.zeroPosition);
            setElevatorPosition(Constants.elevatorConstants.rootPosition);
        }
    }

    // stop the motor if the the top limit switch is triggered
    public void checkTopLimitSwitch() {
        if (isTopLimitSwitchBroken()) {
            elevatorMotor.set(0);
            // elevatorMotor.get()
        }
    }

    // gets the encoder value for safety precautions in the periodic
    private double encoderValue() {
        return elevatorMotor.getEncoder().getPosition();
    }

    public void zeroEncoderValue() {
        elevatorMotor.getEncoder().setPosition(0.0);
    }

    @Override
    public Command getDefaultCommand() {
        return run(() -> elevatorMotor.set(0)); // ***set it to g constant so that it stays put in the future
    }

    public void motor1Manual() {
        elevatorMotor.set(motor1ManualEntry.getDouble(0));
    }

    public void motor2Manual() {
        elevatorMotor2.set(motor1ManualEntry.getDouble(0));
    }

    public void setElevatorMode(double reefLevel) {
        elevatorMode = reefLevel;
    }

    public double getElevatorMode() {
        return elevatorMode;
    }

    public void setElevatorPosition(double elevatorLevel) {
        elevatorPosition = elevatorLevel;
    }

    public double getElevatorPosition() {
        return elevatorPosition;
    }

}