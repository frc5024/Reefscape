package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.elevator.SetElevatorSetpointCmd;

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
            .follow(ElevatorConstants.motorID1, false);

    // created and named the PID controller
    private ProfiledPIDController PID;
    private ElevatorFeedforward feedForward;
    private TrapezoidProfile.Constraints feedForwardConstraints;

    // made it so "speed" is able to be accessed by the whole class
    private double speed;
    private double voltageValue;
    private boolean enabled;

    public boolean slow = false;

    // created and named the limit switches
    private static DigitalInput zeroingLimitSwitch;
    private static DigitalInput stoppingLimitSwitch;

    // added shuffleboard tabs to change the different values in the shuffle board
    // app
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    GenericEntry pEntry = tab.add("SET P", ElevatorConstants.kP).getEntry();
    GenericEntry dEntry = tab.add("SET D", ElevatorConstants.kD).getEntry();
    GenericEntry iEntry = tab.add("SET I", ElevatorConstants.kI).getEntry();
    GenericEntry gEntry = tab.add("SET G", ElevatorConstants.G).getEntry();
    GenericEntry vEntry = tab.add("SET V", ElevatorConstants.kV).getEntry();
    GenericEntry aEntry = tab.add("SET A", ElevatorConstants.kA).getEntry();
    GenericEntry maxSpeedEntry = tab.add("SET Max Speed",
            (ElevatorConstants.elevatorMaxSpeed)).getEntry();
    GenericEntry maxAccelerationEntry = tab.add("SET Max accel",
            (ElevatorConstants.elevatorMaxAccel)).getEntry();
    GenericEntry SETsetPoint = tab.add("SET Dest (DEG)", 0.0).getEntry();
    // GenericEntry motor1ManualEntry = tab.add("SET MANUAL SPEED", 0.0).getEntry();

    private Rumble rumble;
    public boolean zeroRumbled = false;

    public double elevatorMode = ElevatorConstants.L4Position;
    public double elevatorPosition = 0;

    private static Elevator mInstance;

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    // constructor
    private Elevator() {
        // assigning the ID and values
        elevatorMotor = new SparkMax(ElevatorConstants.motorID1, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(ElevatorConstants.motorID2, SparkLowLevel.MotorType.kBrushless);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMotor2.configure(elevatorMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zeroingLimitSwitch = new DigitalInput(7);
        stoppingLimitSwitch = new DigitalInput(1);

        // assigning values to the P, I and D
        feedForwardConstraints = new TrapezoidProfile.Constraints(ElevatorConstants.elevatorMaxSpeed,
                ElevatorConstants.elevatorMaxAccel);

        PID = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD,
                feedForwardConstraints);
        PID.setTolerance(2, 1.5); // TODO: put in constants
        feedForward = new ElevatorFeedforward(0, ElevatorConstants.G, ElevatorConstants.kV, ElevatorConstants.kA); // ks,
                                                                                                                   // kg,
                                                                                                                   // kv,
                                                                                                                   // ka
        elevatorMotor.getEncoder().setPosition(ElevatorConstants.zeroPosition);

        // elevatorMotor.setPosition(0);

        // callback loop. calls the function everytime it wants a value. Constantly
        // checks the value.
        // tab.addDouble("PID Speed Value", () -> speed);
        tab.addBoolean("bottom limitswitch", () -> isBottomLimitSwitchBroken());
        tab.addBoolean("toplimitswitch", () -> isTopLimitSwitchBroken());
        tab.addDouble("voltage", () -> voltageValue);
        tab.addDouble("Actual Velocity", () -> rotationsToInches(elevatorMotor.getEncoder().getVelocity()) / 60);
        tab.addDouble("Estimated Velocity", () -> PID.getSetpoint().velocity);
        tab.addDouble("actual Position", () -> rotationsToInches(elevatorMotor.getEncoder().getPosition()));
        tab.addDouble("estimated Position", () -> PID.getSetpoint().position);
        tab.addDouble("encoder value", () -> elevatorMotor.getEncoder().getPosition());
        tab.addDouble("appliedOutput", () -> speed);
        tab.addDouble("goal", () -> PID.getGoal().position);
        tab.addBoolean("Elevator AtTarget", () -> targetReached());

        // TODO: log voltage anything else you think you need

    }

    @Override
    public void periodic() {

        // getting the PID values and showing them on the shuffle board ("getDouble"
        // constantly checks the value)
        PID.setP(pEntry.getDouble(ElevatorConstants.kP));
        PID.setI(iEntry.getDouble(ElevatorConstants.kI));
        PID.setD(dEntry.getDouble(ElevatorConstants.kD));
        feedForward.setKa(aEntry.getDouble(ElevatorConstants.kA));
        feedForward.setKv(vEntry.getDouble(ElevatorConstants.kV));
        feedForward.setKg(gEntry.getDouble(ElevatorConstants.G));

        if (enabled) {
            feedPIDMotor();
        } else {
            elevatorMotor.set(0);
        }

        speed = elevatorMotor.getAppliedOutput();

        // //if the boolean enabled is true then run this command

        // if (speed >= maxUpSpeedEntry.getDouble(ElevatorConstants.elevatorMaxUpSpeed))
        // {
        // elevatorMotor.set(maxUpSpeedEntry.getDouble(ElevatorConstants.elevatorMaxUpSpeed));
        // } else if (speed <=
        // -maxDownSpeedEntry.getDouble(ElevatorConstants.elevatorMaxDownSpeed)) {
        // elevatorMotor.set(-maxDownSpeedEntry.getDouble(ElevatorConstants.elevatorMaxDownSpeed));
        // } else {
        // elevatorMotor.set(speed);
        // }

        // //safety precaution to prevent the motor from trying to go past the bottom
        // stop
        if (PID.getGoal().position == ElevatorConstants.rootPosition
                && elevatorHeight() <= ElevatorConstants.minimumBottomValue) {
            elevatorMotor.set(0); // TODO: verify our minimumBottomValue tolerance is working and reasonable
        }

        checkTopLimitSwitch();
        zeroingEncoder();

        rumble = Rumble.getInstance();

        if (!zeroRumbled && isBottomLimitSwitchBroken()) {
            rumble.staticRumble(false);
            zeroRumbled = true;
        } else if (!isBottomLimitSwitchBroken()) {
            zeroRumbled = false;
        }
    }

    // TODO: rewrite comment to be more accurate
    // creates a command for calculating speed through PID which will be used in the
    // command instead of the periodic
    public void feedPIDMotor() {
        voltageValue = PID.calculate(rotationsToInches(elevatorMotor.getEncoder().getPosition()))
                + feedForward.calculate(PID.getSetpoint().velocity);
        elevatorMotor.setVoltage(voltageValue);
    }

    public boolean targetReached() {
        if (PID.atGoal()) {
            return true;
        } else {
            return false;
        }
    }

    // gets the position from the SetElevatorSetpointCmd
    public void setGoal(double inches) {
        if (!slow) {
            PID.setConstraints(new TrapezoidProfile.Constraints(
                    maxSpeedEntry.getDouble(ElevatorConstants.elevatorMaxSpeed),
                    maxAccelerationEntry.getDouble(ElevatorConstants.elevatorMaxAccel)));
        } else {
            PID.setConstraints(new TrapezoidProfile.Constraints(
                    maxSpeedEntry.getDouble(ElevatorConstants.elevatorMaxSpeed) / 4,
                    maxAccelerationEntry.getDouble(ElevatorConstants.elevatorMaxAccel) / 4));
        }

        PID.setGoal(inches);
        resetPID();

    }

    // makes it accesible to the SetElevatorSetpointCmd
    public void resetPID() {
        PID.reset(rotationsToInches(elevatorMotor.getEncoder().getPosition()));
    }

    // setting the manual target speed that can be controlled by the driver to the
    // speed used in calculations and safety checks
    // public void controlMotor(double targetSpeed) {
    // speed = targetSpeed;
    // }

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
            elevatorMotor.getEncoder().setPosition(ElevatorConstants.zeroPosition);
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

    // @Override
    // public Command getDefaultCommand() {
    // return run(() -> elevatorMotor.set(0)); // ***set it to g constant so that it
    // stays put in the future
    // }

    // public void motor1Manual() {
    // elevatorMotor.set(motor1ManualEntry.getDouble(0));
    // }

    // public void motor2Manual() {
    // elevatorMotor2.set(motor1ManualEntry.getDouble(0));
    // }

    // conversion method
    public static double inchesToRotations(double distanceValue) {
        return distanceValue * (0.729687);
    }

    public static double rotationsToInches(double angleValue) {
        return angleValue / (0.729687); // TODO: verify this conversion is accurate (does the carriage actually move X
                                        // inches?)
    }

    public void togglePID(boolean enabled) {
        this.enabled = enabled;
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

    public double getElevatorPercent() {
        double encoder = encoderValue();

        double percent = encoder / 60; // divide by amount of rotations

        return percent;
    }

    public double elevatorHeight() {
        return rotationsToInches(elevatorMotor.getEncoder().getPosition());
    }

    public Command goToL1Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.L1Position);
    }

    public Command goToL2Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.L2Position);
    }

    public Command goToL3Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.L3Position);
    }

    public Command goToL4Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.L4Position);
    }

    public Command goToAlgae1Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.Algae1);
    }

    public Command goToAlgae2Position() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.Algae2);
    }

    public Command bottomElevator() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.rootPosition);
    }

    public Command bottomAutoElevator() {
        return new SetElevatorSetpointCmd(this, ElevatorConstants.rootAutoPosition);
    }

    public Command goToModePosition() {
        return new SetElevatorSetpointCmd(this, elevatorMode);
    }

    public Command slowL2() {
        return new SequentialCommandGroup(
                runOnce(() -> slow = true),
                goToL2Position())
                .finallyDo(() -> slow = false);
    }
}