package frc.robot.modules.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/**
 * 
 */
public class ElevatorModuleIOSim implements ElevatorModuleIO {
    // This gearbox represents a gearbox containing 4 Vex 775pro motors.
    private final DCMotor m_elevatorGearbox = DCMotor.getVex775Pro(4);

    // Standard classes for controlling our elevator
    private final ProfiledPIDController m_controller = new ProfiledPIDController(
            SimConstants.kElevatorKp,
            SimConstants.kElevatorKi,
            SimConstants.kElevatorKd,
            new TrapezoidProfile.Constraints(2.45, 2.45));
    ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
            SimConstants.kElevatorkS,
            SimConstants.kElevatorkG,
            SimConstants.kElevatorkV,
            SimConstants.kElevatorkA);
    private final Encoder m_encoder = new Encoder(SimConstants.kEncoderAChannel, SimConstants.kEncoderBChannel);
    private final PWMSparkMax m_motor = new PWMSparkMax(SimConstants.kMotorPort);

    // Simulation classes help us simulate what's going on, including gravity.
    private final ElevatorSim m_elevatorSim = new ElevatorSim(
            m_elevatorGearbox,
            SimConstants.kElevatorGearing,
            SimConstants.kCarriageMass,
            SimConstants.kElevatorDrumRadius,
            SimConstants.kMinElevatorHeightMeters,
            SimConstants.kMaxElevatorHeightMeters,
            true,
            0,
            0.01,
            0.0);
    private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);
    private final PWMSim m_motorSim = new PWMSim(m_motor);

    private final ElevatorMechanism elevatorMechanism = new ElevatorMechanism();

    /**
     * 
     */
    public ElevatorModuleIOSim() {
        m_encoder.setDistancePerPulse(SimConstants.kElevatorEncoderDistPerPulse);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        // With the setpoint value we run PID control like normal
        double pidOutput = m_controller.calculate(m_encoder.getDistance());
        double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
        m_motor.setVoltage(pidOutput + feedforwardOutput);

        m_elevatorSim.setInput(m_motorSim.getSpeed() * RobotController.getBatteryVoltage());
        m_elevatorSim.update(0.020);

        m_encoderSim.setDistance(m_elevatorSim.getPositionMeters());
        elevatorMechanism.update(m_elevatorSim.getPositionMeters());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_elevatorSim.getCurrentDrawAmps()));

        Logger.recordOutput("Subsystems/Elevator/PositionMeters", m_elevatorSim.getPositionMeters());
    }

    @Override
    public boolean isAtDistance() {
        return m_controller.atGoal();
    }

    @Override
    public void runOpenLoop(double output) {
        m_controller.setGoal(output);

        // // With the setpoint value we run PID control like normal
        // double pidOutput = m_controller.calculate(m_encoder.getDistance());
        // double feedforwardOutput =
        // m_feedforward.calculate(m_controller.getSetpoint().velocity);
        // m_motor.setVoltage(pidOutput + feedforwardOutput);
    }

    @Override
    public void runVolts(double volts) {
    }

    @Override
    public void stop() {
        m_controller.setGoal(0.0);
        m_motor.set(0.0);
    }

    @Override
    public void runPosition(double positionRad, double feedforward) {
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
    }
}
