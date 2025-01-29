package frc.robot.subsystems.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.lib.statemachine.StateMetadata;
import frc.robot.modules.algae.AlgaeIntakeModuleIO;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.utils.MapleSimUtil;

public class AlgaeIntakeSubsystemSim extends AlgaeIntakeSubsystem {
    /**
     * 
     */
    public AlgaeIntakeSubsystemSim(AlgaeIntakeModuleIO intakeModule) {
        super(intakeModule);
    }

    /**
     * 
     */
    protected void handleEject(StateMetadata<Action> stateMetadata) {
        super.handleEject(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.ejectAlgae();
        }
    }

    /**
     * 
     */
    protected void handleIntake(StateMetadata<Action> stateMetadata) {
        super.handleIntake(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getIntakeSimulation().startIntake();
        }
    }

    /**
     * 
     */
    protected void handleStop(StateMetadata<Action> stateMetadata) {
        super.handleStop(stateMetadata);

        if (stateMetadata.isFirstRun()) {
            MapleSimUtil.getIntakeSimulation().stopIntake();
        }
    }

    @Override
    public boolean hasAlgae() {
        return MapleSimUtil.getIntakeSimulation().getGamePiecesAmount() != 0;
    }

    @Override
    public boolean hasEjected() {
        MapleSimUtil.getIntakeSimulation().obtainGamePieceFromIntake();
        return !hasAlgae();
    }

    @Override
    public void simulationPeriodic() {
        Pose3d algaeIntakePose = new Pose3d();

        if (hasAlgae()) {
            Pose2d robotPose = MapleSimUtil.getSwerveDriveSimulation().getSimulatedDriveTrainPose();
            Transform3d transform3d = new Transform3d(Units.inchesToMeters(-11.5), 0.0, Units.inchesToMeters(16.0),
                    new Rotation3d());

            algaeIntakePose = new Pose3d(robotPose).transformBy(transform3d);
        }

        Logger.recordOutput("FieldSimulation/Algae Intake Pose", algaeIntakePose);
    }
}
