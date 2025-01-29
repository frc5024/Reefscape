package frc.robot.subsystems.simulation;

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
}
