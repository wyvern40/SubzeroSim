package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class Arm extends SubsystemBase {
    
    public enum ArmState {
        CORAL_STOW(ArmConstants.coralStowSetpoint),
        CORAL_ALIGN(ArmConstants.coralAlignSetpoint),
        CORAL_SCORE(ArmConstants.coralScoreSetpoint);

		private final Angle angle;

		ArmState(Angle angle) {
			this.angle = angle;
		}
    }

    public class ArmData {

        @Logged(name = "State")
        public ArmState state;

        @Logged(name = "Position")
        public Angle position;
        @Logged(name = "Velocity")
        public AngularVelocity velocity;

        @Logged(name = "Target Position")
        public Angle targetPosition;
        @Logged(name = "Target Velocity")
        public AngularVelocity targetVelocity;

        @Logged(name = "Roller Position")
        public Angle rollerPosition;
        @Logged(name = "Roller Velocity")
        public AngularVelocity rollerVelocity;

    }

    @Getter
    @Logged(name = "Data")
    private final ArmData data = new ArmData();

    private ArmState state = ArmState.CORAL_STOW;

    private final TalonFX pivotMotor = new TalonFX(ArmConstants.pivotMotorID);
    private final TalonFXSimState pivotMotorSim = pivotMotor.getSimState();

    private final TalonFX rollerMotor = new TalonFX(ArmConstants.rollerMotorID);
    private final TalonFXSimState rollerMotorSim = pivotMotor.getSimState();
    
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(ArmConstants.startingAngle);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        ArmConstants.gearRatio,
        ArmConstants.moi,
        ArmConstants.length.in(Meters),
        ArmConstants.minAngle.in(Radians),
        ArmConstants.maxAngle.in(Radians),
        true,
        ArmConstants.startingAngle.in(Radians)
    );

    public Arm() {
        setUpPivotMotor();
        setUpRollerMotor();
    }

    private void applyPIDConfigs() {
        var talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.Slot0 = new Slot0Configs()
            .withKP(ArmConstants.kP.get())
            .withKS(ArmConstants.kS.get())
            .withKG(ArmConstants.kG.get())
            .withKV(ArmConstants.kV.get())
            .withKV(ArmConstants.kA.get())
            .withGravityType(GravityTypeValue.Arm_Cosine);

        talonFXConfigs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.maxVelocity.get();
		talonFXConfigs.MotionMagic.MotionMagicAcceleration = ArmConstants.maxAcceleration.get();

        pivotMotor.getConfigurator().apply(talonFXConfigs);
    }

    private void setUpPivotMotor() {

        applyPIDConfigs();

        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.pivotStatorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ArmConstants.pivotSupplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;

        var feedbackConfigs = new FeedbackConfigs().withSensorToMechanismRatio(ArmConstants.gearRatio);

		pivotMotor.getConfigurator().apply(limitConfigs);
		pivotMotor.getConfigurator().apply(feedbackConfigs);

    }

    private void setUpRollerMotor() {
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.StatorCurrentLimit = ArmConstants.rollerStatorCurrentLimit;
		limitConfigs.StatorCurrentLimitEnable = true;

		limitConfigs.SupplyCurrentLimit = ArmConstants.rollerSupplyCurrentLimit;
		limitConfigs.SupplyCurrentLimitEnable = true;
        
        rollerMotor.getConfigurator().apply(limitConfigs);
    }

    public void simulationPeriodic() {

        pivotMotor.setControl(motionMagic
			.withSlot(0)
			.withPosition(state.angle)
        );
        
        if(
            ArmConstants.kP.hasChanged() ||
            ArmConstants.kS.hasChanged() ||
            ArmConstants.kG.hasChanged() ||
            ArmConstants.kV.hasChanged() ||
            ArmConstants.kA.hasChanged() ||
            ArmConstants.maxVelocity.hasChanged() ||
            ArmConstants.maxAcceleration.hasChanged()
        ) {
            applyPIDConfigs();
        }

		pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        rollerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

		armSim.setInput(pivotMotor.getMotorVoltage().getValueAsDouble());

		armSim.update(0.020);

		pivotMotorSim.setRawRotorPosition(Radians.of(armSim.getAngleRads() * ArmConstants.gearRatio));
		pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(armSim.getVelocityRadPerSec() * ArmConstants.gearRatio));

		RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));
		
        data.position = pivotMotor.getPosition().getValue();
		data.velocity = pivotMotor.getVelocity().getValue();

		data.targetPosition = Rotations.of(pivotMotor.getClosedLoopReference().getValue());
        data.targetVelocity = RotationsPerSecond.of(pivotMotor.getClosedLoopReferenceSlope().getValue());

        data.rollerPosition = rollerMotor.getPosition().getValue();
        data.rollerVelocity = rollerMotor.getVelocity().getValue();

		data.state = state;
	}

    public boolean atSetpoint() {
        return data.position.minus(state.angle).abs(Degrees) < ArmConstants.setpointTolerance.in(Degrees);
    }

    public Command swapState(ArmState state) {
		return this.run(() -> this.state = state);
	}
}
