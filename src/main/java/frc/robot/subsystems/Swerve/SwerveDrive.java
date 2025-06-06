package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.BranchSide;
import frc.robot.commands.DriveToPoseCommand;
import frc.robot.subsystems.Swerve.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SwerveDrive extends TunerSwerveDrivetrain implements Subsystem {
    
    private static SwerveDrive instance;

	public static synchronized SwerveDrive getInstance() {
		if (instance == null) {
			instance = TunerConstants.createDrivetrain();
		}

		return instance;
	}

    public enum SwerveState {
        DRIVER_CONTROL,
        PATH_TO_REEF,
        ALIGN_TO_REEF,
        STOPPED;
    }

    //private SwerveState state;

    private BranchSide targetSide;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private DriveToPoseCommand autoDriveCommand;

    public SwerveDrive(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        configurePathPlanner();
    }


    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    private void configurePathPlanner() {

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            throw new Error(e.getMessage());
        }

        AutoBuilder.configure(
            () -> this.getState().Pose,
            this::resetPose,
            () -> getState().Speeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds, feedforwards),
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            config,
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );
    }

    private void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        this.setControl(new SwerveRequest.ApplyRobotSpeeds()
            .withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
        );
    }

    private Command driveToReef() {

        var nearestPose = this.getState().Pose.nearest(Arrays.asList(FieldConstants.REEF_FACES));

        return autoDriveCommand = new DriveToPoseCommand(
            nearestPose.transformBy(
                new Transform2d(SwerveConstants.LINEUP_DISTANCE.unaryMinus(), targetSide.offset.unaryMinus(), Rotation2d.kZero)
            ),
            false
        );
    }

    private Command alignToReef() {

        var nearestPose = this.getState().Pose.nearest(Arrays.asList(FieldConstants.REEF_FACES));

        return autoDriveCommand = new DriveToPoseCommand(
            nearestPose.transformBy(
                new Transform2d(Meters.of(0.0), targetSide.offset.unaryMinus(), Rotation2d.kZero)
            ),
            true
        );
    }

    public boolean atTargetPose() {
        if(autoDriveCommand != null) {
            return autoDriveCommand.atSetpoint();
        }
        return true;
    }

    public void setTargetSide(BranchSide side) {
        this.targetSide = side;
    }

    public Command requestState(SwerveState requestedState) {
    
        //this.state = requestedState;
        
        //this.getCurrentCommand().cancel();

        switch(requestedState) {
            case PATH_TO_REEF:
                return driveToReef();
            case ALIGN_TO_REEF:
                return alignToReef();
            case DRIVER_CONTROL:
                return this.getDefaultCommand();
            case STOPPED:
                return Commands.idle(this);
            default:
                return Commands.none();
        }
    }
}