package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
// import frc.robot.systems.AutoHandlerSystem.AutoFSMState;
import edu.wpi.first.wpilibj.I2C;
// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LED;
import frc.robot.MechConstants;

public class MBRFSMv2 {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum MBRFSMState {
		MOVE_TO_GROUND,
		INTAKING,
		MOVE_TO_SHOOTER,
		SHOOTING,
		MOVE_TO_AMP,
	}

	private double[] currLogs;
	private int tick = 0;
	private boolean holding = false;
	private int noteColorFrames = 0;

	/* ======================== Private variables ======================== */
	private MBRFSMState currentState;
	private CANSparkMax shooterLeftMotor;
	private CANSparkMax shooterRightMotor;
	private TalonFX intakeMotor;
	private TalonFX pivotMotor;
	private final ColorSensorV3 colorSensor;
	private LED led = new LED();

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private Encoder throughBore;
	private Timer timer;



	/* ======================== Constructor ======================== */
	/**
	 * Create PivotFSM and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public MBRFSMv2() {
		shooterLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_LSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);

		shooterRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_RSHOOTER_MOTOR,
										CANSparkMax.MotorType.kBrushless);
		intakeMotor = new TalonFX(HardwareMap.DEVICE_ID_INTAKE_MOTOR);
		intakeMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotMotor = new TalonFX(HardwareMap.DEVICE_ID_ARM_MOTOR);
		pivotMotor.setNeutralMode(NeutralModeValue.Brake);

		throughBore = new Encoder(0, 1);
		throughBore.reset();

		timer = new Timer();
		currLogs = new double[MechConstants.AVERAGE_SIZE];

		colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public MBRFSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		led.greenLight(false);
		currentState = MBRFSMState.MOVE_TO_SHOOTER;
		holding = false;

		timer.stop();
		timer.reset();
		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		if (input == null) {
			return;
		}

		currLogs[tick % MechConstants.AVERAGE_SIZE] =
			intakeMotor.getSupplyCurrent().getValueAsDouble();

		tick++;

		double avgcone = 0;
		for (int i = 0; i < MechConstants.AVERAGE_SIZE; i++) {
			avgcone += currLogs[i];
		}
		avgcone /= MechConstants.AVERAGE_SIZE;

		//SmartDashboard.putNumber("avg current", avgcone);
		SmartDashboard.putBoolean("holding", holding);
		SmartDashboard.putString("TeleOP STATE", currentState.toString());
		//SmartDashboard.putBoolean("Input button pressed", input.isIntakeButtonPressed());
		SmartDashboard.putNumber("CurrentNoteFrames", noteColorFrames);
		SmartDashboard.putString("Current State", getCurrentState().toString());
		SmartDashboard.putNumber("CurrentIntakeMotor", intakeMotor.get());
		SmartDashboard.putNumber("Intake power", intakeMotor.get());
		SmartDashboard.putNumber("Pivot power", pivotMotor.get());
		//SmartDashboard.putNumber("Left shooter power", shooterLeftMotor.get());
		//SmartDashboard.putNumber("Right shooter power", shooterRightMotor.get());
		SmartDashboard.putNumber("Pivot encoder count", throughBore.getDistance());
		//SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
		SmartDashboard.putBoolean("HASNOTE --- ", hasNote());

		switch (currentState) {
			case MOVE_TO_SHOOTER:
				handleMoveShooterState(input);
				break;
			case INTAKING:
				handleIntakingState(input);
				break;
			case MOVE_TO_GROUND:
				handleMoveGroundState(input);
				break;
			case SHOOTING:
				handleShootingState(input);
				break;
			case MOVE_TO_AMP:
				handleMoveAmpState(input);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	///**
	// * Performs specific action based on the autoState passed in.
	// * @param autoState autoState that the subsystem executes.
	// * @return if the action carried out in this state has finished executing
	// */
	// public boolean updateAutonomous(AutoFSMState autoState) {
	// 	switch (autoState) {
	// 		case NOTE1:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE2:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE3:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE4:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE5:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE6:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE7:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case NOTE8:
	// 			return handleAutoMoveGround() && handleAutoIntake();
	// 		case SPEAKER:
	// 			return handleAutoMoveShooter() & handleAutoRev();
	// 		case SHOOT:
	// 			return handleAutoShoot();
	// 		case SHOOT_PRELOADED:
	// 			return handleAutoShootPreloaded();
	// 		default:
	// 			return true;
	// 	}
	// }

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private MBRFSMState nextState(TeleopInput input) {
		switch (currentState) {
			case MOVE_TO_SHOOTER:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isAmpButtonPressed()
					&& !holding) {
					return MBRFSMState.MOVE_TO_GROUND;
				}

				if (input.isAmpButtonPressed() && !input.isIntakeButtonPressed()
					&& !input.isShootButtonPressed()
					&& !input.isRevButtonPressed()) {
					return MBRFSMState.MOVE_TO_AMP;
				}

				if (!input.isIntakeButtonPressed() && !input.isAmpButtonPressed()
					&& (input.isShootButtonPressed()
					|| input.isRevButtonPressed())) {
					if (inRange(throughBore.getDistance(),
						MechConstants.SHOOTER_ENCODER_ROTATIONS)) {
						holding = false;

						return MBRFSMState.SHOOTING;
					} else {
						return MBRFSMState.MOVE_TO_SHOOTER;
					}
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case MOVE_TO_GROUND:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isAmpButtonPressed()) {
					if (inRange(throughBore.getDistance(),
						MechConstants.GROUND_ENCODER_ROTATIONS)) {
						return MBRFSMState.INTAKING;
					} else {
						return MBRFSMState.MOVE_TO_GROUND;
					}
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case INTAKING:
				if (input.isIntakeButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isAmpButtonPressed()) {
					return MBRFSMState.INTAKING;
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case SHOOTING:
				if (!input.isIntakeButtonPressed() && (input.isShootButtonPressed()
					|| input.isRevButtonPressed()) && !input.isAmpButtonPressed()) {
					return MBRFSMState.SHOOTING;
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			case MOVE_TO_AMP:
				if (input.isAmpButtonPressed() && !input.isShootButtonPressed()
					&& !input.isRevButtonPressed() && !input.isIntakeButtonPressed()) {
					return MBRFSMState.MOVE_TO_AMP;
				}
				return MBRFSMState.MOVE_TO_SHOOTER;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ Command handlers ------------------------ */

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < MechConstants.INRANGE_VALUE; //EXPERIMENTAL
	}

	private double pid(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P * (targetEncoder - currentEncoderPID);
		return Math.min(MechConstants.MAX_TURN_SPEED, Math.max(MechConstants.MIN_TURN_SPEED, correction));
	}

	private double pidAuto(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P_AUTO * (targetEncoder - currentEncoderPID);
		return Math.min(MechConstants.MAX_TURN_SPEED_AUTO, Math.max(MechConstants.MIN_TURN_SPEED_AUTO, correction));
	}

	public void setIntakeMotorPower(float power) {
		intakeMotor.set(power);
	}

	public void pidPivotAuto(double encoderFinal) {
		System.out.println(throughBore.getDistance());
		pivotMotor.set(pidAuto(throughBore.getDistance(), encoderFinal));
	}

	public boolean pidPivotCompleted(double encoderFinal) {
		return inRange(throughBore.getDistance(), encoderFinal);
	}

	public void setShooterLeftMotorPower(double power) {
		shooterLeftMotor.set(-power);
	}

	public void setShooterRightMotorPower(double power) {
		shooterRightMotor.set(power);
	}

	/* ---------------------------- FSM State Handlers ---------------------------- */

	/**
	 * Handles the moving to shooter state of the MBR Mech.
	 * @param input
	 */
	public void handleMoveShooterState(TeleopInput input) {
		if (!holding) {
			led.blueLight();
		} else {
			led.rainbow();
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		if (!input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(0);
		} else if (input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_INTAKE_POWER);
		} else if (input.isManualOuttakeButtonPressed() && !input.isManualIntakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_OUTTAKE_POWER);
			holding = false;
		}
	}

	/**
	 * Handles the moving to ground state of the MBR Mech.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleMoveGroundState(TeleopInput input) {
		led.orangeLight(false);
		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);

		if (!input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(0);
		} else if (input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_INTAKE_POWER);
		} else if (input.isManualOuttakeButtonPressed() && !input.isManualIntakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_OUTTAKE_POWER);
		}
	}

	/**
	 * Handles the intaking state of the MBR Mech.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleIntakingState(TeleopInput input) {
		if (!holding) {
			led.orangeLight(false);
		} else {
			led.rainbow();
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		if (input.isIntakeButtonPressed() && !holding && !input.isManualIntakeButtonPressed()) {
			intakeMotor.set(MechConstants.INTAKE_POWER);
		} else if (!input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(0);
		} else if (input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_INTAKE_POWER);
		} else if (input.isManualOuttakeButtonPressed() && !input.isManualIntakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_OUTTAKE_POWER);
		}
	}

	/**
	 * Handles the shooting state of the MBR Mech.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleShootingState(TeleopInput input) {
		led.blueLight();

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));
		if (input.isRevButtonPressed() && !input.isShootButtonPressed()) {
			shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
			shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			intakeMotor.set(0);
		}
		if (input.isShootButtonPressed()) {
			shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
			shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			intakeMotor.set(MechConstants.OUTTAKE_POWER);
		}
	}

	/**
	 * Handles the moving to Amp state of the MBR Mech.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void handleMoveAmpState(TeleopInput input) {
		//led.greenLight(false);
		// shooterLeftMotor.set(0);
		// shooterRightMotor.set(0);
		if (!holding) {
			led.redLight(false);
		} else {
			led.rainbow();
		}

		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.AMP_ENCODER_ROTATIONS));
		// if (input.isShootAmpButtonPressed()) {
		// 	intakeMotor.set(AMP_SHOOT_POWER);
		// } else {
		// 	intakeMotor.set(0);
		// }

		//if (input.isAmpButtonPressed() && !input.isShootAmpButtonPressed()) {
		//	shooterLeftMotor.set(-MechConstants.AMP_SHOOTER_POWER); // dont forget the - sign
		//	shooterRightMotor.set(MechConstants.AMP_SHOOTER_POWER);
		//	intakeMotor.set(0);
		//}

		//if (input.isAmpButtonPressed() && input.isShootAmpButtonPressed()) {
		//	shooterLeftMotor.set(-MechConstants.AMP_SHOOTER_POWER);
		//	shooterRightMotor.set(MechConstants.AMP_SHOOTER_POWER);
		//	intakeMotor.set(MechConstants.AMP_OUTTAKE_POWER);
		//}

		if (input.isManualIntakeButtonPressed() && !input.isManualOuttakeButtonPressed()) {
			intakeMotor.set(MechConstants.MANUAL_INTAKE_POWER);
		} else if (input.isManualOuttakeButtonPressed() && !input.isManualIntakeButtonPressed()) {
			intakeMotor.set(MechConstants.AMP_OUTTAKE_POWER);
		} else {
			intakeMotor.set(0);
		}
	}


	/**
	 * Handles the Auto Move to Ground state of the MBR Mech.
	 * @return if the pivot is at the correct position
	 */
	public boolean handleAutoMoveGround() {
		led.orangeLight(false);
		pivotMotor.set(pidAuto(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), MechConstants.GROUND_ENCODER_ROTATIONS);
	}

	/**
	 * Handles the Auto Move to Shooter state of the MBR Mech.
	 * @return if the pivot is at the correct position
	 */
	public boolean handleAutoMoveShooter() {
		if (holding) {
			led.rainbow();
		} else {
			led.orangeLight(false);
		}

		intakeMotor.set(MechConstants.AUTO_HOLDING_POWER);
		pivotMotor.set(pidAuto(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));
		return inRange(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS);
	}

	/**
	 * Handles the Auto Rev state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoRev() {
		if (holding) {
			led.rainbow();
		} else {
			led.orangeLight(false);
		}
		shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
		shooterRightMotor.set(MechConstants.SHOOTING_POWER);
		return true;
	}

	/**
	 * Handles the Auto Shoot state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoShoot() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));
		if (timer.get() > MechConstants.AUTO_SHOOTING_TIME) {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		} else {
			intakeMotor.set(MechConstants.OUTTAKE_POWER);
			shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
			shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			return false;
		}
	}

	/**
	 * Handles the Auto Shoot state for a preloaded note.
	 * @return if the action is completed
	 */
	public boolean handleAutoShootPreloaded() {
		if (timer.get() == 0) {
			timer.start();
		}
		pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));
		if (timer.get() < 1 + 0.5) {
			intakeMotor.set(0);
			shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
			shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			return false;
		} else if (timer.get() < MechConstants.AUTO_PRELOAD_SHOOTING_TIME + 0.5) {
			intakeMotor.set(MechConstants.OUTTAKE_POWER);
			shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
			shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			return false;
		} else {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);
			timer.stop();
			timer.reset();
			return true;
		}
	}

	/**
	 * Handles the Auto Intake state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoIntake() {
		intakeMotor.set(MechConstants.AUTO_INTAKE_POWER);
		shooterLeftMotor.set(0);
		shooterRightMotor.set(0);
		return hasNote();
	}

	/**
	 * Handles the Auto Outtake state of the MBR Mech.
	 * @return if the action is completed
	 */
	public boolean handleAutoOuttake() {
		intakeMotor.set(MechConstants.OUTTAKE_POWER);
		pivotMotor.set(0);
		return false;
	}

	/**
	 * Checks if the intake is holding a note.
	 * @return if the intake is holding a note
	 */
	public boolean hasNote() {
		boolean isInRange = colorSensor.getProximity() >= MechConstants.PROXIMIIY_THRESHOLD;
		SmartDashboard.putBoolean("is close enough", isInRange);

		noteColorFrames = isInRange ? (noteColorFrames + 1) : 0;
		holding = noteColorFrames >= MechConstants.NOTE_FRAMES_MIN;

		return holding;
	}

	/* --------------------------- COMMAND CLASSES --------------------------- */

	public class ShootPreloadedCommand extends Command {

		private Timer timerSub;

		/**
		 * ShootPreloadedNoteCommand command.
		 */
		public ShootPreloadedCommand() {
			timerSub = new Timer();
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			timerSub.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			led.rainbow();
			pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));

			if (timerSub.get() < 0.8 + 0.5) {
				intakeMotor.set(0);
				shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
				shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			} else if (timerSub.get() < MechConstants.AUTO_PRELOAD_SHOOTING_TIME + 0.5) {
				intakeMotor.set(MechConstants.OUTTAKE_POWER);
				shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
				shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);

			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return timerSub.get() >= MechConstants.AUTO_PRELOAD_SHOOTING_TIME + 0.5;
		}
	}

	public class ShootNoteCommand extends Command {

		private Timer timerSub;

		/**
		 * ShootNoteCommand command.
		 */
		public ShootNoteCommand() {
			timerSub = new Timer();
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			timerSub.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			pivotMotor.set(pid(throughBore.getDistance(), MechConstants.SHOOTER_ENCODER_ROTATIONS));

			if (timerSub.get() < MechConstants.AUTO_SHOOTING_TIME) {
				intakeMotor.set(MechConstants.OUTTAKE_POWER);
				shooterLeftMotor.set(0);
				shooterRightMotor.set(0);
			} else if (timerSub.get() < MechConstants.AUTO_SHOOTING_TIME + 0.5) {
				shooterLeftMotor.set(-MechConstants.SHOOTING_POWER);
				shooterRightMotor.set(MechConstants.SHOOTING_POWER);
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			intakeMotor.set(0);
			shooterLeftMotor.set(0);
			shooterRightMotor.set(0);

			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return timerSub.get() >= MechConstants.AUTO_SHOOTING_TIME + 0.5;
		}
	}

	public class IntakeNoteCommand extends Command {

		Timer timerSub;

		public IntakeNoteCommand() {
			timerSub = new Timer();
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			led.orangeLight(false);
			timerSub.start();
		}
		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			setIntakeMotorPower(0);
			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return handleAutoIntake();
		}
	}

	public class PivotGroundToShooterCommand extends Command {

		private Timer timerSub;

		/**
		 * PivotGroundToShooter command.
		 */
		public PivotGroundToShooterCommand() {
			timerSub = new Timer();
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			setIntakeMotorPower(MechConstants.AUTO_HOLDING_POWER);
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			if (holding) {
				led.rainbow();
			}

			if (handleAutoMoveShooter()) {
				timerSub.start();
			}
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			setIntakeMotorPower(0);
			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return handleAutoMoveShooter() && timerSub.get() > 0.5;
		}
	}

	public class PivotShooterToGroundCommand extends Command {

		private Timer timerSub;

		/**
		 * PivotShooterToGround command.
		 */
		public PivotShooterToGroundCommand() {
			timerSub = new Timer();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			led.orangeLight(false);
			if (handleAutoMoveGround()) {
				timerSub.start();
			}

			System.out.println("pstg");
		}

		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			timerSub.stop();
			timerSub.reset();
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return handleAutoMoveGround() && timerSub.get() > 0.5;
		}
	}

	public class RevShooterCommand extends Command {

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return handleAutoRev();
		}
		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			//setShooterLeftMotorPower(0);
			//setShooterRightMotorPower(0);
		}
	}

	public class OuttakeNoteCommand extends Command {
		private Timer timerSub;

		/**
		 * OuttakeNote command.
		 */
		public OuttakeNoteCommand() {
			timerSub = new Timer();
		}

		// Called when the command is initially scheduled.
		@Override
		public void initialize() {
			led.blueLight();
			timerSub.start();
		}

		// Called every time the scheduler runs while the command is scheduled.
		@Override
		public void execute() {
			handleAutoOuttake();
			System.out.println("OUTTAKING" + timerSub.get());
		}

		// Returns true when the command should end.
		@Override
		public boolean isFinished() {
			return handleAutoOuttake() || timerSub.get() >= 0.7;
		}
		// Called once the command ends or is interrupted.
		@Override
		public void end(boolean interrupted) {
			timerSub.stop();
			timerSub.reset();

			setShooterLeftMotorPower(0);
			setShooterRightMotorPower(0);
			setIntakeMotorPower(0);
			holding = false;
		}
	}

}
