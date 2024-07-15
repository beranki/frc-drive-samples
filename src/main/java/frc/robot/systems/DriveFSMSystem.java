package frc.robot.systems;

// Third party Hardware Imports
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


// WPILib Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;
import frc.robot.LED;
import frc.robot.RaspberryPI;
import frc.robot.SwerveConstants.AutoConstants;
// import frc.robot.LED;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.SwerveConstants.VisionConstants;

public class DriveFSMSystem extends SubsystemBase {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		TELEOP_STATE,
		ALIGN_TO_SPEAKER_STATE,
		ALIGN_TO_NOTE_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	// The gyro sensor
	private AHRS gyro = new AHRS(SPI.Port.kMXP);
	private Timer timer = new Timer();

	private RaspberryPI rpi = new RaspberryPI();

	//led
	//private LED led = new LED();

	// Create MAXSwerveModules
	private final MAXSwerveModule frontLeft = new MAXSwerveModule(
		HardwareMap.FRONT_LEFT_DRIVING_CAN_ID,
		HardwareMap.FRONT_LEFT_TURNING_CAN_ID,
		DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule frontRight = new MAXSwerveModule(
		HardwareMap.FRONT_RIGHT_DRIVING_CAN_ID,
		HardwareMap.FRONT_RIGHT_TURNING_CAN_ID,
		DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearLeft = new MAXSwerveModule(
		HardwareMap.REAR_LEFT_DRIVING_CAN_ID,
		HardwareMap.REAR_LEFT_TURNING_CAN_ID,
		DriveConstants.REAR_LEFT_CHASSIS_ANGULAR_OFFSET);

	private final MAXSwerveModule rearRight = new MAXSwerveModule(
		HardwareMap.REAR_RIGHT_DRIVING_CAN_ID,
		HardwareMap.REAR_RIGHT_TURNING_CAN_ID,
		DriveConstants.REAR_RIGHT_CHASSIS_ANGULAR_OFFSET);

	// Odometry class for tracking robot pose
	private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
		DriveConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(getHeading()),
		new SwerveModulePosition[] {
			frontLeft.getPosition(),
			frontRight.getPosition(),
			rearLeft.getPosition(),
			rearRight.getPosition()
		});


	private int lockedSpeakerId;
	private boolean isSpeakerAligned;
	private boolean isNoteAligned;
	private boolean isSpeakerPositionAligned;


	private boolean redAlliance;
	private Double[] tagOrientationAngles;

	private StructArrayPublisher<SwerveModuleState> statePublisher
		= NetworkTableInstance.getDefault().getStructArrayTopic("MyStates",
		SwerveModuleState.struct).publish();

	private StructArrayPublisher<Pose2d> posePublisher
		= NetworkTableInstance.getDefault().getStructArrayTopic("MyPose",
		Pose2d.struct).publish();


	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		gyro = new AHRS(SPI.Port.kMXP);

		AutoBuilder.configureHolonomic(
				this::getPose,
					// Robot pose supplier
				this::resetPose,
					// Method to reset odometry (will be called if your auto has a starting pose)
				this::getRobotRelativeSpeeds,
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative,
					// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig(
						// HolonomicPathFollowerConfig, this should live in your Constants class
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID const
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID const
						4.5, // Max module speed, in m/s
						0.4, // Drive base radius (in m). Dist: robot center -> furthest module.
						new ReplanningConfig() // Default path replanning config.
				),
				() -> {
				// Boolean supplier that controls when the path will be mirrored for red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					redAlliance = (alliance.get() == DriverStation.Alliance.Red);
				}
				redAlliance = false;

				return redAlliance;
				},
				this // Reference to this subsystem to set requirements
		);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}


	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return odometry.getPoseMeters();
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
		currentState = FSMState.TELEOP_STATE;
		//led.turnOff();
		resetPose(new Pose2d());

		gyro.reset();
		gyro.setAngleAdjustment(0);

		if (redAlliance) {
			tagOrientationAngles = new Double[]
				{null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null, null,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null, null,
					null, null, null, null};
		} else {
			tagOrientationAngles = new Double[]
				{null, VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null,
					null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null,
					null, null, null, null, null};
		}

		lockedSpeakerId = -1;
		isSpeakerAligned = false;
		isNoteAligned = false;
		isSpeakerPositionAligned = false;

		update(null);
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */

	public void resetAutonomus() {
		gyro.reset();
		gyro.setAngleAdjustment(0);

		if (redAlliance) {
			tagOrientationAngles = new Double[]
				{null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null, null,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					-VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null, null, null, null, null,
					null};
		} else {
			tagOrientationAngles = new Double[]
				{null, VisionConstants.SOURCE_TAG_ANGLE_DEGREES,
					VisionConstants.SOURCE_TAG_ANGLE_DEGREES, null,
					null, null, null, VisionConstants.SPEAKER_TAG_ANGLE_DEGREES,
					VisionConstants.SPEAKER_TAG_ANGLE_DEGREES, null, null, null,
					null, null, null, null, null};
		}

		lockedSpeakerId = -1;
		isSpeakerAligned = false;
		isNoteAligned = false;
		isSpeakerPositionAligned = false;
	}


	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetPose(Pose2d pose) {
		odometry.resetPosition(
			Rotation2d.fromDegrees(getHeading()),
				new SwerveModulePosition[] {
					frontLeft.getPosition(),
					frontRight.getPosition(),
					rearLeft.getPosition(),
					rearRight.getPosition()
				},
				pose);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		odometry.update(Rotation2d.fromDegrees(getHeading()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()});

		if (input == null) {
			return;
		}

		if (redAlliance) {
			if (!(rpi.getAprilTagZInv(VisionConstants.RED_SOURCE_TAG1_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
				&& rpi.getAprilTagZInv(VisionConstants.RED_SOURCE_TAG2_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
				&& rpi.getAprilTagZInv(VisionConstants.RED_SPEAKER_TAG_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT)) {
				//led.greenLight();
				//SmartDashboard.putBoolean("Can see tag", true);
			} else {
				//led.orangeLight();
				//SmartDashboard.putBoolean("Can see tag", false);
			}
		} else {
			if (!(rpi.getAprilTagZInv(VisionConstants.BLUE_SOURCE_TAG1_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
				&& rpi.getAprilTagZInv(VisionConstants.BLUE_SOURCE_TAG2_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT
				&& rpi.getAprilTagZInv(VisionConstants.BLUE_SPEAKER_TAG_ID)
				== VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT)) {
				//led.greenLight();
				//SmartDashboard.putBoolean("Can see tag", true);
			} else {
				//led.orangeLight();
				//SmartDashboard.putBoolean("Can see tag", false);
			}
		}

		SmartDashboard.putString("Drive State", getCurrentState().toString());
		//SmartDashboard.putBoolean("Is Speaker Aligned", isSpeakerAligned);

		//SmartDashboard.putNumber("X Pos", getPose().getX());
		//SmartDashboard.putNumber("Y Pos", getPose().getY());
		//SmartDashboard.putNumber("Heading", getPose().getRotation().getDegrees());

		/*
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
		SmartDashboard.putNumber("Gyro Fused Heading", gyro.getFusedHeading());
		*/

		/*SmartDashboard.putNumber("x feed", -MathUtil.applyDeadband((
			input.getControllerLeftJoystickY()
					* Math.abs(input.getControllerLeftJoystickY()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2),
					OIConstants.DRIVE_DEADBAND));

		SmartDashboard.putNumber("y feed", -MathUtil.applyDeadband((
			input.getControllerLeftJoystickX()
					* Math.abs(input.getControllerLeftJoystickX()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2),
					OIConstants.DRIVE_DEADBAND));

		SmartDashboard.putNumber("ang feed", -MathUtil.applyDeadband(
			(input.getControllerRightJoystickX()
					* ((input.getLeftTrigger() / 2) + DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT)
					/ DriveConstants.ANGULAR_SPEED_LIMIT_CONSTANT),
					OIConstants.DRIVE_DEADBAND));



		SwerveModuleState[] states = new SwerveModuleState[] {
			frontLeft.getState(),
			frontRight.getState(),
			rearLeft.getState(),
			rearRight.getState()
		};

		Pose2d[] poses = new Pose2d[] {
			getPose()
		};

		statePublisher.set(states);
		posePublisher.set(poses);

		*/

		switch (currentState) {
			case TELEOP_STATE:
				drive(-MathUtil.applyDeadband((input.getControllerLeftJoystickY()
					* Math.abs(input.getControllerLeftJoystickY()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2), OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband((input.getControllerLeftJoystickX()
					* Math.abs(input.getControllerLeftJoystickX()) * ((input.getLeftTrigger() / 2)
					+ DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT) / 2), OIConstants.DRIVE_DEADBAND),
					-MathUtil.applyDeadband((input.getControllerRightJoystickX()
					* ((input.getLeftTrigger() / 2) + DriveConstants.LEFT_TRIGGER_DRIVE_CONSTANT)
					/ DriveConstants.ANGULAR_SPEED_LIMIT_CONSTANT), OIConstants.DRIVE_DEADBAND),
					true);

				if (input.isCrossButtonPressed()) {
					gyro.reset();
				}

				if (input.isTriangleButtonPressed()) {
					setForwardFormation();
				}

				if (input.isCircleButtonPressed()) {
					setXFormation();
				}

				break;

			case ALIGN_TO_SPEAKER_STATE:
				if (lockedSpeakerId == -1) {
					if (redAlliance) {
						//id 4
						if (rpi.getAprilTagZInv(VisionConstants.RED_SPEAKER_TAG_ID)
							!= VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
							lockedSpeakerId = VisionConstants.RED_SPEAKER_TAG_ID;
						}
					} else {
						//id 7
						if (rpi.getAprilTagZInv(VisionConstants.BLUE_SPEAKER_TAG_ID)
							!= VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
							lockedSpeakerId = VisionConstants.BLUE_SPEAKER_TAG_ID;
						}
					}
				} else {
					alignToSpeaker(lockedSpeakerId);
				}

				break;

			case ALIGN_TO_NOTE_STATE:
				if (rpi.getNoteYaw() != VisionConstants.UNABLE_TO_SEE_NOTE_CONSTANT) {
					alignToNote();
				}

				break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());

		}
		currentState = nextState(input);
	}

	/**
	 * Updates odo + smartboard values.
	 */
	public void updateAutonomous() {
		odometry.update(Rotation2d.fromDegrees(-gyro.getAngle()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()});
	}


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
	private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case TELEOP_STATE:
				if (input.isCircleButtonPressed()) {
					return FSMState.ALIGN_TO_SPEAKER_STATE;
				} else if (input.isCrossButtonPressed()) {
					isNoteAligned = false;
					return FSMState.ALIGN_TO_NOTE_STATE;
				}

				return FSMState.TELEOP_STATE;

			case ALIGN_TO_SPEAKER_STATE:
				if (input.isCircleButtonReleased()) {
					lockedSpeakerId = -1;
					isSpeakerAligned = false;
					isSpeakerPositionAligned = false;
					return FSMState.TELEOP_STATE;
				}
				return FSMState.ALIGN_TO_SPEAKER_STATE;

			case ALIGN_TO_NOTE_STATE:
				if (input.isCrossButtonReleased()) {
					isNoteAligned = false;
					return FSMState.TELEOP_STATE;
				}
				return FSMState.ALIGN_TO_NOTE_STATE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	*/
	public void drive(double xSpeed, double ySpeed, double rot,
		boolean fieldRelative) {
		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
		double rotDelivered = rot * DriveConstants.MAX_ANGULAR_SPEED;

		//SmartDashboard.putNumber("x speed delivered", xSpeedDelivered);
		//SmartDashboard.putNumber("y speed delivered", ySpeedDelivered);
		//SmartDashboard.putNumber("rot speed delivered", rotDelivered);


		var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered,
					rotDelivered, Rotation2d.fromDegrees(getHeading()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		/*SmartDashboard.putNumber("s0d", swerveModuleStates[0].speedMetersPerSecond);
		SmartDashboard.putNumber("s1d", swerveModuleStates[1].speedMetersPerSecond);
		SmartDashboard.putNumber("s2d", swerveModuleStates[2].speedMetersPerSecond);
		SmartDashboard.putNumber("s3d", swerveModuleStates[(2 + 1)].speedMetersPerSecond);

		SmartDashboard.putNumber("s0t", swerveModuleStates[0].angle.getRadians());
		SmartDashboard.putNumber("s1t", swerveModuleStates[1].angle.getRadians());
		SmartDashboard.putNumber("s2t", swerveModuleStates[2].angle.getRadians());
		SmartDashboard.putNumber("s3t", swerveModuleStates[(2 + 1)].angle.getRadians());*/

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[(2 + 1)]);

		//System.out.println("S1" + swerveModuleStates[0]);
		//System.out.println("S2" + swerveModuleStates[1]);
		//System.out.println("S3" + swerveModuleStates[2]);
		//System.out.println("S4" + swerveModuleStates[(2 + 1)]);
	}


	/**
	 * Drives the robot to a final odometry state.
	 * @param pose final odometry position for the robot
	 * @return if the robot has driven to the current position
	 */
	public boolean driveToPose(Pose2d pose) {
		double x = pose.getX();
		double y = (redAlliance ? -pose.getY() : pose.getY());
		double angle = pose.getRotation().getDegrees();

		double xDiff = x - getPose().getX();
		double yDiff = y - getPose().getY();
		double aDiff = angle - getPose().getRotation().getDegrees();

		if (aDiff > AutoConstants.DEG_180) {
			aDiff -= AutoConstants.DEG_360;
		} else if (aDiff < -AutoConstants.DEG_180) {
			aDiff += AutoConstants.DEG_360;
		}

		System.out.println(aDiff);

		double xSpeed;
		double ySpeed;
		if (Math.abs(xDiff) > Math.abs(yDiff)) {
			xSpeed = clamp(xDiff / AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
			ySpeed = xSpeed * (yDiff / xDiff);
			if (Math.abs(xDiff) < AutoConstants.CONSTANT_SPEED_THRESHOLD && Math.abs(yDiff)
				< AutoConstants.CONSTANT_SPEED_THRESHOLD) {
				xSpeed = (AutoConstants.CONSTANT_SPEED_THRESHOLD * xDiff / Math.abs(xDiff))
					/ AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT;
				ySpeed = xSpeed * (yDiff / xDiff);
			}
		} else {
			ySpeed = clamp(yDiff / AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT,
			-AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_SPEED_METERS_PER_SECOND);
			xSpeed = ySpeed * (xDiff / yDiff);
			if (Math.abs(xDiff) < AutoConstants.CONSTANT_SPEED_THRESHOLD && Math.abs(yDiff)
				< AutoConstants.CONSTANT_SPEED_THRESHOLD) {
				ySpeed = (AutoConstants.CONSTANT_SPEED_THRESHOLD * yDiff / Math.abs(yDiff))
					/ AutoConstants.AUTO_DRIVE_TRANSLATIONAL_SPEED_ACCEL_CONSTANT;
				xSpeed = ySpeed * (xDiff / yDiff);
			}
		}

		xSpeed = Math.abs(xDiff) > AutoConstants.AUTO_DRIVE_METERS_MARGIN_OF_ERROR
			? xSpeed : 0;
		ySpeed = Math.abs(yDiff) > AutoConstants.AUTO_DRIVE_METERS_MARGIN_OF_ERROR
			? ySpeed : 0;
		double aSpeed = Math.abs(aDiff) > AutoConstants.AUTO_DRIVE_DEGREES_MARGIN_OF_ERROR
			? (aDiff > 0 ? Math.min(AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.AUTO_DRIVE_ANGULAR_SPEED_ACCEL_CONSTANT) : Math.max(
			-AutoConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, aDiff
			/ AutoConstants.AUTO_DRIVE_ANGULAR_SPEED_ACCEL_CONSTANT)) : 0;

		drive(xSpeed, ySpeed, aSpeed, true);
		if (xSpeed == 0 && ySpeed == 0 && aSpeed == 0) {
			return true;
		}
		return false;
	}


	/**
	 * @param id Id of the tag we are positioning towards.
	 * Positions the robot to the correct distance from the speaker to shoot
	 */
	public void alignToSpeaker(int id) {
		if (rpi.getAprilTagX(id) != VisionConstants.UNABLE_TO_SEE_TAG_CONSTANT) {
			resetPose(new Pose2d(rpi.getAprilTagZ(id), rpi.getAprilTagX(id),
				new Rotation2d(rpi.getAprilTagXInv(id))));
		}
		double yDiff = odometry.getPoseMeters().getY();
		double xDiff = odometry.getPoseMeters().getX() - VisionConstants.SPEAKER_TARGET_DISTANCE;
		double aDiff = odometry.getPoseMeters().getRotation().getRadians();

		double xSpeed = Math.abs(xDiff) > VisionConstants.X_MARGIN_TO_SPEAKER
			? clamp(xDiff / VisionConstants.SPEAKER_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND) : 0;
		double ySpeed = Math.abs(yDiff) > VisionConstants.Y_MARGIN_TO_SPEAKER
			? clamp(yDiff / VisionConstants.SPEAKER_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND) : 0;
		double aSpeed = Math.abs(aDiff) > VisionConstants.ROT_MARGIN_TO_SPEAKER
			? -clamp(aDiff / VisionConstants.SPEAKER_ROTATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND) : 0;

		double xSpeedField = (xSpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			+ (ySpeed * Math.sin(Math.toRadians(tagOrientationAngles[id])));
		double ySpeedField = (ySpeed * Math.cos(Math.toRadians(tagOrientationAngles[id])))
			- (xSpeed * Math.sin(Math.toRadians(tagOrientationAngles[id])));
		if (xSpeedField == 0 && ySpeedField == 0) {
			isSpeakerPositionAligned = true;
		}
		if (!isSpeakerPositionAligned) {
			drive(xSpeedField, ySpeedField, aSpeed, true);
		} else {
			drive(0, 0, aSpeed, true);
			if (aSpeed == 0) {
				isSpeakerAligned = true;
			}
		}
	}
	/**
	 * Positions the robot to align to the closest note detected.
	 */
	public void alignToNote() {
		double xDiff = rpi.getNoteDistance();
		double aDiff = rpi.getNoteYaw();
		System.out.println(xDiff);

		double ySpeed = 0;
		double xSpeed = Math.abs(xDiff) > VisionConstants.X_MARGIN_TO_NOTE ? clamp(xDiff
			/ VisionConstants.NOTE_TRANSLATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_SPEED_METERS_PER_SECOND,
			VisionConstants.MAX_SPEED_METERS_PER_SECOND) : 0;
		double aSpeed = Math.abs(aDiff) > VisionConstants.ROT_MARGIN_TO_NOTE
			? -clamp(aDiff / VisionConstants.NOTE_ROTATIONAL_ACCEL_CONSTANT,
			-VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
			VisionConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND) : 0;

		//System.out.println(aSpeed);
		//SmartDashboard.putNumber("yaw", rpi.getNoteYaw());

		if (!isNoteAligned) {
			drive(0, ySpeed, aSpeed, false);
			if (aSpeed == 0) {
				System.out.println("HERE");
				drive(xSpeed, ySpeed, 0, false);
				if (xSpeed == 0) {
					isNoteAligned = true;
				}
			}
		}
		if (isNoteAligned) {
			System.out.println("stopped note alignment");
			drive(0, 0, 0, false);
		}
	}

	/**
	 * Sets the formation of the swerve wheels in an X.
	 */
	public void setXFormation() {
		frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	/**
	 * Sets the formation of the swerve wheels to be forward.
	 */
	public void setForwardFormation() {
		frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
		rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
	}

	/**
	 * Returns whether the note is aligned at the time of the function call.
	 * @return isNoteAligned
	 */
	public boolean noteAligned() {
		return isNoteAligned;
	}

	/**
	 * Returns whether the speaker is aligned at the time of the function call.
	 * @return isSpeakerAligned
	 */
	public boolean speakerAligned() {
		return isSpeakerAligned;
	}

	/**
	 * Resets alignment constants during auto paths upon alignment command ending.
	 */
	public void autoResetAlignment() {
		lockedSpeakerId = -1;
		isSpeakerAligned = false;
		isNoteAligned = false;
		isSpeakerPositionAligned = false;
	}


	/**
	 * Driving robot relative given relative chassis speeds.
	 * @param robotRelSpeeds relative chassis speeds
	 */
	public void driveRobotRelative(ChassisSpeeds robotRelSpeeds) {
		odometry.update(Rotation2d.fromDegrees(getHeading()),
			new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()});
		SmartDashboard.putNumber("X Pos", getPose().getX());
		SmartDashboard.putNumber("Y Pos", getPose().getY());
		SmartDashboard.putNumber("Heading", getHeading());

		var swerveModuleStates =
			DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(robotRelSpeeds);

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[(2 + 1)]);
	}

	/**
	 * get the relative robot speeds.
	 * @return chassis speeds of swerve modules
	 */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		SwerveModuleState[] swerveStates =
			new SwerveModuleState[]{
				frontLeft.getState(), frontRight.getState(),
				rearLeft.getState(), rearRight.getState()};

		return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(swerveStates);
	}
	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
			desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[2 + 1]);
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return (Rotation2d.fromDegrees(-gyro.getAngle())).getDegrees();
	}

	/**
	 * Clamps a double between two values.
	 * @param value the value wished to be bounded
	 * @param lowerBound the lower limit for the value
	 * @param upperBound the upper limit for the value
	 * @return the clamped value of the double
	 */
	public static double clamp(double value, double lowerBound, double upperBound) {
		return Math.min(Math.max(value, lowerBound), upperBound);
	}
}
