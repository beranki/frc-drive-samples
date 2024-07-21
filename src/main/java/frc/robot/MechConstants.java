package frc.robot;

public final class MechConstants {

	public static final float SHOOTING_POWER = 0.8f;
	public static final float AMP_SHOOTER_POWER = 0.1f;
	public static final float AMP_OUTTAKE_POWER = -0.6f; // -0.75
	public static final double AUTO_SHOOTING_TIME = 0.5;
	public static final double AUTO_PRELOAD_SHOOTING_TIME = 1.4;

	public static final float TIMED_INTAKING_DURATION = 1.5f;
	public static final float TIMED_REVVING_DURATION = 2;

	public static final float INTAKE_POWER = 0.2f; //0.4
	public static final float AUTO_INTAKE_POWER = 0.2f;
	public static final float OUTTAKE_POWER = -0.8f;
	public static final float MANUAL_INTAKE_POWER = 0.2f;
	public static final float MANUAL_OUTTAKE_POWER = -0.2f;
	public static final float TELE_HOLDING_POWER = 0.0f;
	public static final float AUTO_HOLDING_POWER = 0.05f;
	public static final int AVERAGE_SIZE = 7;
	public static final float CURRENT_THRESHOLD = 11.0f;
	public static final int NOTE_FRAMES_MIN = 5;

	public static final double MIN_TURN_SPEED = -0.4;
	public static final double MAX_TURN_SPEED = 0.4;
	public static final double MIN_TURN_SPEED_AUTO = -0.6;
	public static final double MAX_TURN_SPEED_AUTO = 0.6;
	public static final double PID_CONSTANT_PIVOT_P = 0.00075;
	public static final double PID_CONSTANT_PIVOT_P_AUTO = 0.001;

	public static final double GROUND_ENCODER_ROTATIONS = -1200;
	public static final double AMP_ENCODER_ROTATIONS = -525;
	public static final double SHOOTER_ENCODER_ROTATIONS = 0;
	public static final double INRANGE_VALUE = 30;

	public static final double PROXIMIIY_THRESHOLD = 200;
	public static final double GREEN_LOW = 0.18;
	public static final double BLUE_LOW = 0.00;
	public static final double RED_LOW = 0.54;

	public static final double GREEN_HIGH = 0.35;
	public static final double BLUE_HIGH = 0.1;
	public static final double RED_HIGH = 0.8;
}
