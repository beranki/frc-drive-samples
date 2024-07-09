package frc.robot.commands.mech;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.MechConstants;
import frc.robot.systems.MBRFSMv2;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;

public class RevShooterTimed extends Command {
	private CANSparkMax shooterLeftMotor;
	private CANSparkMax shooterRightMotor;
	private Encoder throughBore;
	private Timer timer;
	private double timeShooting;
	private MBRFSMv2 mbrFSM;

	/**
	 * Makes a command that shoots the note out.
	 * @param timeShooting how much time the shooter is expected to rev for
	 */
	public RevShooterTimed(MBRFSMv2 mbrFSM, double timeShooting) {
		// Use addRequirements() here to declare subsystem dependencies.

		//throughBore = new Encoder(0, 1);
		//throughBore.reset();

		this.mbrFSM = mbrFSM;

		timer = new Timer();
		this.timeShooting = timeShooting;

		//addRequirements(...);

	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		if (timer.get() == 0) {
			timer.start();
		}
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		mbrFSM.setShooterLeftMotorPower(MechConstants.SHOOTING_POWER);
		mbrFSM.setShooterRightMotorPower(MechConstants.SHOOTING_POWER);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		mbrFSM.setShooterLeftMotorPower(0);
		mbrFSM.setShooterRightMotorPower(0);
		timer.stop();
		timer.reset();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > timeShooting;
	}

	private double pidAuto(double currentEncoderPID, double targetEncoder) {
		double correction = MechConstants.PID_CONSTANT_PIVOT_P_AUTO
			* (targetEncoder - currentEncoderPID);

		return Math.min(MechConstants.MAX_TURN_SPEED_AUTO,
			Math.max(MechConstants.MIN_TURN_SPEED_AUTO, correction));
	}

	private boolean inRange(double a, double b) {
		return Math.abs(a - b) < MechConstants.INRANGE_VALUE; //EXPERIMENTAL
	}
}
