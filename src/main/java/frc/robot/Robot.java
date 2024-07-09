// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.mechanisms.MechanismState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Encoder;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveConstants.AutoConstants;
import frc.robot.SwerveConstants.DriveConstants;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.MBRFSMv2;
import frc.robot.commands.mech.IntakeNoteTimed;
//Commands
import frc.robot.commands.mech.IntakeNoteUntimed;
import frc.robot.commands.mech.PivotGroundToShooter;
import frc.robot.commands.mech.PivotShooterToGround;
import frc.robot.commands.mech.RevShooterTimed;
import frc.robot.commands.mech.RevShooterUntimed;
import frc.robot.commands.mech.OuttakeNote;
import frc.robot.commands.align.AprilTagAlign;
import frc.robot.commands.align.NoteAlign;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveFSMSystem;
	private MBRFSMv2 mbrfsMv2;
	private SendableChooser<Command> autoChooser;
	private Command autonomousCommand;
	private final Field2d mField = new Field2d();

	private UsbCamera driverCam;
	private UsbCamera chainCam;
	private VideoSink videoSink;
	private MjpegServer driverStream;
	private MjpegServer chainStream;

	private final int streamWidth = 320;
	private final int streamHeight = 240;

	private final int redSpeakerTagID = 4;
	private final int blueSpeakerTagID = 7;
	private Encoder throughBore;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		driveFSMSystem = new DriveFSMSystem();
		mbrfsMv2 = new MBRFSMv2();

		NamedCommands.registerCommand("S_TIN", new IntakeNoteTimed(mbrfsMv2, MechConstants.TIMED_INTAKING_DURATION));
		NamedCommands.registerCommand("S_UIN", new IntakeNoteUntimed(mbrfsMv2));
		NamedCommands.registerCommand("S_PGS", new PivotGroundToShooter(mbrfsMv2));
		NamedCommands.registerCommand("S_PSG", new PivotShooterToGround(mbrfsMv2));
		NamedCommands.registerCommand("S_URS", new RevShooterUntimed(mbrfsMv2));
		NamedCommands.registerCommand("S_TRS", new RevShooterTimed(mbrfsMv2, MechConstants.TIMED_REVVING_DURATION));
		NamedCommands.registerCommand("S_TON", new OuttakeNote(MechConstants.AUTO_SHOOTING_TIME,
			mbrfsMv2));
		NamedCommands.registerCommand("S_ART", new AprilTagAlign(redSpeakerTagID,
			driveFSMSystem, 0.5));
		NamedCommands.registerCommand("S_ABT", new AprilTagAlign(blueSpeakerTagID,
			driveFSMSystem, 0.5));
		NamedCommands.registerCommand("S_AXN", new NoteAlign(driveFSMSystem,
			0.5));

		autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putData("Field", mField);

		//driverCam = CameraServer.startAutomaticCapture(0);
		//driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
		//driverCam.setResolution(streamWidth, streamHeight);

		//Label all named commands here
	}


	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		driveFSMSystem.resetAutonomus();
		autonomousCommand = getAutonomousCommand();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
		CommandScheduler.getInstance().run();
		// driveFSMSystem.updateAutonomous();
		mField.setRobotPose(driveFSMSystem.getPose());
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveFSMSystem.reset();
		mbrfsMv2.reset();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		mbrfsMv2.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {

	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}
}
