// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Encoder;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.systems.ClimberMechFSMLeft;
import frc.robot.systems.ClimberMechFSMRight;
// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.MBRFSMv2;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	// Systems
	private DriveFSMSystem driveFSMSystem;
	private MBRFSMv2 mbrfsMv2;
	private ClimberMechFSMLeft chainLeftFSM;
	private ClimberMechFSMRight chainRightFSM;
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
		chainLeftFSM = new ClimberMechFSMLeft();
		chainRightFSM = new ClimberMechFSMRight();

		NamedCommands.registerCommand("S_TIN", mbrfsMv2.new IntakeNoteCommand());
		NamedCommands.registerCommand("S_TON", mbrfsMv2.new OuttakeNoteCommand());
		NamedCommands.registerCommand("S_PGS",
			mbrfsMv2.new PivotGroundToShooterCommand());
		NamedCommands.registerCommand("S_PSG",
			mbrfsMv2.new PivotShooterToGroundCommand());
		NamedCommands.registerCommand("S_TRS", mbrfsMv2.new RevShooterCommand());
		NamedCommands.registerCommand("G_SPN", mbrfsMv2.new ShootPreloadedCommand());
		NamedCommands.registerCommand("G_SSN", mbrfsMv2.new ShootNoteCommand());

		/*
		NamedCommands.registerCommand("S_ART", new AprilTagAlign(redSpeakerTagID,
			driveFSMSystem, 0.5));
		NamedCommands.registerCommand("S_ABT", new AprilTagAlign(blueSpeakerTagID,
			driveFSMSystem, 0.5));
		NamedCommands.registerCommand("S_AXN", new NoteAlign(driveFSMSystem,
			0.5));
		*/

		autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData("Auto Chooser", autoChooser);
		SmartDashboard.putData("Field", mField);

		driverCam = CameraServer.startAutomaticCapture(0);
		VideoMode videoMode = new VideoMode(PixelFormat.kMJPEG,256,144,30);
		driverCam.setVideoMode(videoMode);
		driverCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
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
		chainLeftFSM.reset();
		chainRightFSM.reset();
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
		driveFSMSystem.update(input);
		mbrfsMv2.update(input);
		chainLeftFSM.update(input);
		chainRightFSM.update(input);
		mField.setRobotPose(driveFSMSystem.getPose());
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
