/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AimByLimelight;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.MoveIntake.IntakePosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterAngle;
import frc.robot.subsystems.shooter.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShintakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final ShintakeSubsystem m_shintakeSubsystem = new ShintakeSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        m_driveSubsystem.setDefaultCommand(
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            new RunCommand(() -> m_driveSubsystem
                .driveRobot(-m_driverController.getRawAxis(OIConstants.kDriverYAxis),
                    m_driverController.getRawAxis(OIConstants.kDriverTurnAxis)
                ), m_driveSubsystem
            )
        );

        m_shintakeSubsystem.setDefaultCommand(
            new RunCommand(() -> m_shintakeSubsystem
                .drive(m_operatorController.getRawAxis(OIConstants.kOperatorLeftTrigger) 
                    - m_operatorController.getRawAxis(OIConstants.kOperatorRightTrigger)
                ), m_shintakeSubsystem
            )
        );

        m_elevatorSubsystem.setDefaultCommand(
            new RunCommand(() -> m_elevatorSubsystem
                .moveElevator(
                    -getOutput(0.25, 1, m_operatorController.getRawAxis(OIConstants.kOperatorLeftJoystick))
                ), m_elevatorSubsystem
            )
        );

        m_intakeSubsystem.setDefaultCommand(
            new RunCommand(() -> m_intakeSubsystem
                .drive(
                    getOutput(0.1, 1, m_operatorController.getRawAxis(OIConstants.kOperatorRightJoystick))
                ), m_intakeSubsystem
            )
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        new JoystickButton(m_operatorController, 2)
            .whileActiveOnce(
                new RunCommand(() -> {
                    m_shooterSubsystem.startShooter();
                    System.out.println("start");
                }, m_shooterSubsystem
            ), true)
            .whenInactive(
                new RunCommand(() -> {
                    m_shooterSubsystem.stopShooter();
                    System.out.println("stop");
                }, m_shooterSubsystem
            ), true);

        new JoystickButton(m_driverController, 6).whileActiveContinuous(new AimByLimelight(m_driveSubsystem), true);
        
        new JoystickButton(m_operatorController, 3)
            .whenPressed(
                new RunCommand(() -> {
                        m_elevatorSubsystem.resetPosition();
                    }, m_elevatorSubsystem
                )
            );

        new JoystickButton(m_operatorController, 4)
            .whenPressed(
                new RunCommand(() -> {
                        m_shooterSubsystem.toggleShooterAngle(ShooterAngle.UP);
                    }, m_shooterSubsystem
                ),
            true);

        new JoystickButton(m_operatorController, 1)
            .whenPressed(
                new RunCommand(() -> {
                        m_shooterSubsystem.toggleShooterAngle(ShooterAngle.DOWN);
                    }, m_shooterSubsystem
                ),
            true);


        Button dpadUp = new Button(() -> m_operatorController.getPOV() == 0);
        Button dpadDown = new Button(() -> m_operatorController.getPOV() == 180);
        dpadUp.whenPressed(new MoveIntake(m_intakeSubsystem, IntakePosition.UP));
        dpadDown.whenPressed(new MoveIntake(m_intakeSubsystem, IntakePosition.DOWN));

    }

    public void teleopInit(){
        m_shooterSubsystem.toggleShooterAngle(ShooterAngle.DOWN);
        m_elevatorSubsystem.resetPosition();
        // m_elevatorSubsystem.setPos(ElevatorConstants.kHomePosition);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
        
        // Create a voltage constraint to ensure we don't accelerate too fast
        // var autoVoltageConstraint =
        // new DifferentialDriveVoltageConstraint(
        //     new SimpleMotorFeedforward(DriveConstants.ksVolts,
        //                             DriveConstants.kvVoltSecondsPerMeter,
        //                             DriveConstants.kaVoltSecondsSquaredPerMeter),
        //     DriveConstants.kDriveKinematics,
        //     10);

        // // Create config for trajectory
        // TrajectoryConfig config =
        //     new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
        //                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //         // Add kinematics to ensure max speed is actually obeyed
        //         .setKinematics(DriveConstants.kDriveKinematics)
        //         // Apply the voltage constraint
        //         .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //     // Start at the origin facing the +X direction
        //     new Pose2d(0, 0, new Rotation2d(0)),
        //     // Pass through these two interior waypoints, making an 's' curve path
        //     List.of(
        //         new Translation2d(1, 1),
        //         new Translation2d(2, -1)
        //     ),
            // End 3 meters straight ahead of where we started, facing forward
            // new Pose2d(3, 0, new Rotation2d(0)),
            // // Pass config
            // config
        // );

        // RamseteCommand ramseteCommand = new RamseteCommand(
        //     exampleTrajectory,
        //     m_robotDrive::getPose,
        //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        //     new SimpleMotorFeedforward(DriveConstants.ksVolts,
        //                             DriveConstants.kvVoltSecondsPerMeter,
        //                             DriveConstants.kaVoltSecondsSquaredPerMeter),
        //     DriveConstants.kDriveKinematics,
        //     m_robotDrive::getWheelSpeeds,
        //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //     new PIDController(DriveConstants.kPDriveVel, 0, 0),
        //     // RamseteCommand passes volts to the callback
        //     m_robotDrive::tankDriveVolts,
        //     m_robotDrive
        // );

        // // Run path following command, then stop at the end.
        // return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    // }
    public static double getOutput(double deadband, double maxOutput, double axis) {
		double output;
		if (Math.abs(axis) < deadband) {
			output = 0;
		} else {
			double motorOutput = (((Math.abs(axis) - deadband) / (1 - deadband)) * (axis / Math.abs(axis)));
			output = motorOutput * maxOutput;
		}
		return output;
	}
}

