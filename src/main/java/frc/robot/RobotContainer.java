/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.EnumSet;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AimAndShootByLimelight;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.MoveIntake.IntakePosition;
import frc.robot.commands.auto.AutoDriveForward;
import frc.robot.commands.auto.DriveForwardAndShoot;
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
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
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
        configureChooser();

        m_driveSubsystem.setDefaultCommand(
            // A split-stick arcade command, with forward/backward controlled by the left
            // hand, and turning controlled by the right.
            new RunCommand(() -> m_driveSubsystem
                .driveRobot(-m_driverController.getRawAxis(OIConstants.kDriverYAxis) * (1 - (0.5 * m_driverController.getRawAxis(OIConstants.kDriverSpeedAxis))),
                    m_driverController.getRawAxis(OIConstants.kDriverTurnAxis) * (1 - (0.5 * m_driverController.getRawAxis(OIConstants.kDriverSpeedAxis)))
                ), m_driveSubsystem
            )
        );

        m_shintakeSubsystem.setDefaultCommand(
            new RunCommand(() -> m_shintakeSubsystem
                .drive(
                    (m_shooterSubsystem.atSetpoint() //if the shooter is at the right rpm
                        ? (m_operatorController.getRawAxis(OIConstants.kOperatorLeftTrigger) //run based on the triggers
                            - m_operatorController.getRawAxis(OIConstants.kOperatorRightTrigger)
                        )
                    : 0                              //else don't run Shintake
                    )
                ), m_shintakeSubsystem
            )
        );

        // m_shintakeSubsystem.setDefaultCommand(
        //     new RunCommand(() -> {
        //         m_shintakeSubsystem
        //             .driveTop(
        //                 (m_shooterSubsystem.atSetpoint() //if the shooter is at the right RPM 
        //                     ? (m_operatorController.getRawButton(OIConstants.kOperatorLeftBumper)  //and the bumper is pressed
        //                         ? -m_operatorController.getRawAxis(OIConstants.kOperatorLeftTrigger)      //run the top shintake in reverse
        //                         : m_operatorController.getRawAxis(OIConstants.kOperatorLeftTrigger)       //else run the top shintake forward
        //                     ) 
        //                     : 0                          //else dont run the shintake
        //                 )
        //             );
                    
        //         m_shintakeSubsystem
        //             .driveBottom(
        //                 (m_shooterSubsystem.atSetpoint() //if the shooter is at the right RPM 
        //                     ? (m_operatorController.getRawButton(OIConstants.kOperatorRightBumper)  //and the bumper is pressed
        //                         ? -m_operatorController.getRawAxis(OIConstants.kOperatorRightTrigger)      //run the bottom shintake in reverse
        //                         : m_operatorController.getRawAxis(OIConstants.kOperatorRightTrigger)       //else run the bottom shintake forward
        //                     ) 
        //                     : 0                          //else dont run the shintake
        //                 )
        //             );
        //         },
        //         m_shintakeSubsystem
        //     )
        // );

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

    public void configureChooser(){
        autoChooser.setDefaultOption("Simple Drive Forward", new AutoDriveForward(m_driveSubsystem));
        autoChooser.addOption("Drive Forward and Shoot", new DriveForwardAndShoot(m_driveSubsystem, m_shooterSubsystem, m_shintakeSubsystem));
        SmartDashboard.putData("Auto Chooser", autoChooser);
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
                }, m_shooterSubsystem
            ), true)
            .whenInactive(
                new RunCommand(() -> {
                    m_shooterSubsystem.stopShooter();
                }, m_shooterSubsystem
            ), true);

        // Turn light on
        // new JoystickButton(m_operatorController, 6).whenPressed(new RunCommand(() -> {
            
        //     System.out.println("ON");
        // }), false).whenReleased(new RunCommand(() -> {
            
        //     System.out.println("OFF");
        // }), false);

        new JoystickButton(m_operatorController, 6)
            .whileActiveContinuous(
                new AimAndShootByLimelight(m_driveSubsystem, m_shooterSubsystem, m_shintakeSubsystem), true);
        
        // Turn light off
        // new JoystickButton(m_operatorController, 6);

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

        SmartRunner.run(() -> {
            m_elevatorSubsystem.setPos(ElevatorConstants.kHomePosition);
            System.out.println("elevator ran");
        }, EnumSet.of(SmartRunner.RunLevel.MATCH));
    }

    public void teleopPeriodic(){
        
    }

    public void autoInit(){
        m_shooterSubsystem.toggleShooterAngle(ShooterAngle.DOWN);
        m_elevatorSubsystem.resetPosition();

        SmartRunner.run(() -> {
            m_elevatorSubsystem.setPos(ElevatorConstants.kHomePosition);
        }, EnumSet.of(SmartRunner.RunLevel.MATCH));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }


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

