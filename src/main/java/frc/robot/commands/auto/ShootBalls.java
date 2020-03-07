/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.shooter.ShintakeSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class ShootBalls extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ShooterSubsystem m_shooterSubsystem;
    private final ShintakeSubsystem m_shintakeSubsystem;

    public ShootBalls(ShooterSubsystem shooterSubsystem, ShintakeSubsystem shintakeSubsystem) {
        m_shooterSubsystem = shooterSubsystem;
        m_shintakeSubsystem = shintakeSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooterSubsystem, shintakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterSubsystem.startShooter();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (m_shooterSubsystem.atSetpoint()) {
            m_shintakeSubsystem.drive(1);
        } else {
            m_shintakeSubsystem.drive(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stopShooter();
        m_shintakeSubsystem.drive(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
