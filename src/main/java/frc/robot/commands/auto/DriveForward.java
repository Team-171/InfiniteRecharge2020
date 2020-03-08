/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForward extends CommandBase {

    private double distance = 60;
    private DriveSubsystem m_driveSubsystem;

    public DriveForward(DriveSubsystem driveSubsystem) {
        addRequirements(driveSubsystem);
        m_driveSubsystem = driveSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveSubsystem.resetEncoders();
        m_driveSubsystem.setTargetDistance(distance);
        m_driveSubsystem.setRelativeTargetAngle(0);
        m_driveSubsystem.setPIDMode(DriveSubsystem.PIDMode.DIST_WITH_TURN);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.setPIDMode(DriveSubsystem.PIDMode.NONE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_driveSubsystem.isOnTarget();
    }
}
