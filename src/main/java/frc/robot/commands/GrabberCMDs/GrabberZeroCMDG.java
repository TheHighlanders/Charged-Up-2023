// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.GrabberCMDs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.subsystems.GrabberSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GrabberZeroCMDG extends SequentialCommandGroup {
  /** Creates a new GrabberZeroCMDG. */
  GrabberSubsystem grabberSubsystem;
  public GrabberZeroCMDG(GrabberSubsystem grabber_subsystem) {
    grabberSubsystem = grabber_subsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new GrabberCloseCMD(grabberSubsystem),
      // new AUTOWaitCMD(1),
      // new InstantCommand(() -> DriverStation.reportWarning("ZEROcMDG",false)),
      new GrabberZeroCMD(grabberSubsystem)
    );
  }
}
