// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonCMDs.AUTONgroups;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCMDs.ArmMoveCMD;
import frc.robot.commands.AutonCMDs.AUTOWaitCMD;
import frc.robot.commands.AutonCMDs.AUTOwaitUntilPIDtargetCMD;
import frc.robot.commands.GrabberCMDs.GrabberCloseCMD;
import frc.robot.commands.GrabberCMDs.GrabberOpenCMD;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAUTON extends SequentialCommandGroup {
  Arm armSubsystem;
  Intake intakeSubsystem;
  GrabberSubsystem grabberSubsystem;
  /** Creates a new TestAUTON. */
  public TestAUTON(Arm arm_subsystem, Intake intake_subsystem, GrabberSubsystem grabber_subsystem) {
    armSubsystem = arm_subsystem;
    intakeSubsystem = intake_subsystem;
    grabberSubsystem = grabber_subsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new ArmMoveCMD(ArmConstants.kDownPos, armSubsystem, intakeSubsystem),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, 5*8100/360),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new ArmMoveCMD(ArmConstants.kMiddlePos, armSubsystem, intakeSubsystem),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, 5*8100/360),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new ArmMoveCMD(ArmConstants.kShelfPos, armSubsystem, intakeSubsystem),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, 5*8100/360),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new ArmMoveCMD(ArmConstants.kTopPos, armSubsystem, intakeSubsystem),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, 5*8100/360),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new ArmMoveCMD(ArmConstants.kStowedPos, armSubsystem, intakeSubsystem),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOwaitUntilPIDtargetCMD(armSubsystem.armPID, armSubsystem.ArmEncoder, 5*8100/360),
      new InstantCommand(() -> DriverStation.reportWarning("Step", false)),
      new AUTOWaitCMD(10),

      new GrabberCloseCMD(grabberSubsystem),
      new AUTOWaitCMD(5),
      new GrabberOpenCMD(grabberSubsystem)
      );
  }
}