package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;


import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class TransferSample {

    public static Scheduler TransferSampleActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Intake.Block();
                    Outtake.openClaw();
                    Outtake.setExtensionPos(0);
                    Outtake.OverHead_TAKESAMPLE();
                    Intake.RotateToStore();
                    Extendo.state = Extendo.ExtendoStates.RETRACTING;
                    Intake.DropUp();
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            })
            .waitSeconds(0.3)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Intake.RotateToEject();
                    Lift.CustomPowerToMotors(-0.5);
                }

                @Override
                protected boolean Conditions() {
                    return Extendo.getPosition() < 20 && Lift.getPosition() < 20;
                }
            })
            .waitSeconds(0.15)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Intake.StopSpinner();
                    Outtake.setExtensionPos(0.3);
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.OverHeadDoneness();
                }
            })
            .waitSeconds(0.15)
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.closeClaw();
                }
                @Override
                protected boolean Conditions() {
                    return Outtake.IsClawDone();
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.OFF;
                }
                @Override
                protected boolean Conditions() {
                    return true;
                }
            });
}
