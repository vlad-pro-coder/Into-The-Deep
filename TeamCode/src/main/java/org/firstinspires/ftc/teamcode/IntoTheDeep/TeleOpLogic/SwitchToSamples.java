package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class SwitchToSamples {

    public static Scheduler SwitchToSampsActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.FREEWILL;
                    Outtake.OverHead_TAKESAMPLE();
                    Outtake.openClaw();
                    Lift.setLiftPos(100);
                }
                @Override
                protected boolean Conditions() {
                    return Lift.getPosition() > 50 && Outtake.OverHeadDoneness();
                }
            }).addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.CustomPowerToMotors(-0.5);
                }
                @Override
                protected boolean Conditions() {
                    return Lift.getPosition() < 20;
                }
            }).addTask(new Task() {
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
