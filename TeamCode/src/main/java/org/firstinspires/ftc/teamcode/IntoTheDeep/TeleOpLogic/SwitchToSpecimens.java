package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class SwitchToSpecimens {
    public static Scheduler SwitchToSpecsActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.FREEWILL;
                    Outtake.OverHead_TAKESPECIMEN();
                    Outtake.openClaw();
                    Lift.setLiftPos(100);
                }
                @Override
                protected boolean Conditions() {
                    return Lift.getPosition() > 50;
                }
            }).addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.CustomPowerToMotors(-0.5);
                    Outtake.OverHead_TAKESPECIMEN();
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
