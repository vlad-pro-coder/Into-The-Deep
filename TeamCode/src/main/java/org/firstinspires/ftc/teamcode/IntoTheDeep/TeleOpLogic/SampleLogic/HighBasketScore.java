package org.firstinspires.ftc.teamcode.IntoTheDeep.TeleOpLogic.SampleLogic;


import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;

public class HighBasketScore {
    public static Scheduler HighBasketScoreActions = new Scheduler()
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Lift.state = Lift.LIFTSTATES.HIGHBASKET;
                    Outtake.OverHead_BASKETMOVINGSAFEPOS();
                    Lift.setLiftPos(Lift.HighBasketPos);
                }

                @Override
                protected boolean Conditions() {
                    return Lift.getPosition() > Lift.HighBasketPos - 200 && Outtake.OverHeadDoneness(30);
                }
            })
            .addTask(new Task() {
                @Override
                protected void Actions() {
                    Outtake.OverHead_OVERBASKET();
                }

                @Override
                protected boolean Conditions() {
                    return Outtake.OverHeadDoneness();
                }
            });
}
