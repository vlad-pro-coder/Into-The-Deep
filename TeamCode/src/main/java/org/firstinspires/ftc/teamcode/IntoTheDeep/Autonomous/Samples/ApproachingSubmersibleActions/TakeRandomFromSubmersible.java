package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;

import java.util.Random;

public class TakeRandomFromSubmersible {
    public static Scheduler TakeRandomFromSubmersibleActions = new Scheduler()
            .addTask(new Task() {

                @Override
                protected void Actions() {
                    Random rand  = new Random();
                    int max = 30;
                    int min = -30;
                    int randomNum = rand.nextInt((max-min)+1)+min;
                    Chassis.setHeading(Math.toRadians(randomNum));
                }

                @Override
                protected boolean Conditions() {
                    return true;
                }
            });
}
