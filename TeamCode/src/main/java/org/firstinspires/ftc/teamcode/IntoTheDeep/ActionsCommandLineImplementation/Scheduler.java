package org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation;

import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;

import java.util.LinkedList;
import java.util.Queue;

public class Scheduler {

    public Queue<Task> tasks;

    public Scheduler(){
            tasks = new LinkedList<>();
    }

    public Scheduler addTask(Task t){
        tasks.add(t);
        return this;
    }
    public boolean IsSchedulerDone(){
        return tasks.isEmpty();
    }
    public Scheduler waitForStill(){
        addTask(new Task() {
            @Override
            protected void Actions() {

            }
            @Override
            protected boolean Conditions() {
                return Localizer.getVelocity().x < 10 && Localizer.getVelocity().y < 10 && Localizer.getVelocity().h < Math.toRadians(5);
            }
        });
        return this;
    }

    public Scheduler waitSeconds(double sec){
        addTask(new Task() {
            private long wait;
            private long track;

            @Override
            protected void Actions() {
                wait = (long) (sec * 1000);
                track = -1;
            }

            @Override
            protected boolean Conditions() {
                if (track == -1) {
                    track = System.currentTimeMillis();
                    return false;
                }

                boolean r = (System.currentTimeMillis() - track) >= wait;
                if (r) track = -1;
                return r;
            }
        });
        return this;
    }

    public Scheduler AddAnotherScheduler(Scheduler scheduler){
        tasks.addAll(scheduler.tasks);
        return this;
    }
    public void clear(){
        Task t = tasks.peek();
        if(t != null)
            t.RanOnce = false;
        tasks.clear();
    }

    public void update(){
        if(IsSchedulerDone())
            return;
        Task t = tasks.peek();
        assert t != null;

        boolean result = t.Run();
        if(result) {
            tasks.poll();
        }
    }
}
