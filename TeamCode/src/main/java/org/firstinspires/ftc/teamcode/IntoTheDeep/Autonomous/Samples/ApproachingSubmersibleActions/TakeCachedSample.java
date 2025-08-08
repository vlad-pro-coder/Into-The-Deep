package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.GetExtendoTicksToTravelAndNeededAngleFromSample;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;

public class TakeCachedSample {
    public static Scheduler TakeCachedSample(YellowSampleDetectionPipeline.SamplePoint Sample, double timeout){
        SparkFunOTOS.Pose2D data = GetExtendoTicksToTravelAndNeededAngleFromSample(new SparkFunOTOS.Pose2D(Sample.x,Sample.y,0));
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                        Chassis.setHeading(data.h);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsHeadingDone(5);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.setExtendoPos(data.x-200);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(30);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.DropDown();
                        Intake.RotateToStore();
                        Extendo.setExtendoPos(data.x);
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .addTask(new Task() {
                    private long wait;
                    private long track;
                    @Override
                    protected void Actions() {
                        wait = (long) (timeout * 1000);
                        track = -1;
                    }

                    @Override
                    protected boolean Conditions() {
                        if (track == -1) {
                            track = System.currentTimeMillis();
                        }

                        boolean r = (System.currentTimeMillis() - track) >= wait;
                        if (r) track = -1;
                        return (Intake.HasMixedTeamPiece() && Intake.SampleReachedTrapDoor()) || r;
                    }
                });
    }
}
