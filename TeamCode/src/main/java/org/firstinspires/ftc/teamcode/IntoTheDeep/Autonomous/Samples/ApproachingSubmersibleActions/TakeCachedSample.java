package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.GetExtendoTicksToTravelAndNeededAngleFromSample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.MathHelpers.GetSamplePosition.getExtendoRotPair;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;

public class TakeCachedSample {
    public static Scheduler TakeCachedSampleActions(YellowSampleDetectionPipeline.SamplePoint Sample, double timeout){
        SparkFunOTOS.Pose2D data = getExtendoRotPair(Sample.x,Sample.y);
        return new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
                        Chassis.setHeading(data.h);
                        RobotLog.ii("taken sample"," x: " + Sample.x + " y: " + Sample.y);
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsHeadingDone(2);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.setExtendoPos(Math.max(data.x-67,250));
                    }

                    @Override
                    protected boolean Conditions() {
                        return Extendo.IsExtendoDone(60);
                    }
                })
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Intake.DropDown();
                        Intake.RotateToStore();
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.1)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Extendo.CustomPowerToMotors(0.5);
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
                        return Intake.HasMixedTeamPiece() || r;
                    }
                });
    }
}
