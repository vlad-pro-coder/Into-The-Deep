package org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples;

import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.AfterThirdSampleGoToSubmersible.AfterThirdSampleGoToSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.FailedToDetect.FailedToDetectActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeCachedSample.TakeCachedSample;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.ApproachingSubmersibleActions.TakeSubmersibleSampleToHighBasket.GoingBackAndForthToHighBasketAndSubmersibleActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample1pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample2pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.CHASSIS_sample3pos;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample1;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample2;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.EXTENDO_sample3;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.OVERHEAD_idle;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.AutoConstants.orderedSamples;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.PutLastSamplePredefined.PutLastSamplePredefinedActions;
import static org.firstinspires.ftc.teamcode.IntoTheDeep.Autonomous.Samples.FOURpreloadsLogic.SimultaniosSampleGrabingScoring.SimultaniosSampleGrabingScoringActions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.IntoTheDeep.CameraPipelines.YellowSampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Extendo;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Intake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Lift;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.Outtake;
import org.firstinspires.ftc.teamcode.IntoTheDeep.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.IntoTheDeep.Wrapers.Logitech920;

@Autonomous
@Config
public class Samples extends LinearOpMode {
    public enum RobotStates{
        CRUISETOSUBMERSIBLE,
        OCCUPIEDPRELOADS,
        DETECT,
        FAILEDDETECTION,
        GRABFROMSUBMERSIBLE,
        SUBMERSIBLESAMPLECYCLE,
    }
    public boolean WereInstructionGiven = false;
    public RobotStates robotstate = RobotStates.OCCUPIEDPRELOADS;
    public boolean everpurepersuit = false;
    Logitech920 camera;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.InitializeForOperation();
        camera  = new Logitech920("logitik",hardwareMap);
        camera.startCamera();
        Scheduler tasks = new Scheduler();
        /*tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample1pos,EXTENDO_sample1,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample2pos,EXTENDO_sample2,1));
        tasks.AddAnotherScheduler(SimultaniosSampleGrabingScoringActions(CHASSIS_sample3pos,EXTENDO_sample3,1));
        tasks.AddAnotherScheduler(PutLastSamplePredefinedActions);*/
        tasks.AddAnotherScheduler(AfterThirdSampleGoToSubmersibleActions);
        //tasks.AddAnotherScheduler(GoingBackAndForthToHighBasketAndSubmersibleActions);

        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;
        Lift.DoingAuto = true;
        Extendo.DoingAuto = true;

        while(opModeInInit()){
            RobotInitializers.clearCache(false);
            Outtake.setOverHeadPos(OVERHEAD_idle);
            Intake.Block();
            Outtake.openClaw();

            Lift.update();
            Extendo.update();
            Outtake.update();

        }

        waitForStart();

        while(opModeIsActive())
        {
            RobotInitializers.clearCache();

            RobotInitializers.Dashtelemetry.addData("is Scheduler done", tasks.tasks.size());

            RobotInitializers.Dashtelemetry.addData("location done",Chassis.IsPositionDone(500));
            RobotInitializers.Dashtelemetry.addData("angle done",Chassis.IsHeadingDone(30));

            switch (robotstate){
                case OCCUPIEDPRELOADS:
                    if(tasks.IsSchedulerDone())
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                    break;
                case CRUISETOSUBMERSIBLE:
                    if(tasks.IsSchedulerDone())
                    {
                        if(orderedSamples.isEmpty())
                            robotstate = RobotStates.DETECT;
                        else
                            robotstate = RobotStates.GRABFROMSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;
                case DETECT:
                    if(!WereInstructionGiven){
                        tasks.waitForStill();
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone())//and no blur
                    {
                        orderedSamples = camera.yellow.getSamplesPrioritized();
                        robotstate = RobotStates.GRABFROMSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;

                case GRABFROMSUBMERSIBLE:
                    YellowSampleDetectionPipeline.SamplePoint bestsample = orderedSamples.poll();
                    if(!WereInstructionGiven){
                        assert bestsample != null;
                        tasks.AddAnotherScheduler(TakeCachedSample(bestsample,1));
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        if(Intake.HasMixedTeamPiece() && Intake.SampleReachedTrapDoor())
                            robotstate = RobotStates.SUBMERSIBLESAMPLECYCLE;
                        else
                            robotstate = RobotStates.FAILEDDETECTION;
                        WereInstructionGiven = false;
                    }
                    break;
                case FAILEDDETECTION:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(FailedToDetectActions);
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.DETECT;
                        WereInstructionGiven = false;
                    }
                    break;
                case SUBMERSIBLESAMPLECYCLE:
                    if(!WereInstructionGiven){
                        tasks.AddAnotherScheduler(GoingBackAndForthToHighBasketAndSubmersibleActions);
                        WereInstructionGiven = true;
                    }
                    if(tasks.IsSchedulerDone()){
                        robotstate = RobotStates.CRUISETOSUBMERSIBLE;
                        WereInstructionGiven = false;
                    }
                    break;
            }

            Chassis.update();
            Lift.update();
            Extendo.update();
            Outtake.update();
            tasks.update();
            Localizer.Update();

        }

    }
}
