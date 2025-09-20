package org.firstinspires.ftc.teamcode.IntoTheDeep.ActionsCommandLineImplementation;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

public class TreeScheduler {

    ArrayList<ArrayList<Task>> tree;
    ArrayList<Integer> BranchCurrentIndex;
    int activeProcesses = 1;

    public TreeScheduler(){
        BranchCurrentIndex = new ArrayList<>(Collections.singletonList(0));
        tree = new ArrayList<ArrayList<Task>>();
    }

    public TreeScheduler addParallelTasks(ArrayList<Task> tasks){
        tree.add(tasks);
        return this;
    }

    public TreeScheduler addTask(Task task){
        tree.add(new ArrayList<>(Collections.singletonList(task)));
        return this;
    }

    public TreeScheduler addAnotherTreeScheduler(TreeScheduler Othertree){
        tree.addAll(Othertree.tree);
        return this;
    }

    public boolean IsSchedulerDone(){
        int mainBranch = BranchCurrentIndex.get(0);
        if (tree.get(mainBranch).size() != BranchCurrentIndex.size()) {
            BranchCurrentIndex = new ArrayList<>(Collections.nCopies(tree.get(mainBranch).size(), mainBranch));
        }
        int leastAdvancedScheduler = Collections.min(BranchCurrentIndex);
        if (leastAdvancedScheduler > 0) {
            tree.subList(0, leastAdvancedScheduler-1).clear();
        }
        for (Integer i: BranchCurrentIndex)
            BranchCurrentIndex.add(i - leastAdvancedScheduler);
        return BranchCurrentIndex.get(0) >= tree.size();
    }

    public int RemainingParallelProcesses(){
        return activeProcesses;
    }

    public void update(){
        if(IsSchedulerDone())
            return;
        activeProcesses = BranchCurrentIndex.size();
        for(int i = BranchCurrentIndex.size()-1;i>=0 ;i--){
            try {
                int index = BranchCurrentIndex.get(i);
                Task task = tree.get(index).get(i);
                assert task != null;

                boolean result = task.Run();
                if (result && i != 0) {
                    BranchCurrentIndex.add(index + 1);
                }
                else if(result && RemainingParallelProcesses() == 1) {
                    BranchCurrentIndex.add(index + 1);
                }
            }catch(Exception ignore){
                RobotLog.ii("scheduler","branch " + i + " has reached its limit ignoring futher instrcutions");
                activeProcesses--;
            }
        }

    }

}
