package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import android.util.Pair;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Iterator;

public class NotifierSubsystem extends SubsystemBase { // TODO This might be bettter in FTClib
    private ElapsedTime timer;

    ArrayList<Pair<Double, InstantCommand>> notifications = new ArrayList<Pair<Double, InstantCommand>>(); // TODO Maybe Runable instead of Instant Command

    public NotifierSubsystem(ElapsedTime timer) {
        this.timer = timer;
    }

    public void addNotification(double time, InstantCommand command) { // TODO Maybe Runable instead of Instant Command
        notifications.add(new Pair<Double, InstantCommand>((Double) time, command)); // TODO Maybe Runable instead of Instant Command
    }

    @Override
    public void periodic() {
        // Check through times and notify the driver TODO: TEST THIS

        Iterator<Pair<Double, InstantCommand>> iterator = notifications.iterator();
        while (iterator.hasNext()) {
            Pair<Double, InstantCommand> pair = iterator.next();
            if (timer.seconds() >= pair.first) {
                pair.second.schedule();
                iterator.remove();
            }
        }
    }
}
