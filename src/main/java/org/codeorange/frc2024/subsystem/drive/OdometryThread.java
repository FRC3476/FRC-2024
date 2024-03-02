package org.codeorange.frc2024.subsystem.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * @author 6328
 */
public class OdometryThread extends Thread {
    private final Lock signalsLock = new ReentrantLock();
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static OdometryThread INSTANCE;
    public static OdometryThread getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new OdometryThread();
        }
        return INSTANCE;
    }

    private OdometryThread() {
        setName("OdometryThread");
        setDaemon(true);
    }

    @Override
    public void start() {
        if(timestampQueues.size() > 1) {
            super.start();
        }
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        signalsLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
            queues.add(queue);
        } finally {
            signalsLock.unlock();
            Drive.odometryLock.unlock();
        }

        return queue;
    }

    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            Drive.odometryLock.unlock();
        }
        return queue;
    }

    @Override
    public void run() {
        while(true) {
            signalsLock.lock();
            try {
                Thread.sleep((long) 4);
                if(signals.length > 0) BaseStatusSignal.refreshAll(signals);
            } catch (InterruptedException ignored) {
            } finally {
                signalsLock.unlock();
            }

            Drive.odometryLock.lock();
            try {
                double timestamp = Logger.getRealTimestamp() * 1e-6;
                double totalLatency = 0.0;
                for (BaseStatusSignal signal : signals) {
                    totalLatency += signal.getTimestamp().getLatency();
                }
                if (signals.length > 0) {
                    timestamp -= totalLatency / signals.length;
                }
                for (int i = 0; i < signals.length; i++) {
                    queues.get(i).offer(signals[i].getValueAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            } finally {
                Drive.odometryLock.unlock();
            }
        }
    }
}
