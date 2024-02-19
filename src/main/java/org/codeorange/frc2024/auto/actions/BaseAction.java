package org.codeorange.frc2024.auto.actions;

public interface BaseAction {
    default void start() {};
    default void update() {};
    default void done() {};
    default boolean isFinished() {return true;};
}
