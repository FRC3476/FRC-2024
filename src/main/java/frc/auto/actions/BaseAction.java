package frc.auto.actions;

public interface BaseAction {
    default void start() {};
    default void update() {};
    default void done() {};
    default boolean isFinished() {return true;};
}
