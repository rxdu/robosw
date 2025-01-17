# Swerve-drive Robot

## SBot Controller Application

The SBot Controller is a motion controller for the WsSbotBase, a custom-built robot with WaveShare servos and motors.
The application initializes the SbotSystem and keeps it running until the user stops the application. 

```mermaid
sequenceDiagram
    participant ControlThread
    participant MainThread
    participant EventThread

    MainThread ->> EventThread: Initialize
    activate EventThread
    MainThread ->> ControlThread: Initialize
    activate ControlThread
    EventThread -->> MainThread: JoystickEvent
    loop MainLoop
        MainThread->>MainThread: Handle HRI/FSM
    end
    MainThread -->> ControlThread: Command
    loop ControlLoop
        ControlThread->>ControlThread: Update Actuators/Sensors
    end
    ControlThread -->> MainThread: Feedback
    MainThread ->> ControlThread: Stop
    deactivate ControlThread
    MainThread ->> EventThread: Stop
    deactivate EventThread
```

## Hardware Configuration

PocketBeagle hardware configuration:

* steering_motor_port: /dev/ttyS0
* driving_motor_port: /dev/ttyS2
* rc_receiver_port: /dev/ttyS4