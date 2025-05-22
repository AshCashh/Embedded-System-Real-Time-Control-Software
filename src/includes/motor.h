typedef enum {
    IDLE,
    RUNNING,
    STOP,
    ESTOP
} MotorState_t;

typedef struct Motor {
    float desiredRPM, averageRPM, lastRPM, avgCurrent;
    MotorState_t MotorState;
} Motor_t;