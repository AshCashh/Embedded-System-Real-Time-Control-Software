typedef enum {
    IDLE,
    RUNNING,
    STOP,
    ESTOP
} MotorState_t;

typedef struct {
  uint16_t upper;
  uint16_t lower;
} limits_t;

typedef struct Motor {
    uint16_t desired_rpm;
    limits_t current_limit;
    limits_t acceleration_limit;
    MotorState_t MotorState;
} Motor_t;