enum mode {IDLE, PWM, ITEST, HOLD, TRACK};

void set_mode(enum mode m);
enum mode get_mode();
