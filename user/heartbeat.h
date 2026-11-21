typedef enum {
    HEARTBEAT_DISCONNECTED, ///< 起動直後、Heartbeat未受信
    HEARTBEAT_CONNECTED, ///< Heartbeat正常受信中
    HEARTBEAT_TIMEOUT ///< 一定時間Heartbeat未受信
} HEARTBEAT_STATE;

void heartbeat_received(void);
ER heartbeat_init(void);
void task_heartbeat(INT stacd, void *exinf);