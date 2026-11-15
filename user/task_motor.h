/**
 * @file    task_motor.h
 * @brief   モーター制御タスクの初期化関数と実行関数を宣言する。
 */

#ifndef TASK_MOTOR_H
#define TASK_MOTOR_H
#include <trykernel.h>
ER task_motor_init(void);
void task_motor_set_drive(BOOL enable_avoidance, RELTIM duration);
void task_motor_stop_drive(void);
ER task_motor_request_avoidance(void);
ER task_motor_abort_avoidance(void);
void task_motor_observe_range(BOOL valid);
void task_motor(INT stacd, void *exinf);
#endif
