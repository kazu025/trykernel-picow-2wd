/**
 * @file    task_motor.c
 * @brief   モーター出力を周期的に目標値へ近づけるタスクを実装する。
 *
 * @details
 * 周期ハンドラからセマフォでモータータスクへ通知し、タスク側で
 * motor_ramp_step()を呼び出して出力を段階的に更新する。
 */

#include <trykernel.h>
#include "motor.h"
#include "task_motor.h"
#include "uart_tx.h"

#define MOTOR_PERIOD_MS         50U
#define AVOID_STOP_TICKS       2U   /* 100 ms */
#define AVOID_REVERSE_TICKS    6U   /* 300 ms */
#define AVOID_PIVOT_MAX_TICKS 60U   /* 約3秒 */
#define AVOID_CLEAR_SAMPLES    2U
#define AVOID_DUTY_PERCENT    30U

/** モーター出力更新周期の通知に使用するセマフォID。 */
static ID motor_tick_semid;
static volatile BOOL avoidance_request_pending;
static volatile BOOL avoidance_abort_pending;

typedef enum {
    AVOIDANCE_IDLE,
    AVOIDANCE_STOP,
    AVOIDANCE_REVERSE,
    AVOIDANCE_PIVOT_LEFT,
    AVOIDANCE_WAIT_CLEAR
} AVOIDANCE_STATE;

static AVOIDANCE_STATE avoidance_state;
static UW avoidance_ticks;
static UINT avoidance_clear_samples;
static BOOL avoidance_enabled;
static UW drive_ticks_remaining;
static BOOL drive_expired;
/**
 * @brief 時間指定走行と障害物回避制御の状態を設定する。
 *
 * @param[in] enable_avoidance 障害物回避制御の有効／無効。
 *                             TRUE：有効、FALSE：無効。
 * @param[in] duration         走行継続時間（ms）。
 *                             0を指定した場合は時間制限を設定しない。
 *
 * @details
 * 障害物回避制御の有効／無効を設定し、回避要求フラグ、
 * 回避状態および経過周期数を初期化する。
 *
 * 走行継続時間はモータ制御周期（MOTOR_PERIOD_MS）単位に切り上げ、
 * 初回周期通知までの時間が不定であることを考慮して、
 * さらに1周期を加算して管理する。
 *
 * また、走行時間満了フラグをクリアする。
 *
 * 本関数は走行管理状態の設定のみを行い、
 * モータのPWM出力は直接変更しない。
 *
 * 共有状態の更新中は割り込みを禁止し、処理終了時に
 * 元の割り込み状態へ復元する。
 *
 * @note durationに0を指定すると走行時間の監視を無効にする。
 *       モータを停止する場合は、別途停止処理が必要となる。
 */
void task_motor_set_drive(BOOL enable_avoidance, RELTIM duration)
{
    UINT intsts;
    DI(intsts);
    avoidance_enabled = enable_avoidance;
    avoidance_request_pending = FALSE;
    avoidance_abort_pending = FALSE;
    avoidance_state = AVOIDANCE_IDLE;
    avoidance_ticks = 0U;
    /* 初回の周期通知までの時間は不定なので、1周期余分に数える。 */
    drive_ticks_remaining = duration > 0
        ? ((UW)duration + MOTOR_PERIOD_MS - 1U) / MOTOR_PERIOD_MS + 1U : 0U;
    drive_expired = FALSE;
    EI(intsts);
}
/**
 * @brief 走行管理と障害物回避制御を解除し、モータを即時停止する。
 *
 * @details
 * 時間指定走行の管理を解除し、障害物回避制御の状態を
 * 初期化した後、左右モータのPWM出力を即時停止する。
 *
 * 通常のランプ減速処理を行わず、強制的に停止する。
 *
 * 一連の処理は割り込み禁止区間で実行し、
 * 処理終了時に元の割り込み状態へ復元する。
 *
 * @note 本関数は走行停止処理を行うが、
 *       以降の走行指示を恒久的に禁止するものではない。
 */
void task_motor_stop_drive(void)
{
    UINT intsts;
    DI(intsts);
    task_motor_set_drive(FALSE, 0);
    motor_stop_all();
    EI(intsts);
}

ER task_motor_request_avoidance(void)
{
    UINT intsts;
    DI(intsts);
    if(avoidance_enabled == FALSE){
        EI(intsts);
        return E_OK;
    }
    avoidance_request_pending = TRUE;
    EI(intsts);
    return tk_sig_sem(motor_tick_semid, 1);
}

ER task_motor_abort_avoidance(void)
{
    UINT intsts;
    DI(intsts);
    avoidance_abort_pending = TRUE;
    EI(intsts);
    return tk_sig_sem(motor_tick_semid, 1);
}

/* motor_obstacle_update() と同じ割り込み禁止区間で呼び出す。 */
void task_motor_observe_range(BOOL valid)
{
    if(avoidance_state != AVOIDANCE_PIVOT_LEFT) return;
    if(valid == FALSE) avoidance_abort_pending = TRUE;
    if(motor_obstacle_blocked() == TRUE){
        avoidance_clear_samples = 0U;
    }else if(avoidance_clear_samples < AVOID_CLEAR_SAMPLES){
        avoidance_clear_samples++;
    }
}

static const char *task_motor_avoidance_step(void)
{
    UINT intsts;
    BOOL request;
    BOOL abort;

    DI(intsts);
    request = avoidance_request_pending;
    avoidance_request_pending = FALSE;
    abort = avoidance_abort_pending;
    avoidance_abort_pending = FALSE;
    EI(intsts);

    if((abort != FALSE) && (avoidance_state != AVOIDANCE_IDLE)){
        motor_stop_all();
        task_motor_set_drive(FALSE, 0);
        return "Avoidance: stopped, distance unavailable\r\n";
    }

    if((request != FALSE) && (avoidance_state == AVOIDANCE_IDLE)){
        motor_stop_all();
        avoidance_state = AVOIDANCE_STOP;
        avoidance_ticks = 0U;
        return "Avoidance: stop\r\n";
    }

    switch(avoidance_state){
    case AVOIDANCE_STOP:
        motor_stop_all();
        if(++avoidance_ticks >= AVOID_STOP_TICKS){
            motor_drive_reverse(AVOID_DUTY_PERCENT);
            avoidance_state = AVOIDANCE_REVERSE;
            avoidance_ticks = 0U;
            return "Avoidance: reverse\r\n";
        }
        break;
    case AVOIDANCE_REVERSE:
        if(++avoidance_ticks >= AVOID_REVERSE_TICKS){
            motor_drive_avoidance_left(AVOID_DUTY_PERCENT);
            avoidance_state = AVOIDANCE_PIVOT_LEFT;
            avoidance_clear_samples = 0U;
            avoidance_ticks = 0U;
            return "Avoidance: pivot left\r\n";
        }
        break;
    case AVOIDANCE_PIVOT_LEFT:
        if(avoidance_clear_samples >= AVOID_CLEAR_SAMPLES){
            motor_stop_all();
            avoidance_state = AVOIDANCE_WAIT_CLEAR;
            avoidance_ticks = 0U;
            return "Avoidance: clear path confirmed\r\n";
        }
        if(++avoidance_ticks >= AVOID_PIVOT_MAX_TICKS){
            task_motor_stop_drive();
            return "Avoidance: stopped, pivot timeout\r\n";
        }
        break;
    case AVOIDANCE_WAIT_CLEAR:
        motor_stop_all();
        if(motor_obstacle_blocked() == TRUE){
            task_motor_stop_drive();
            return "Avoidance: stopped, path blocked again\r\n";
        }
        motor_drive_forward(AVOID_DUTY_PERCENT);
        avoidance_state = AVOIDANCE_IDLE;
        return "Avoidance: resume forward\r\n";
    case AVOIDANCE_IDLE:
    default:
        break;
    }
    return NULL;
}

/**
 * @brief 周期通知をモータータスクへ送る。
 * @param[in] exinf 周期ハンドラの拡張情報（この関数では使用しない）。
 * @details 滞留した複数周期分をまとめて1回の通知とし、急な追い付き更新を防ぐ。
 */
static void motor_periodic_handler(void *exinf)
{
    (void)exinf;
    if(drive_ticks_remaining > 0U && --drive_ticks_remaining == 0U){
        drive_expired = TRUE;
    }
    /* 滞留分は1回にまとめ、急な追い付き更新を避ける。 */
    (void)tk_sig_sem(motor_tick_semid, 1);
}

/**
 * @brief モーター更新用セマフォと50 ms周期ハンドラを初期化・開始する。
 * @retval E_OK 周期ハンドラの開始に成功した。
 * @return セマフォまたは周期ハンドラの作成・開始に失敗した場合は、
 *         TRYカーネルのエラーコード。
 */
ER task_motor_init(void)
{
    static const T_CSEM csem = { .sematr = TA_TFIFO, .isemcnt = 0, .maxsem = 1 };
    static const T_CCYC ccyc = {
        .exinf = NULL,
        .cycatr = TA_HLNG, .cychdr = motor_periodic_handler,
        .cyctim = MOTOR_PERIOD_MS, .cycphs = MOTOR_PERIOD_MS,
    };
    ID cycid;
    ER err;
    motor_tick_semid = tk_cre_sem(&csem);
    if(motor_tick_semid < E_OK) return (ER)motor_tick_semid;
    cycid = tk_cre_cyc(&ccyc);
    if(cycid < E_OK){
        return (ER)cycid;
    }
    err = tk_sta_cyc(cycid);
    if(err < E_OK){
        (void)tk_del_cyc(cycid);
    }
    return err;
}

/**
 * @brief 周期通知を待ち、モーター出力のランプ更新を実行するタスク。
 * @param[in] stacd タスク起動コード（この関数では使用しない）。
 * @param[in] exinf タスク拡張情報（この関数では使用しない）。
 * @details セマフォ通知を受け取るたびにmotor_ramp_step()を呼び出す。
 */
void task_motor(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    avoidance_state = AVOIDANCE_IDLE;
    avoidance_ticks = 0U;
    avoidance_request_pending = FALSE;
    avoidance_abort_pending = FALSE;
    while(1){
        if(tk_wai_sem(motor_tick_semid, 1, TMO_FEVR) == E_OK){
            UINT intsts;
            const char *message;
            DI(intsts);
            if(drive_expired != FALSE){
                task_motor_set_drive(FALSE, 0);
                motor_drive_stop();
                message = "drive: timed run decelerating\r\n";
            }else{
                message = task_motor_avoidance_step();
            }
            motor_ramp_step();
            EI(intsts);
            if(message != NULL) uart_tx_send(message);
        }
    }
}
