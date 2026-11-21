/**
 * @file heartbeat.c
 * @brief Heartbeat通信監視機能を実装する
 *
 * @details
 * UARTから受信したHeartbeatによって通信状態を管理する。
 * 一定時間Heartbeatを受信しない場合、通信が切断されたと判断する。
 */

#include <trykernel.h>
#include "heartbeat.h"
#include "uart_tx.h"

#define HEARTBEAT_PERIOD_MS   50U
#define HEARTBEAT_TIMEOUT_MS  500U

/* Heartbeat通信状態 */
static HEARTBEAT_STATE heartbeat_state = HEARTBEAT_DISCONNECTED;

/* 最終Heartbeat受信からの経過周期数 */
static UW heartbeat_elapsed_ticks = 0U;

/* Heartbeat関しタスクへの周期通知用セマフォID */
static ID heartbeat_semid = 0;

#define HEARTBEAT_TIMEOUT_TICKS \
    (HEARTBEAT_TIMEOUT_MS / HEARTBEAT_PERIOD_MS + 1U)
/**
 * @brief Heartbeat通信状態を取得する
 * @return heartbeat_state Heartbeat通信状態
 */
HEARTBEAT_STATE heartbeat_get_status(void)
{
    HEARTBEAT_STATE status;
    UINT intsts;
    DI(intsts);
    status = heartbeat_state;
    EI(intsts);
    return status;
}
/**
 * @brief Heartbeat通信が正常か判断する
 * @retval TRUE Heartbeat通信が正常
 * @retval FALSE 未接続またはタイムアウト
 */
BOOL heartbeat_is_connected(void){
    return (heartbeat_get_status() == HEARTBEAT_CONNECTED) ? TRUE : FALSE;
}

/**
 * @brief Heartbeat受信を通知する
 * @details
 * 正常なheartbeatを受信した際に、
 * 経過周波数をクリアし、通信状態をCONNECTEDへ変更する。
 */
void heartbeat_received(void)
{
    UINT intsts;
    DI(intsts);
    heartbeat_elapsed_ticks = 0U;
    heartbeat_state = HEARTBEAT_CONNECTED;
    EI(intsts);
}
/**
 * @brief Heartbeat監視タスクへ周期通知を送る。
 * @param[in] exinf 周期ハンドラの拡張情報（未使用）。
 *
 * @details
 * 50msごとにSysTick割り込みコンテキストから呼び出される。
 * セマフォを通知してHeartbeat監視タスクを起床させる。
 */
static void heartbeat_periodic_handler(void* exinf){
    (void)exinf;
    (void)tk_sig_sem(heartbeat_semid, 1);
}

/**
 * @brief Heartbeat監視機能を初期化する。
 *
 * @details
 * 通信状態を未接続に初期化し、監視タスクへの
 * 通知用セマフォと50ms周期ハンドラを生成・開始する。
 *
 * @retval E_OK 初期化成功。
 * @return 初期化に失敗した場合はTryKernelのエラーコード。
 */
ER heartbeat_init(void){
    static const T_CSEM csem = {
        .sematr = TA_TFIFO,
        .isemcnt = 0,
        .maxsem = 1
    };
    static const T_CCYC ccyc = {
        .exinf = NULL,
        .cycatr = TA_HLNG,
        .cychdr = heartbeat_periodic_handler,
        .cyctim = HEARTBEAT_PERIOD_MS, //　周期
        .cycphs = HEARTBEAT_PERIOD_MS // 初回実行までの遅延時間
    };
    ID cycid;
    ER err;
    UINT intsts;

    DI(intsts);
    heartbeat_state = HEARTBEAT_DISCONNECTED;
    heartbeat_elapsed_ticks = 0U;
    EI(intsts);

    heartbeat_semid = tk_cre_sem(&csem);
    if(heartbeat_semid < E_OK) {
        return (ER)heartbeat_semid;
    }

    cycid = tk_cre_cyc(&ccyc);
    if(cycid < E_OK) {
        return (ER)cycid;
    }

    err = tk_sta_cyc(cycid);
    if(err < E_OK) {
        (void)tk_del_cyc(cycid);
//        (void)tk_del_sem(heartbeat_semid);
    }

    return err;
}


/**
 * @brief Heartbeat受信状態を周期的に監視するタスク。
 *
 * @param[in] stacd タスク起動コード（未使用）。
 * @param[in] exinf タスク拡張情報（未使用）。
 *
 * @details
 * 50ms周期のセマフォ通知を受信するたびに、
 * Heartbeat未受信時間を監視する。
 *
 * 規定周期数に達すると通信状態をTIMEOUTへ変更し、
 * UARTへタイムアウトを通知する。
 *
 * @note モータ停止処理は今後の実装で追加する。
 */
void task_heartbeat(INT stacd, void *exinf)
{
    UINT intsts;
    BOOL timeout;

    (void)stacd;
    (void)exinf;

    while(1){
        if(tk_wai_sem(heartbeat_semid, 1, TMO_FEVR) == E_OK){

            timeout = FALSE;

            DI(intsts);

            if(heartbeat_state == HEARTBEAT_CONNECTED){

                heartbeat_elapsed_ticks++;
                if(heartbeat_elapsed_ticks >= HEARTBEAT_TIMEOUT_TICKS){
                    heartbeat_state = HEARTBEAT_TIMEOUT;
                    timeout = TRUE;
                }
            }

            EI(intsts);

            if(timeout == TRUE){
                uart_tx_send("heartbeat: timeout\r\n");
            }
        }
    }
}
