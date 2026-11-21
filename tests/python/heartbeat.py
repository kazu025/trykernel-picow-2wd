
import time
import serial

PORT = "/dev/ttyACM0"  # 実際のポート名に変更
BAUDRATE = 115200

HEARTBEAT_INTERVAL = 0.1   # 100ms
PHASE_DURATION = 5.0       # 5秒ごとに送信/停止

with serial.Serial(PORT, BAUDRATE, timeout=0) as ser:
    print("Heartbeat test start (Ctrl+C to exit)")

    sending = True
    now = time.monotonic()
    next_send = now
    next_switch = now + PHASE_DURATION

    try:
        while True:
            now = time.monotonic()

            # 送信・停止状態を5秒ごとに切り替える
            if now >= next_switch:
                sending = not sending
                next_switch = now + PHASE_DURATION

                if sending:
                    print("\n[PC] Heartbeat RESUME")
                    next_send = now
                else:
                    print("\n[PC] Heartbeat PAUSE")

            # 送信中は100ms周期でhbコマンドを送る
            if sending and now >= next_send:
                ser.write(b"hb\r\n")
                next_send = now + HEARTBEAT_INTERVAL

            # Picoから受信したログを表示
            available = ser.in_waiting

            if available > 0:
                data = ser.read(available)
                print(
                    data.decode("utf-8", errors="replace"),
                    end="",
                    flush=True
                )

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[PC] Heartbeat test finished")
