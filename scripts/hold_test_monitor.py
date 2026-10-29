#!/usr/bin/env python3
"""トルクON保持中の電源・通信を監視するテスト（motor bridge は止めておく）。

ブリッジと同じ「現在姿勢を保持」を一定時間行い、各サイクルで電圧・電流・通信失敗を記録する。
電源のへたり（電圧低下）や接触不良（特定モータだけ無応答）を切り分ける。
終了時は必ずトルクを切る。Ctrl+C でも切る。

  python3 scripts/hold_test_monitor.py [--port /dev/ttyACM0] [--seconds 8] [--joints wrist_roll,gripper]
アームは手で支えられる状態で実行すること。
"""
import argparse, time
from so101_hw_interface.bus_lock import acquire_bus_lock
from so101_hw_interface.motors import Motor, MotorNormMode
from so101_hw_interface.motors.feetech.feetech import FeetechMotorsBus

J = {'shoulder_pan': 1, 'shoulder_lift': 2, 'elbow_flex': 3, 'wrist_flex': 4, 'wrist_roll': 5, 'gripper': 6}
ap = argparse.ArgumentParser(); ap.add_argument('--port', default='/dev/ttyACM0'); ap.add_argument('--seconds', type=float, default=8.0)
ap.add_argument('--joints', default='all', help='トルクを入れる関節（カンマ区切り、all で全部）')
a = ap.parse_args()
targets = list(J) if a.joints == 'all' else a.joints.split(',')
lock = acquire_bus_lock(a.port)
bus = FeetechMotorsBus(a.port, {n: Motor(i, 'sts3215', MotorNormMode.DEGREES) for n, i in J.items()})
bus.connect(handshake=False)
try:
    pos0 = bus.sync_read('Present_Position', normalize=False)
    v0 = bus.sync_read('Present_Voltage', normalize=False)
    print('torque OFF 時の電圧[V]:', {n: v / 10 for n, v in v0.items()})
    print(f'トルクON対象: {targets}  保持目標 = 現在位置 {[pos0[n] for n in targets]}')
    input('アームを支えて Enter（トルクが入ります） > ')
    for n in targets:
        bus.write('Torque_Enable', n, 1, num_retry=3)
    t0 = time.time(); fails = 0; cycles = 0; vmin = 99.0; imax = {n: 0 for n in J}; last_print = 0
    vmin_info = None; dips = 0
    while time.time() - t0 < a.seconds:
        cycles += 1
        try:
            bus.sync_write('Goal_Position', {n: pos0[n] for n in targets}, normalize=False)
            v = bus.sync_read('Present_Voltage', normalize=False)
            c = bus.sync_read('Present_Current', normalize=False)
            vnow = min(v.values()) / 10
            if vnow < vmin:
                vmin = vnow; vmin_info = (time.time() - t0, dict(v), dict(c))
            if vnow < 11.0:
                dips += 1
                print(f'  !! {time.time()-t0:4.1f}s 電圧低下 V={vnow:4.1f}  I(raw)=' + ' '.join(f'{n[:5]}:{c[n]:3d}' for n in J))
            for n in J: imax[n] = max(imax[n], c[n])
            if time.time() - last_print > 1.0:
                last_print = time.time()
                print(f'  {time.time()-t0:4.1f}s  V={min(v.values())/10:4.1f}  I(raw)=' + ' '.join(f'{n[:5]}:{c[n]:3d}' for n in J) + f'  fails={fails}')
        except Exception as e:  # noqa: BLE001
            fails += 1
            print(f'  {time.time()-t0:4.1f}s  通信失敗 #{fails}: {type(e).__name__} {str(e)[:70]}')
            time.sleep(0.05)
        time.sleep(0.02)
    print(f'\n結果: {cycles} サイクル中 失敗 {fails}。最低電圧 {vmin:.1f} V（11 V 未満のサンプル {dips} 回）。最大電流(raw, 1=6.5 mA): {imax}')
    if vmin_info:
        tt, vv, cc = vmin_info
        print(f'  最低電圧の瞬間: {tt:.1f}s  各モータ電圧 ' + ' '.join(f'{n[:5]}:{vv[n]/10:.1f}' for n in J) + '  電流 ' + ' '.join(f'{n[:5]}:{cc[n]}' for n in J))
        total_a = sum(cc.values()) * 0.0065
        print(f'  その時の合計電流（概算）: {total_a:.2f} A')
    print('  判定目安: 失敗が続く+電圧低下 → 電源容量/接触不良。特定モータだけ無応答 → そのモータ手前の配線。')
finally:
    try:
        bus.disable_torque(num_retry=5); print('トルクを切りました。アームを支えてください。')
    except Exception as e:  # noqa: BLE001
        print('トルクOFF の送信に失敗:', e, '（モータが応答していません。電源を確認）')
    bus.disconnect(disable_torque=False)
