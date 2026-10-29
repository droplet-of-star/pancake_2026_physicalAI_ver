# udev ルール

`99-so101-lerobot.rules` を `/etc/udev/rules.d/` に置くと、USB の差し順に依存しない
`/dev/usbserial_lerobot_follower` / `/dev/usbserial_lerobot_leader` が作られる。
適用は `scripts/install_sudo_deps.sh`（sudo 必要）。

シリアル番号の確認: `udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`

## OAK-D Pro

`80-movidius.rules` は OAK（USB ベンダ 03e7）を一般ユーザーで開くためのルール。未適用だと depthai が
`X_LINK_INSUFFICIENT_PERMISSIONS` で失敗する。適用後は OAK を一度抜き差しする。
