#!/usr/bin/env bash
# eMudhra 中間 CA 鍵侵害 (2026-10) の対策 ― 最小構成版。内容を確認してから手動で実行する:
#   bash scripts/fix_emudhra_ca.sh
#
# やること (これ以外には触れない):
#   1. ~/Downloads/emudhra-ca-fix/ に置いた USN-8944-1 の .deb 2 つ (ca-certificates, libnss3) の SHA-256 を再確認
#      (.deb は ports.ubuntu.com の jammy-security から取得し、Ubuntu の署名付き InRelease → Packages → .deb の
#       ハッシュ連鎖で検証済み)
#   2. openssl の設定を完了させてから (ca-certificates の依存。postinst はリンク作成のみ)、その 2 つだけを dpkg -i で入れる。
#      apt-get は使わない。他の "unpacked" パッケージや docker-ce には触れない
#   3. Java の cacerts をシステムストアから再生成
#   4. curl / wget / python3 が侵害 CA のテストホストを拒否することを確認
# 参考: https://ubuntu.com/security/notices/USN-8944-1  https://emudhra.we-have-a-problem.com/
set -euo pipefail
D="$HOME/Downloads/emudhra-ca-fix"
CA=ca-certificates_20260816ubuntu0.22.04.1_all.deb
NSS=libnss3_3.98-0ubuntu0.22.04.5_arm64.deb
TEST=https://dv-g3b-emudhra.we-have-a-problem.com/

cd "$D"
echo "[1/4] .deb の SHA-256 を再確認"
sha256sum -c SHA256SUMS

echo "[2/4] ca-certificates と libnss3 だけを更新 (dpkg -i、apt は使わない)"
# openssl も Tanium の更新で "unpacked" のまま止まっており、ca-certificates の設定が依存で止まる。
# openssl の postinst は /usr/lib/ssl -> /etc/ssl のリンクを張るだけなので、先に設定だけ完了させる。
sudo dpkg --configure openssl || true
sudo dpkg -i "$CA" "$NSS"
dpkg-query -W -f='  ${Package} ${Version} (${Status})\n' ca-certificates libnss3

echo "[3/4] Java の cacerts から emSign ECC Root CA - G3 を削除"
# ca-certificates-java 20190909 は削除通知を取りこぼす (postinst 時の update-ca-certificates はフックを呼ばず、
# trigger 時には差分が 0 件) ため、Java の keystore に古いルートが残る。keytool で該当エイリアスだけ消す。
if keytool -list -cacerts -storepass changeit 2>/dev/null | grep -qi 'emsign_ecc_root_ca_-_g3'; then
  sudo keytool -delete -alias 'debian:emsign_ecc_root_ca_-_g3.pem' -keystore /etc/ssl/certs/java/cacerts -storepass changeit
fi
if keytool -list -cacerts -storepass changeit 2>/dev/null | grep -qi 'emsign_ecc_root_ca_-_g3'; then
  echo "  NG: Java cacerts にまだ emSign ECC Root CA - G3 が残っている"
else
  echo "  OK: Java cacerts から emSign ECC Root CA - G3 を削除"
fi

echo "[4/4] 検証"
if ls /etc/ssl/certs | grep -qi 'emsign_ecc_root_ca_-_g3'; then echo "  NG: /etc/ssl/certs に emSign ECC Root CA - G3 が残っている"
else echo "  OK: /etc/ssl/certs から emSign ECC Root CA - G3 を除去"; fi
if curl -sS -m 15 -o /dev/null "$TEST" 2>/dev/null; then echo "  NG: curl はまだ接続できる"; else echo "  OK: curl は拒否した"; fi
if wget -q -O /dev/null --timeout=15 "$TEST" 2>/dev/null; then echo "  NG: wget はまだ接続できる"; else echo "  OK: wget は拒否した"; fi
if python3 -c "import urllib.request; urllib.request.urlopen('$TEST', timeout=15)" 2>/dev/null; then echo "  NG: python3 はまだ接続できる"; else echo "  OK: python3 は拒否した"; fi
echo "完了。Node.js (~/.local/node → 24.22.1) と Chromium は対応済み。"
echo "livekit-app のコンテナを戻すには: cd ~/vscode/livekit-app && docker compose up -d"
