# 提出動画用の素材

`python3 scripts/make_video_assets.py` で再生成できる（Noto Sans CJK と Pillow を使用）。すべて 1920×1080。

## slides/ — 説明スライド（PNG、不透明）

| ファイル | 内容 | 想定の使いどころ |
|---|---|---|
| 01_title.png | タイトル（課題名、アカウント名の記入欄） | 冒頭 5 秒 |
| 02_overview.png | パンケーキ焼きロボットの 4 工程と写真 | 背景説明 |
| 03_task.png | 今回のタスクの流れと対象物・方針 | 実機映像の前 |
| 04_system.png | ノード構成図とハード | 実機映像の前 or 後 |
| 05_point1_plane.png | 工夫① 台平面方式（深度画像つき） | 観測シーンの補足 |
| 06_point2_detect.png | 工夫② 縁の円検出と YOLOv8n（実機画面つき） | パンケーキ確認シーンの補足 |
| 07_point3_control.png | 工夫③ 実測補正・把持判定・安全 | 把持シーンの補足 |
| 08_results.png | 結果の表 | 実機映像の後 |
| 09_ai.png | AI の活用（推論と開発プロセス） | 終盤 |
| 10_future.png | 反省と今後 | 終盤 |
| 11_end.png | 終了（コード URL、使用 OSS） | 最後 |

## telops/ — 場面テロップ（PNG、背景透過）

実機映像や画面録画の上に重ねる下部帯。t01〜t08 が pick_place の各段階、t09 は空振りからのやり直し用。
badge_2x / badge_4x / badge_realtime は右上の再生速度表示（倍速の場合は提出要件により必ず表示する）。

## 画面録画（docs/video/、git 管理外）

pickplace_20261026_*.mp4 は 1280×490、15 fps、各 2〜3 分。左が検出ノード、右がパンケーキ検出、下が controller のログ。
