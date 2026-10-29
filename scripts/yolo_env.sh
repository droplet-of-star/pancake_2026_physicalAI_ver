# YOLO 用 venv を有効にする（GPU 版があればそれ）。使い方: source scripts/yolo_env.sh
if [ -n "$YOLO_VENV" ]; then VENV="$YOLO_VENV"; elif [ -x "$HOME/.venvs/yolo-gpu/bin/python" ]; then VENV="$HOME/.venvs/yolo-gpu"; else VENV="$HOME/.venvs/yolo"; fi
export PATH="$VENV/bin:$PATH"
CUDSS="$VENV/lib/python3.10/site-packages/nvidia/cu12/lib"; [ -d "$CUDSS" ] && export LD_LIBRARY_PATH="$CUDSS${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
