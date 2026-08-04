# WSL2ミラーモードでROS 2 CLIが停止する問題と対処方法

## 1. 概要

WSL2をミラーモードで使用している環境で、WSL起動後に次のコマンドを実行すると、処理が停止してトピック一覧が表示されないことがある。

```bash
ros2 topic list
```

実際の現象は次のとおり。

```text
$ ros2 topic list
^C

$ pgrep -af '_ros2_daemon|ros2-daemon'
# 何も表示されない

$ ros2 daemon start
The daemon has been started

$ ros2 topic list
/parameter_events
/rosout
```

`ros2 daemon start`を明示的に実行した後は、`ros2 topic list`が正常に動作する。

---

## 2. 使用環境

今回の構成では、別PCからGStreamerのUDP映像をWSLへ送信するため、WSL2のミラーモードを使用している。

```text
別PC
  │
  │ GStreamer UDP
  ▼
Windows PC
  │
  │ WSL2ミラーモード
  ▼
WSL上のGStreamer / gscam2
  │
  │ ROS 2 Topic
  ▼
画像処理ノード・RViz
```

WSLの設定例は次のとおり。

```ini
# C:\Users\<ユーザー名>\.wslconfig

[wsl2]
networkingMode=mirrored
dnsTunneling=true
firewall=true
```

設定変更後は、Windows側で次を実行する。

```powershell
wsl --shutdown
```

---

## 3. 発生する問題

通常、`ros2 topic list`や`ros2 node list`などのROS 2 CLIを実行すると、ROS 2 daemonが存在しない場合は自動的に起動する。

しかし、今回のWSL2ミラーモード環境では、daemonが存在しない状態で次を実行すると処理が停止する。

```bash
ros2 topic list
```

同様に、daemonが存在しない状態で次を実行した場合も、処理が停止することがある。

```bash
ros2 daemon stop
```

一方、次を明示的に実行すると正常に起動する。

```bash
ros2 daemon start
```

---

## 4. `wsl --shutdown`後のdaemon状態

Windows側で次を実行すると、WSL2の仮想マシンと、その中で動作しているすべてのプロセスが終了する。

```powershell
wsl --shutdown
```

そのため、次回WSLを起動した直後にROS 2 daemonのプロセスが存在しないのは正常である。

確認コマンド：

```bash
pgrep -af '_ros2_daemon|ros2cli.daemon'
```

何も表示されない場合、daemonは起動していない。

```text
$ pgrep -af '_ros2_daemon|ros2cli.daemon'
$
```

`wsl --shutdown`後は、終了対象となるdaemonが存在しないため、次を実行する必要はない。

```bash
ros2 daemon stop
pkill -f _ros2_daemon
```

必要なのは、WSL起動後にdaemonを明示的に起動することである。

```bash
ros2 daemon start
```

---

## 5. 手動での対処方法

WSLを起動した直後に、次を実行する。

```bash
ros2 daemon start
```

daemonが起動したことを確認する。

```bash
pgrep -af '_ros2_daemon|ros2cli.daemon'
```

その後、ROS 2 CLIを実行する。

```bash
ros2 topic list
```

確認例：

```text
$ ros2 daemon start
The daemon has been started

$ ros2 topic list
/parameter_events
/rosout
```

---

## 6. 自動化の要件

毎回手動で`ros2 daemon start`を実行するのは手間がかかる。

ただし、複数のターミナルからWSLへログインするたびに、毎回`ros2 daemon start`を実行することも避けたい。

必要な動作は次のとおり。

| 状態                         | 動作            |
| -------------------------- | ------------- |
| NATモード                     | 何もしない         |
| WSLミラーモードの最初のターミナル         | daemonを起動する   |
| 同じWSL起動中の2個目以降のターミナル       | 何もしない         |
| `wsl --shutdown`後の最初のターミナル | 再びdaemonを起動する |
| daemonがすでに起動している           | 何もしない         |

---

## 7. WSLの起動単位を判定する方法

Linuxでは、現在のブートを識別するIDを次のファイルから取得できる。

```bash
cat /proc/sys/kernel/random/boot_id
```

例：

```text
b7d14aec-e6bc-41c8-8330-39017943c65e
```

このIDには次の特徴がある。

* 同じWSL起動中は変化しない
* `wsl --shutdown`後にWSLを起動すると変化する
* 複数のターミナルで同じ値になる

この`boot_id`を保存しておくことで、同じWSL起動中に処理済みか判定できる。

---

## 8. ミラーモードの判定

現在のWSLネットワークモードは次のコマンドで確認する。

```bash
wslinfo --networking-mode
```

ミラーモードの場合：

```text
mirrored
```

NATモードの場合：

```text
nat
```

今回の自動処理は、結果が`mirrored`の場合だけ実行する。

---

## 9. `.bashrc`へ追加する自動起動処理

次の内容を`~/.bashrc`へ追加する。

ROS 2の環境設定や`ROS_DOMAIN_ID`などを設定した後に配置する。

```bash
# =========================================================
# WSL2ミラーモード用 ROS 2 daemon起動処理
#
# ・ミラーモードの場合だけ実行
# ・WSLの1ブートにつき1回だけ実行
# ・複数ターミナルの同時起動にも対応
# ・daemonがすでに存在する場合は何もしない
# =========================================================
ros2_daemon_start_once_per_wsl_boot()
{
    # ROS 2が利用できない場合は何もしない
    command -v ros2 >/dev/null 2>&1 || return 0

    # WSLネットワークモードを確認できない場合は何もしない
    command -v wslinfo >/dev/null 2>&1 || return 0

    # flockが利用できない場合は何もしない
    command -v flock >/dev/null 2>&1 || {
        echo "[ROS 2] flock command is not available." >&2
        return 1
    }

    local networking_mode
    local current_boot_id
    local domain_id
    local rmw
    local state_dir
    local state_file
    local lock_file

    networking_mode="$(
        wslinfo --networking-mode 2>/dev/null |
        tr '[:upper:]' '[:lower:]' |
        xargs
    )"

    # ミラーモード以外では何もしない
    if [ "$networking_mode" != "mirrored" ]; then
        return 0
    fi

    current_boot_id="$(cat /proc/sys/kernel/random/boot_id)" || return 1

    domain_id="${ROS_DOMAIN_ID:-0}"
    rmw="${RMW_IMPLEMENTATION:-default}"

    state_dir="$HOME/.cache/ros2"
    state_file="$state_dir/daemon-${domain_id}-${rmw}.boot-id"
    lock_file="$state_dir/daemon-${domain_id}-${rmw}.lock"

    mkdir -p "$state_dir"

    (
        # 複数ターミナルから同時に実行された場合の重複防止
        flock -x 9

        # 同じWSLブートで処理済み、かつdaemonが存在する場合は終了
        if [ -f "$state_file" ] &&
           [ "$(cat "$state_file")" = "$current_boot_id" ] &&
           pgrep -f '_ros2_daemon|ros2cli\.daemon' >/dev/null 2>&1; then
            exit 0
        fi

        # daemonがすでに存在する場合は、処理済みとして記録する
        if pgrep -f '_ros2_daemon|ros2cli\.daemon' >/dev/null 2>&1; then
            printf '%s\n' "$current_boot_id" > "$state_file"
            exit 0
        fi

        echo "[ROS 2] WSL mirrored mode detected."
        echo "[ROS 2] Starting daemon for this WSL boot..."

        # WSL起動直後のネットワーク初期化を少し待つ
        sleep 1

        if timeout -k 2s 15s ros2 daemon start >/dev/null 2>&1; then
            printf '%s\n' "$current_boot_id" > "$state_file"
            echo "[ROS 2] Daemon started."
        else
            echo "[ROS 2] Failed to start daemon." >&2
            exit 1
        fi

    ) 9>"$lock_file"
}

ros2_daemon_start_once_per_wsl_boot
```

---

## 10. `.bashrc`の配置例

```bash
# ROS 2 Humble
source /opt/ros/humble/setup.bash

# ROS 2ワークスペース
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

# ROS 2通信設定
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp


# =========================================================
# WSL2ミラーモード用 ROS 2 daemon起動処理
# =========================================================
ros2_daemon_start_once_per_wsl_boot()
{
    command -v ros2 >/dev/null 2>&1 || return 0
    command -v wslinfo >/dev/null 2>&1 || return 0

    command -v flock >/dev/null 2>&1 || {
        echo "[ROS 2] flock command is not available." >&2
        return 1
    }

    local networking_mode
    local current_boot_id
    local domain_id
    local rmw
    local state_dir
    local state_file
    local lock_file

    networking_mode="$(
        wslinfo --networking-mode 2>/dev/null |
        tr '[:upper:]' '[:lower:]' |
        xargs
    )"

    if [ "$networking_mode" != "mirrored" ]; then
        return 0
    fi

    current_boot_id="$(cat /proc/sys/kernel/random/boot_id)" || return 1

    domain_id="${ROS_DOMAIN_ID:-0}"
    rmw="${RMW_IMPLEMENTATION:-default}"

    state_dir="$HOME/.cache/ros2"
    state_file="$state_dir/daemon-${domain_id}-${rmw}.boot-id"
    lock_file="$state_dir/daemon-${domain_id}-${rmw}.lock"

    mkdir -p "$state_dir"

    (
        flock -x 9

        if [ -f "$state_file" ] &&
           [ "$(cat "$state_file")" = "$current_boot_id" ] &&
           pgrep -f '_ros2_daemon|ros2cli\.daemon' >/dev/null 2>&1; then
            exit 0
        fi

        if pgrep -f '_ros2_daemon|ros2cli\.daemon' >/dev/null 2>&1; then
            printf '%s\n' "$current_boot_id" > "$state_file"
            exit 0
        fi

        echo "[ROS 2] WSL mirrored mode detected."
        echo "[ROS 2] Starting daemon for this WSL boot..."

        sleep 1

        if timeout -k 2s 15s ros2 daemon start >/dev/null 2>&1; then
            printf '%s\n' "$current_boot_id" > "$state_file"
            echo "[ROS 2] Daemon started."
        else
            echo "[ROS 2] Failed to start daemon." >&2
            exit 1
        fi

    ) 9>"$lock_file"
}

ros2_daemon_start_once_per_wsl_boot
```

---

## 11. 設定の反映

`.bashrc`を編集した後、次を実行する。

```bash
source ~/.bashrc
```

現在のWSLブートでdaemonが存在しなければ、次のように表示される。

```text
[ROS 2] WSL mirrored mode detected.
[ROS 2] Starting daemon for this WSL boot...
[ROS 2] Daemon started.
```

すでにdaemonが起動している場合は何も表示されない。

---

## 12. 動作確認

### 12.1 ネットワークモードを確認する

```bash
wslinfo --networking-mode
```

想定結果：

```text
mirrored
```

### 12.2 daemonプロセスを確認する

```bash
pgrep -af '_ros2_daemon|ros2cli.daemon'
```

daemonが起動していれば、Pythonプロセスなどが表示される。

### 12.3 ROS 2 CLIを確認する

```bash
timeout 10s ros2 topic list
```

想定結果：

```text
/parameter_events
/rosout
```

---

## 13. 複数ターミナルでの確認

最初のターミナルを開く。

```text
[ROS 2] WSL mirrored mode detected.
[ROS 2] Starting daemon for this WSL boot...
[ROS 2] Daemon started.
```

同じWSLディストリビューションで2個目のターミナルを開く。

```text
erica@krm-think-ce:~$
```

2個目以降では、daemon起動処理は実行されない。

プロセス数を確認する。

```bash
pgrep -fc '_ros2_daemon|ros2cli\.daemon'
```

通常はdaemonが1つだけ表示される。

---

## 14. `wsl --shutdown`後の動作確認

Windows側で次を実行する。

```powershell
wsl --shutdown
```

その後、WSLを起動する。

```batch
wsl ~ --distribution WorldTracker-22.04 --user erica
```

新しいWSLブートになるため、最初のターミナルで再びdaemonが起動する。

```text
[ROS 2] WSL mirrored mode detected.
[ROS 2] Starting daemon for this WSL boot...
[ROS 2] Daemon started.
```

---

## 15. NATモードの場合

WSLがNATモードの場合、次の結果になる。

```bash
wslinfo --networking-mode
```

```text
nat
```

この場合、自動起動関数は何もせず終了する。

通常のROS 2 CLIによるdaemon自動起動に任せる。

---

## 16. 手動でdaemonを確認・起動する方法

### daemon確認

```bash
pgrep -af '_ros2_daemon|ros2cli.daemon'
```

### daemon起動

```bash
ros2 daemon start
```

### トピック確認

```bash
ros2 topic list
```

---

## 17. daemonが異常終了した場合

同じWSLブート中にdaemonが異常終了した場合は、次のターミナル起動時に関数がdaemonの存在を確認し、存在しなければ再起動する。

手動で再起動する場合は、次を実行する。

```bash
pkill -TERM -f '_ros2_daemon|ros2cli\.daemon' 2>/dev/null || true
sleep 1
ros2 daemon start
```

daemonが存在しない状態では、WSLミラーモードの問題によって`ros2 daemon stop`が停止する可能性があるため、プロセス確認には`pgrep`を使用する。

```bash
pgrep -af '_ros2_daemon|ros2cli.daemon'
```

---

## 18. 注意事項

### ROS 2環境設定より後に配置する

自動起動関数は、次の設定より後に配置する。

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

daemonを起動した後に`ROS_DOMAIN_ID`や`RMW_IMPLEMENTATION`を変更すると、daemonと現在のシェルで設定が一致しなくなる可能性がある。

### ROS_DOMAIN_IDごとに管理される

状態ファイルは、`ROS_DOMAIN_ID`と`RMW_IMPLEMENTATION`ごとに分けて保存される。

例：

```text
~/.cache/ros2/daemon-0-rmw_fastrtps_cpp.boot-id
```

複数のDomain IDを使用する場合は、それぞれの環境でdaemonが起動する。

### 複数のWSLディストリビューション

`boot_id`はWSL2 VMのブートを示す。

状態ファイルは各ディストリビューションのホームディレクトリ内に保存されるため、複数ディストリビューションを使用しても、それぞれで管理される。

---

## 19. 状態ファイルの削除

自動起動状態を手動でリセットしたい場合は、次を実行する。

```bash
rm -f ~/.cache/ros2/daemon-*.boot-id
```

次のターミナル起動時に、daemonの存在を再確認する。

ただし、daemonがすでに起動している場合は再起動せず、現在のブートIDだけを記録する。

---

## 20. 最終的な運用方法

通常の操作は次のとおり。

```text
Windows起動
  ↓
WSLターミナル起動
  ↓
ミラーモードか確認
  ↓
最初のターミナルだけros2 daemon start
  ↓
ros2 topic listなどを使用
```

複数ターミナルを開いた場合：

```text
1個目のターミナル
  → daemonを起動

2個目以降
  → daemonが存在するため何もしない
```

`wsl --shutdown`後：

```text
wsl --shutdown
  ↓
daemonを含むWSLプロセスがすべて終了
  ↓
次回WSL起動
  ↓
最初のターミナルだけdaemonを再起動
```

---

## 21. 結論

今回のWSL2ミラーモード環境では、ROS 2 daemonが存在しない状態で`ros2 topic list`を実行しても、daemonが正常に自動起動せず、CLIが停止する。

明示的に次を実行すると正常に動作する。

```bash
ros2 daemon start
```

対処として、次の条件を満たす自動起動処理を`.bashrc`へ追加する。

* WSL2ミラーモードの場合だけ実行する
* 同じWSL起動中は1回だけ実行する
* 複数ターミナルからの同時実行を防止する
* daemonがすでに存在する場合は何もしない
* `wsl --shutdown`後は再び1回だけ実行する

これにより、GStreamerのUDP受信に必要なWSLミラーモードを維持したまま、ROS 2 CLIを安定して使用できる。
