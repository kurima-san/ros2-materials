# ROS 2で機械学習環境を分離する方法

## 結論

ROS 2をUbuntuの`apt`でインストールしている場合は、用途に応じて次のように使い分けるのがおすすめです。

| 用途                                  | 推奨環境                           |
| ----------------------------------- | ------------------------------ |
| NumPy、SciPy、scikit-learn、DBSCAN、PCA | `venv --system-site-packages`  |
| 軽量な機械学習・フィルタ処理                      | `venv --system-site-packages`  |
| PyTorch CPU版                        | `venv`でも可能                     |
| PyTorch＋CUDA                        | Docker推奨                       |
| YOLO、TensorRT、CUDA、cuDNN            | Docker推奨                       |
| チームで完全に同じ環境を再現                      | Docker／Dev Container           |
| apt版ROS 2とCondaの混在                  | 非推奨                            |
| ROS 2全体をCondaで管理                    | RoboStack。ただしapt版ROS 2とは別環境にする |

今回のように、骨格情報に対して以下の処理を行う程度であれば、まずはPython標準の`venv`で十分です。

* DBSCANによるグループ分け
* PCAによる楕円推定
* NumPy、SciPyを使った数値処理
* FilterPyなどを使った追跡
* scikit-learnを使った軽量な機械学習

推奨構成は次のとおりです。

```bash
/usr/bin/python3 -m venv --system-site-packages ~/ros2_ws/.venv
touch ~/ros2_ws/.venv/COLCON_IGNORE

source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate

python -m pip install scipy scikit-learn filterpy
```

将来的にPyTorch、YOLO、CUDA、TensorRTなどを使う場合は、機械学習推論ノードだけをDockerコンテナへ分離する構成が扱いやすいです。

---

# 1. ROS 2で仮想環境が必要な理由

ROS 2環境に対して直接`pip install`を繰り返すと、次のような問題が起きやすくなります。

* ROS 2が使用するPythonパッケージと競合する
* NumPyのバージョンが変わる
* apt版OpenCVとpip版OpenCVが混在する
* `cv_bridge`が動かなくなる
* プロジェクトごとに必要なライブラリが異なる
* どのパッケージがROS 2本体に必要なのか分からなくなる
* 別のPCで同じ環境を再現しにくい

そのため、機械学習ライブラリはROS 2本体のPython環境へ直接入れず、プロジェクトごとの仮想環境に分離します。

---

# 2. 最も簡単な方法：Python標準の`venv`

## 2.1 仮想環境の作成

ROS 2 HumbleとUbuntu 22.04を想定します。

```bash
sudo apt update
sudo apt install -y python3-venv

cd ~/ros2_ws

/usr/bin/python3 -m venv --system-site-packages .venv
```

重要なのは、次のオプションです。

```text
--system-site-packages
```

これを付けることで、仮想環境内からaptでインストールされたROS 2のPythonパッケージを参照できます。

例えば、以下のパッケージです。

* `rclpy`
* `launch`
* `rosidl_runtime_py`
* `cv_bridge`
* `geometry_msgs`
* `sensor_msgs`
* 独自に生成したROS 2メッセージ

通常の`venv`では、これらのシステム側パッケージが見えません。

---

## 2.2 `COLCON_IGNORE`を置く

仮想環境をROS 2ワークスペース内に作る場合は、`.venv`内を`colcon`が探索しないようにします。

```bash
touch ~/ros2_ws/.venv/COLCON_IGNORE
```

ワークスペースは次のような構成になります。

```text
ros2_ws/
├── .venv/
│   └── COLCON_IGNORE
├── requirements.txt
├── src/
├── build/
├── install/
└── log/
```

---

## 2.3 仮想環境を有効化する

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate
```

有効化後は、プロンプトの先頭に通常は次のような表示が付きます。

```text
(.venv) user@ubuntu:~/ros2_ws$
```

Pythonの参照先を確認します。

```bash
which python
python --version
```

結果例：

```text
/home/user/ros2_ws/.venv/bin/python
Python 3.10.x
```

---

## 2.4 ROS 2のPythonモジュールが見えるか確認する

```bash
python -c "import rclpy; print(rclpy.__file__)"
```

結果例：

```text
/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py
```

次のコマンドでも確認できます。

```bash
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
python -c "import cv2; print(cv2.__version__, cv2.__file__)"
python -c "import rclpy; print(rclpy.__file__)"
```

---

# 3. 機械学習ライブラリのインストール

仮想環境を有効化した状態でインストールします。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate

python -m pip install --upgrade pip setuptools wheel
```

DBSCAN、PCA、追跡処理に必要なライブラリの例です。

```bash
python -m pip install \
    numpy \
    scipy \
    scikit-learn \
    filterpy
```

ただし、NumPyはROS 2や`cv_bridge`との互換性に影響するため、むやみに最新版へ更新しないほうが安全です。

例えば、バージョンを固定します。

```bash
python -m pip install \
    numpy==1.26.4 \
    scipy==1.13.1 \
    scikit-learn==1.5.2 \
    filterpy==1.4.5
```

---

# 4. ROS 2ワークスペースのビルド

仮想環境を有効化してからビルドします。

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash
source .venv/bin/activate

colcon build --symlink-install
```

ビルド後にワークスペースを読み込みます。

```bash
source install/setup.bash
```

実行時は次の順序に統一します。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/.venv/bin/activate
source ~/ros2_ws/install/setup.bash
```

その後、通常どおり実行します。

```bash
ros2 run my_package my_node
```

または、

```bash
ros2 launch my_package my_launch.py
```

---

# 5. `requirements.txt`で依存関係を管理する

必要なPythonパッケージは`requirements.txt`へ記録します。

例：

```text
numpy==1.26.4
scipy==1.13.1
scikit-learn==1.5.2
filterpy==1.4.5
```

インストールは次のように行います。

```bash
source ~/ros2_ws/.venv/bin/activate
python -m pip install -r requirements.txt
```

現在の仮想環境から一覧を出力する場合は、次のコマンドを使えます。

```bash
python -m pip freeze > requirements.txt
```

ただし、`pip freeze`では間接的にインストールされたパッケージもすべて記録されます。

長期的に管理する場合は、実際に使用している主要パッケージだけを手動で記述するほうが分かりやすいです。

---

# 6. `--system-site-packages`の注意点

`--system-site-packages`を使うと、仮想環境からシステム側のPythonパッケージも見えます。

例えば以下です。

```text
aptでインストールしたNumPy
aptでインストールしたOpenCV
aptでインストールしたcv_bridge
aptでインストールしたrclpy
```

そのため、完全に独立したPython環境ではありません。

仮想環境側で別バージョンをインストールすると、仮想環境側のパッケージが優先されることがあります。

特に、次のコマンドには注意が必要です。

```bash
python -m pip install --upgrade numpy
python -m pip install --upgrade opencv-python
```

これにより、次のような問題が発生することがあります。

* `cv_bridge`とNumPyのABIが合わなくなる
* apt版OpenCVとpip版OpenCVが混在する
* `import cv2`の参照先が変わる
* NumPy 1系向けのバイナリにNumPy 2系が読み込まれる
* ROS 2ノード起動時に`ImportError`が発生する

問題が発生した場合は、参照先を確認します。

```bash
python -c "import numpy; print(numpy.__version__, numpy.__file__)"
python -c "import cv2; print(cv2.__version__, cv2.__file__)"
python -c "import rclpy; print(rclpy.__file__)"
```

---

# 7. OpenCVと`cv_bridge`の扱い

ROS 2で画像処理を行う場合は、可能な限りapt版OpenCVを使うほうが安全です。

```bash
sudo apt install \
    python3-opencv \
    ros-humble-cv-bridge
```

特別な理由がない限り、次のパッケージを仮想環境へ追加するのは避けます。

```bash
python -m pip install opencv-python
```

pip版OpenCVが必要になる例は、次のような場合です。

* apt版にない新しいOpenCV機能が必要
* 特定バージョンに固定したい
* ROS 2ノードと画像処理プロセスを分離している
* `cv_bridge`を使用しない
* Docker内でOpenCVも含めて環境を統一している

---

# 8. PyTorchやCUDAを使う場合

以下を使う場合は、`venv`よりDockerのほうが管理しやすくなります。

* PyTorch
* TensorFlow
* CUDA
* cuDNN
* TensorRT
* Ultralytics YOLO
* OpenMMLab
* CUDA対応OpenCV
* ZED SDK
* GPUを使った人物姿勢推定
* GPUを使った画像認識

`venv`が分離できるのは、主にPythonパッケージです。

一方、CUDA、cuDNN、TensorRTなどはOS側の共有ライブラリやGPUドライバにも依存します。

そのため、Python環境だけ分離しても依存関係が複雑になりやすいです。

---

# 9. Dockerを使った推奨構成

機械学習ノードだけをDockerコンテナへ入れ、センサードライバやNavigation2はホスト側で動かす構成が扱いやすいです。

```text
ホストOS
├── カメラドライバ
├── LiDARドライバ
├── ZED／Azure Kinectドライバ
├── Navigation2
├── ロボット制御ノード
└── DDS通信
       │
       ▼
Dockerコンテナ
├── ROS 2
├── PyTorch
├── CUDA
├── TensorRT
├── 学習済みモデル
└── 推論ROS 2ノード
```

ROS 2ノード同士はDDSで通信するため、ホストとコンテナに分かれていてもトピック通信できます。

基本的な起動例です。

```bash
docker run --rm -it \
    --network host \
    --ipc host \
    --gpus all \
    -v ~/ros2_ws:/ros2_ws \
    ros:humble-ros-base
```

カメラをコンテナから直接使う場合は、デバイスを渡します。

```bash
docker run --rm -it \
    --network host \
    --ipc host \
    --device=/dev/video0 \
    -v ~/ros2_ws:/ros2_ws \
    ros:humble-ros-base
```

NVIDIA GPUを使う場合は、ホスト側にNVIDIA Container Toolkitが必要です。

```bash
--gpus all
```

開発中に多数のUSBデバイスへアクセスする場合は、次の方法もあります。

```bash
--privileged
```

ただし、`--privileged`はコンテナへ強い権限を与えるため、運用時は必要なデバイスだけを個別に指定するほうが安全です。

---

# 10. Condaをapt版ROS 2と混ぜない

aptでインストールしたROS 2とCondaを同時に使う構成は、基本的におすすめしません。

例えば次のような構成です。

```bash
source /opt/ros/humble/setup.bash
conda activate ml
```

Condaを有効化すると、Ubuntu標準Pythonではなく、Conda側のPythonが優先されることがあります。

ROS 2 Humbleのaptパッケージは、Ubuntu 22.04の標準Python 3.10向けにビルドされています。

Conda側のPythonバージョンや共有ライブラリが異なると、次のようなエラーが発生します。

```text
ModuleNotFoundError: No module named 'rclpy'
```

```text
ImportError: librclpy_common.so
```

```text
ImportError: undefined symbol
```

```text
The C extension was built for a different Python version
```

Condaを使う場合は、次のどちらかに統一します。

1. apt版ROS 2を使わず、ROS 2も含めてRoboStack／Conda側へ入れる
2. ROS 2ノードとCondaの機械学習プロセスを別プロセスに分ける

ただし、一般的なROS 2の手順やパッケージとの互換性を考えると、実機ロボットでは`venv`またはDockerのほうが扱いやすいです。

---

# 11. 仮想環境を簡単に読み込むスクリプト

毎回複数の`source`を実行するのが面倒な場合は、スクリプトを作成します。

`~/ros2_ws/activate_ros2.sh`

```bash
#!/bin/bash

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/.venv/bin/activate"

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    source "$HOME/ros2_ws/install/setup.bash"
fi

echo "ROS_DISTRO=$ROS_DISTRO"
echo "Python=$(which python)"
echo "ROS 2 workspace activated"
```

実行権限を付けます。

```bash
chmod +x ~/ros2_ws/activate_ros2.sh
```

読み込む場合は、スクリプトを直接実行するのではなく、`source`を使います。

```bash
source ~/ros2_ws/activate_ros2.sh
```

短いエイリアスを設定する場合は、`~/.bashrc`に追加します。

```bash
alias activate_ros2='source ~/ros2_ws/activate_ros2.sh'
```

反映します。

```bash
source ~/.bashrc
```

以降は次のコマンドだけで環境を有効化できます。

```bash
activate_ros2
```

---

# 12. プロジェクトごとに仮想環境を分ける

複数のROS 2プロジェクトで必要な機械学習ライブラリが異なる場合は、ワークスペースごとに`.venv`を作成します。

```text
~/human_tracker_ws/
├── .venv/
└── src/

~/navigation_ws/
├── .venv/
└── src/

~/gaze_control_ws/
├── .venv/
└── src/
```

例えば人物追跡用環境には、次のようなパッケージを入れます。

```text
numpy
scipy
scikit-learn
filterpy
```

視線制御学習用環境には、次のようなパッケージを入れます。

```text
numpy
torch
pandas
tensorboard
```

このように分けることで、異なるプロジェクト間でパッケージのバージョンが衝突しにくくなります。

---

# 13. 今回のシステムに対する推奨構成

骨格情報から人物グループを推定するシステムでは、次のような処理を想定します。

* Azure KinectまたはZEDから骨格情報を受信
* 人物位置を二次元平面へ変換
* DBSCANでグループを分類
* PCAでグループの主軸を計算
* 楕円の中心、長軸、短軸、角度を計算
* 体の向きからグループの平均方向を計算
* Kalman FilterなどでグループIDを追跡

この程度であれば、GPUやCUDAは必要ありません。

推奨構成は次のとおりです。

```text
Ubuntu 22.04
├── apt
│   ├── ROS 2 Humble
│   ├── rclpy
│   ├── cv_bridge
│   └── ROS 2メッセージ
│
└── ~/ros2_ws/.venv
    ├── NumPy
    ├── SciPy
    ├── scikit-learn
    └── FilterPy
```

セットアップ例です。

```bash
sudo apt update
sudo apt install -y python3-venv

cd ~/ros2_ws

/usr/bin/python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE

source /opt/ros/humble/setup.bash
source .venv/bin/activate

python -m pip install --upgrade pip setuptools wheel

python -m pip install \
    numpy==1.26.4 \
    scipy \
    scikit-learn \
    filterpy
```

ビルドします。

```bash
colcon build --symlink-install
source install/setup.bash
```

実行します。

```bash
ros2 run group_tracker group_tracker_node
```

---

# 14. 最終的な判断基準

## `venv`を選ぶ場合

次の条件であれば`venv`が適しています。

* CPUで動作する
* scikit-learn中心
* NumPy、SciPy中心
* ROS 2の`rclpy`を直接使う
* 簡単に環境を分離したい
* Dockerを使うほど依存関係が複雑ではない

## Dockerを選ぶ場合

次の条件であればDockerが適しています。

* GPUを使う
* CUDAのバージョンを固定したい
* TensorRTを使う
* PyTorchやTensorFlowを使う
* ZED SDKなど外部SDKとの組み合わせがある
* 複数PCで完全に同じ環境を再現したい
* 運用環境を固定したい

## Condaを選ぶ場合

次の条件でのみ検討します。

* ROS 2も含めてRoboStackで統一する
* apt版ROS 2を使わない
* データ分析用の独立プロセスとして動かす
* ROS 2ノードとはネットワーク通信などで分離する

---

# まとめ

ROS 2で機械学習系ライブラリを使う場合は、ROS 2本体のPython環境へ直接`pip install`しないほうが安全です。

軽量な機械学習処理では、次の構成を基本とします。

```bash
python3 -m venv --system-site-packages .venv
```

これにより、ROS 2の`rclpy`やメッセージを利用しながら、機械学習ライブラリをプロジェクト単位で管理できます。

一方、PyTorch、YOLO、CUDA、TensorRTなどを使う場合は、推論ROS 2ノードだけをDockerコンテナへ分離します。

今回のDBSCAN、PCA、楕円推定、グループID追跡については、まず`venv --system-site-packages`で構築するのが最も簡単で実用的です。
