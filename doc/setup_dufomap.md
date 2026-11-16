# DUFOMap setup manual

必要なパッケージのインストール
```bash
sudo apt update
sudo apt install -y \
    git \
    wget \
    unzip \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    gcc-10 \
    g++-10 \
    libtbb-dev \
    liblz4-dev
```

GitHubリポジトリの取得
```bash
mkdir -p ws_dufomap
cd ws_dufomap

git clone --recursive -b main --single-branch \
    https://github.com/KTH-RPL/dufomap.git
```

Python仮想環境の作成
```bash
cd ~/ws_dufomap

python3 -m venv .venv
source .venv/bin/activate
```

DUFOMapのインストール
```bash
python -m pip install --upgrade pip
python -m pip install dufomap numpy tqdm fire
```

## C++版のビルド

liblzfのインストール
```bash
sudo apt update
sudo apt install -y liblzf-dev
```

```bash
cd ~/ws_dufomap/dufomap

cmake -S . -B build \
    -DCMAKE_CXX_COMPILER=g++-11 \
    -DCMAKE_BUILD_TYPE=Release

cmake --build build -j 4
```

今後のビルド
```bash
cd ~/ws_dufomap/dufomap
cmake --build build -j 4
```

