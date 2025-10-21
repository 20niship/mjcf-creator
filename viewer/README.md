# MJCF Viewer

MuJoCoを使用してMJCFファイルを可視化するためのインタラクティブなビューワーです。

## 機能

- MJCFファイルの3D可視化
- インタラクティブなカメラ操作（回転、移動、ズーム）
- リアルタイムシミュレーション
- 視点切り替え機能
- シミュレーションの一時停止/再開
- シーンのリセット機能

## インストール

このビューワーは`uv`パッケージマネージャーを使用して設定されています。

### 1. uvのインストール

まだuvをインストールしていない場合は、以下のコマンドでインストールしてください：

```bash
# macOS/Linux
curl -LsSf https://astral.sh/uv/install.sh | sh

# または、pipを使用
pip install uv
```

### 2. 依存関係のインストール

viewerディレクトリに移動して、依存関係をインストールします：

```bash
cd viewer
uv sync
```

これにより、MuJoCo Pythonパッケージとその他の必要な依存関係が自動的にインストールされます。

## 使用方法

### 基本的な使い方

```bash
cd viewer
uv run view_mjcf_file.py <mjcfファイルへのパス>
```

### 使用例

```bash
# カレントディレクトリのモデルを表示
uv run view_mjcf_file.py model.xml

# 相対パスで指定
uv run view_mjcf_file.py ../examples/output/simple_boxes_scene.xml

# 絶対パスで指定
uv run view_mjcf_file.py /path/to/your/model.mjcf
```

### オプション

- `--no-simulate`: シミュレーションを一時停止した状態で開始
- `--timestep <秒>`: シミュレーションのタイムステップを上書き（例：`--timestep 0.002`）

```bash
# シミュレーションを一時停止した状態で開始
uv run view_mjcf_file.py model.xml --no-simulate

# カスタムタイムステップで実行
uv run view_mjcf_file.py model.xml --timestep 0.001
```

## ビューワーの操作方法

### マウス操作

| 操作 | 説明 |
|------|------|
| 左クリック + ドラッグ | 視点を回転 |
| 右クリック + ドラッグ | 視点を平行移動 |
| スクロールホイール | ズームイン/アウト |
| ダブルクリック | ボディを選択 |

### キーボード操作

| キー | 説明 |
|------|------|
| `Ctrl+P` | シミュレーションの一時停止/再開 |
| `Backspace` | シミュレーションをリセット |
| `[` / `]` | カメラを切り替え |
| `0` | フリーカメラに戻る |
| `Esc` または `Ctrl+Q` | ビューワーを終了 |

### カメラの切り替え

MJCFファイルに複数のカメラが定義されている場合、`[`キーと`]`キーで切り替えることができます。

## macOSでの使用について

### M1/M2/M3 Mac（Apple Silicon）の場合

Apple Siliconを搭載したMacでは、MuJoCoのビューワーが正常に動作しない場合があります。その場合は、以下の代替方法を試してください：

#### 方法1: Rosetta 2を使用

```bash
# Rosetta 2がインストールされていることを確認
softwareupdate --install-rosetta

# x86_64アーキテクチャでPythonを実行
arch -x86_64 uv run view_mjcf_file.py model.xml
```

#### 方法2: mjpythonを使用

MuJoCoの公式バイナリには`mjpython`という専用のPythonインタープリターが含まれています。これはビューワーの互換性問題を回避できます。

1. MuJoCoをhttps://github.com/google-deepmind/mujoco/releases からダウンロード
2. `mjpython`を使用してスクリプトを実行：

```bash
# MuJoCoをダウンロード・展開後
/path/to/mujoco/bin/mjpython view_mjcf_file.py model.xml
```

### Intel Macの場合

通常、Intel Macでは追加の設定なしで動作するはずです：

```bash
cd viewer
uv run view_mjcf_file.py model.xml
```

## トラブルシューティング

### "MuJoCo Python package not available"エラー

MuJoCoがインストールされていない場合は、以下のコマンドでインストールしてください：

```bash
uv pip install mujoco
```

### ビューワーウィンドウが開かない

1. グラフィックスドライバーが最新であることを確認
2. macOSの場合、上記の「macOSでの使用について」を参照
3. OpenGLサポートを確認：
   ```bash
   python -c "import mujoco; print(mujoco.mj_versionString())"
   ```

### モデルの読み込みエラー

- ファイルパスが正しいか確認
- MJCFファイルの構文が正しいか確認
- 参照しているメッシュやテクスチャファイルが存在するか確認

## 技術情報

### 依存関係

- Python 3.8以上
- MuJoCo 3.0.0以上
- NumPy 1.20.0以上

### 対応ファイル形式

- `.xml` - MJCF XML形式
- `.mjcf` - MJCF形式

## サンプルモデル

リポジトリには以下のサンプルモデルが含まれています：

```bash
# 簡単なボックスシーン
uv run view_mjcf_file.py ../examples/output/simple_boxes_scene.xml

# マルチロボットシーン
uv run view_mjcf_file.py ../examples/output/test_multi_robot_scene.xml

# ウェアハウスシミュレーション
uv run view_mjcf_file.py ../examples/output/warehouse_simulation.xml
```

## 参考資料

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MuJoCo GitHub Repository](https://github.com/google-deepmind/mujoco)
- [MJCF Format Specification](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

## ライセンス

このプロジェクトは元のリポジトリと同じライセンスに従います。
