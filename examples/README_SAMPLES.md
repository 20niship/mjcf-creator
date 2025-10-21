# MJCF Sample Files

このディレクトリには、MJCFファイルを生成するサンプルC++プログラムが含まれています。

## 概要

単一のC++プログラム（`generate_samples.cpp`）で4つの異なるサンプルシーンを生成できます：

1. **落下オブジェクトシーン**
   - チェッカーボード模様の地面
   - 空中に配置された複数の幾何形状（球体、ボックス、シリンダー、カプセル）
   - 重力によって落下するシミュレーション

2. **二段振り子**
   - 固定されたベースジョイント
   - 2つの連結された細長い棒（カプセル形状）
   - 初期位置は水平方向
   - 重力によって振り子運動

3. **4輪車両**
   - ボックス形状のシャーシ
   - 4つの円柱形の車輪
   - 各車輪がモーターでシャーシに接続
   - 前進・後退が可能

4. **URDF→MJCF変換**
   - 既存のURDFファイルを読み込み
   - MJCF形式に変換
   - URDFコンバーター機能のデモンストレーション

## ビルド方法

プロジェクトのルートディレクトリで：

```bash
mkdir -p build
cd build
cmake ..
make generate_samples
```

## 実行方法

ビルド後、`examples`ディレクトリから実行：

```bash
cd examples

# すべてのサンプルを生成（デフォルト）
../build/examples/generate_samples

# または特定のサンプルのみ生成
../build/examples/generate_samples falling    # 落下オブジェクトシーン
../build/examples/generate_samples pendulum   # 二段振り子
../build/examples/generate_samples vehicle    # 4輪車両
../build/examples/generate_samples urdf       # URDF変換

# ヘルプを表示
../build/examples/generate_samples help
```

プロジェクトのルートから実行する場合：

```bash
# すべてのサンプルを生成
./build/examples/generate_samples

# 特定のサンプルのみ
./build/examples/generate_samples falling
./build/examples/generate_samples pendulum
./build/examples/generate_samples vehicle
./build/examples/generate_samples urdf
```

## 生成されるファイル

各プログラムは `examples/output/` ディレクトリにMJCFファイルを生成します：

- `output/falling_objects.xml` - 落下オブジェクトシーン
- `output/double_pendulum.xml` - 二段振り子
- `output/vehicle.xml` - 4輪車両
- `output/urdf_converted.xml` - URDF変換結果

**注意**: これらの出力XMLファイルは `.gitignore` に追加されており、コミットされません。

## 可視化

生成されたMJCFファイルをMuJoCoビューアーで表示：

```bash
# MuJoCoがインストールされている場合
python -m mujoco.viewer examples/output/falling_objects.xml
python -m mujoco.viewer examples/output/double_pendulum.xml
python -m mujoco.viewer examples/output/vehicle.xml
python -m mujoco.viewer examples/output/urdf_converted.xml
```

## コードの構造

各サンプルプログラムは以下のパターンに従います：

```cpp
#include "mjcf/mjcf.hpp"
#include <fstream>
#include <iostream>

void create_scene() {
    // 1. Mujocoルート要素を作成
    auto mujoco = std::make_shared<mjcf::Mujoco>("model_name");
    
    // 2. オプション設定を追加
    auto option = std::make_shared<mjcf::Option>();
    option->set_integrator(mjcf::IntegratorType::RK4);
    option->set_timestep(0.01);
    
    // 3. アセット（テクスチャ、マテリアル）を追加
    auto asset = std::make_shared<mjcf::Asset>();
    // ... テクスチャとマテリアルの設定
    
    // 4. ワールドボディにジオメトリとボディを追加
    auto worldbody = std::make_shared<mjcf::Worldbody>();
    // ... ボディとジオメトリの追加
    
    // 5. すべてをMujocoに追加
    mujoco->add_children({option, asset, worldbody});
    
    // 6. XMLファイルとして保存
    std::ofstream file("output/scene.xml");
    file << mujoco->xml();
    file.close();
}

int main() {
    create_scene();
    return 0;
}
```

## 各サンプルの詳細

### 1. 落下オブジェクトシーン

**特徴**:
- チェッカーボード模様の地面（10x10メートル）
- 6つの異なるオブジェクト：
  - 赤い球体（半径0.2m）
  - 緑のボックス（0.15x0.15x0.15m）
  - 青いシリンダー（半径0.1m、高さ0.6m）
  - 黄色のカプセル（半径0.1m、長さ0.5m）
  - シアンの球体（半径0.18m）
  - マゼンタのボックス（0.2x0.1x0.15m）
- すべてのオブジェクトはfreeジョイントで2～3メートルの高さから落下
- 重力: 9.81 m/s²

### 2. 二段振り子

**特徴**:
- 固定された基準点（高さ1.5m）
- 第一リンク（赤色）：長さ1.0m、半径0.05m
- 第二リンク（青色）：長さ1.0m、半径0.05m
- 先端に黄色のマーカー
- 初期姿勢：両リンクとも水平
- ヒンジジョイント：回転制限なし
- 減衰係数：0.1

### 3. 4輪車両

**特徴**:
- シャーシ：1.2x0.6x0.3m のボックス（質量10kg）
- 4つの車輪：
  - 前輪：前方0.4m
  - 後輪：後方0.4m
  - 各車輪：半径0.15m、幅0.16m
- 各車輪にモーターを装備
  - モーターゲイン：100
  - 制御範囲：-1 ～ 1
- 摩擦係数：1.5

### 4. URDF→MJCF変換

**特徴**:
- URDFファイルの読み込み（`urdf_files/simple_robot.urdf`）
- MJCFフォーマットへの変換
- 以下を処理：
  - リンクとジョイント
  - ビジュアルと衝突ジオメトリ
  - 慣性特性
  - ジョイント制限とダイナミクス

## カスタマイズ

各サンプルはC++のMJCFライブラリを使用しているため、簡単にカスタマイズできます：

1. オブジェクトの位置、サイズ、色を変更
2. 物理パラメータ（質量、摩擦、減衰）を調整
3. 新しいオブジェクトやジョイントを追加
4. シミュレーション設定（タイムステップ、重力など）を変更

## 技術仕様

すべてのサンプルは以下の共通設定を使用：

- **統合法**: Runge-Kutta 4次 (RK4)
- **角度単位**: 度（Degree）
- **慣性**: ジオメトリから自動計算
- **重力**: 0 0 -9.81 m/s²

## トラブルシューティング

### ビルドエラー

MJCFライブラリが見つからない場合：
```bash
cd build
cmake ..
make
```

### 出力ディレクトリが存在しない

```bash
mkdir -p examples/output
```

### URDFファイルが見つからない

`examples/urdf_files/` ディレクトリにURDFファイルが存在することを確認してください。

## 参考資料

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [MJCF XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- プロジェクトの `src/mjcf/` ディレクトリでMJCF C++ライブラリのAPIを確認

## ライセンス

このサンプルコードはMIT + No Military Useライセンスの下で提供されています。
