# prob_robotics_2025

## 概要
　2025年度の確率ロボティクスの講義で出題された課題に基づき作成したリポジトリ
  本リポジトリでは1次元空間でのカルマンフィルタを実装した
## 実行環境
  - Ubuntu 22.04LTS
  - Python 3.11.0

## インストール
    $ https://github.com/shouzaki978/prob_robotics_2025.git
    $ cd prob_robotics_2025

## 実行方法
    $ python3 kalmanfilter.py

## アルゴリズム
　1次元空間上を移動するロボットを対象とし,
  状態として位置,速度を持つカルマンフィルタを用いる

### 状態ベクトルの定義

ロボットの状態を

$$
x_t =
\begin{pmatrix}
x_t \\
v_t
\end{pmatrix}
$$

とする．ここで，

$$x_t$$

は位置，

$$v_t$$

は速度を表す．


