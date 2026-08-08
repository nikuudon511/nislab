# 2-page Resume Outline: Hybrid V2V/V2N2V with RMR

Status: interim drafting memo, not final paper text.

This memo is for a short 2-page Japanese resume/paper.  It updates the earlier
FIT2026 draft direction, where the V2N2V path was close to an ideal link, toward
the current story: V2N2V becomes useful only when it is combined with V2V and
load-aware RMR under realistic MEC downlink fanout and freshness constraints.

## Working Title

密集高速道路環境における協調認識のためのV2V/V2N2Vハイブリッド経路制御とRMRの検討

Alternative:

MEC下りファンアウト負荷を考慮した協調認識向けV2V/V2N2Vハイブリッド経路制御

## Core Message

The proposal is not simply adding V2V bandwidth and V2N2V bandwidth.  V2V and
V2N2V have different bottlenecks:

- V2V sidelink is efficient for local broadcast, but suffers from congestion and
  packet loss in dense traffic.
- V2N2V can complement V2V, but receiver-wise MEC downlink fanout amplifies load
  and creates tail delay/AoI violations.
- Hybrid+RMR assigns roles: V2V remains the primary broadcast path, while
  V2N2V is kept as a fresh complementary path by reducing CPM payload and MEC
  downlink demand.
- The current hybrid implementation also includes importance-aware route
  control: high-priority information can be sent on both V2V and V2N2V, while
  low-priority V2N2V forwarding is probability-controlled.

## 1. はじめに

自動運転車両の安全性向上に向けて，車載センサで検出した周辺物標を
Collective Perception Message（CPM）として共有する協調認識が重要で
ある．特に高速道路の車両密集区間では，自車センサのみでは見通し外や
遮蔽後方の物標を十分に把握できないため，V2X通信による物標共有が有効
となる．

一方，協調認識では車両数の増加に伴ってCPMの送信量が増大する．NR-V2X
sidelinkによるV2Vは，1回のブロードキャストで近傍複数車両へ配信できる
利点を持つが，密集時にはチャネル混雑やパケット損失により認識率が低下
する．これに対し，セルラ網とMECを介したV2N2Vは，V2Vで届かない情報を
補完できる可能性がある．しかし，V2N2Vを単独の主要経路として用いると，
1台のCPMをMECから多数の受信車両へ転送する下りファンアウトが発生し，
MEC下り要求負荷，遅延のロングテール，AoI違反が支配的になる．

本研究では，V2VとV2N2Vの単純な帯域加算ではなく，両経路の特性差を利用
したハイブリッド経路制御を検討する．V2Vを局所ブロードキャストの主経路
とし，V2N2Vを補完経路として用いる．さらに，Redundancy Mitigation Rules
（RMR）によりCPM内の冗長物標情報を削減し，MEC下り負荷を抑制することで，
V2N2V補完経路の鮮度を維持することを狙う．

### Introduction claim to emphasize

従来の理想化されたV2N2V評価では，MEC経路が高信頼・低遅延に見えるため，
「V2N2V単体で十分ではないか」という疑問が残る．本稿では，MEC下り
ファンアウトと容量制約を考慮すると，V2N2V単体では高密度環境のCPM共有を
支えきれず，V2Vとの役割分担が必要であることを示す．

## 2. 関連研究

山崎らは，協調認識において冗長な物標情報をCPMから削除するRMRを検討し，
通信負荷を抑えつつ認識性能を維持する手法を示した．この方向性はV2Vの
チャネル混雑対策として有効であるが，V2Vのみではsidelinkの損失や混雑を
完全には回避できない．

一方，セルラ網とMECを用いたV2N2Vは，広域配信や見通し外情報の補完に
有効である可能性がある．しかし，MECを介したV2N2Vでは，V2Vブロード
キャストとは異なり，下り方向で受信車両ごとのunicast転送が必要となる．
そのため，車両密度が高い条件では，MEC下り負荷が受信対象数に比例して
増幅される．V2N2Vを理想リンクとして扱う評価では，この下りファンアウト
負荷とAoI劣化を過小評価する恐れがある．

本研究は，V2Vのブロードキャスト効率とV2N2Vの補完性を組み合わせる点，
およびRMRをMEC補完経路の鮮度維持に用いる点に特徴がある．

## 3. 提案手法

### 3.1 Overview

提案手法は，V2V sidelinkとMEC経由V2N2Vを併用するハイブリッド経路制御
である．基本方針は以下である．

- 近傍車両への効率的な配信はV2Vブロードキャストを主経路とする．
- V2Vで損失した物標や，V2Vのみでは鮮度が不足する物標をV2N2Vで補完する．
- V2N2Vは万能な代替経路ではなく，MEC下りファンアウト負荷を抑えた補完
  経路として扱う．
- RMRによりCPM内の冗長物標情報を削減し，V2Vチャネル負荷およびMEC下り
  要求負荷を低減する．

### 3.2 Object Importance

物標重要度は，距離，接近速度，TTCに基づいて定義する．近距離または短時間
で接近する物標は高重要度とし，高重要度物標ではORRだけでなく，受信側の
冗長認識レベル（RL）も確認する．低重要度物標は，厳格な冗長性よりも，
TTL内に認識できるかを主に評価する．

### 3.3 Route Control and High-Priority Dual Transmission

現在の `v2n2v-adaptive-probability` 実装では，CBRしきい値と物標重要度に
基づいて送信経路を制御する．低CBR時はV2V sidelinkを主経路とし，CBRが
警戒域または混雑域に入るとMEC経由V2N2Vを補完経路として有効化する．

特に高重要度情報については，`hybrid-high-dual-tx=true` の場合，混雑時に
V2VとV2N2Vの双方で二重送信する．実装上，NR/V2V側はprimary経路としてRMR
削減後のCPMを送信し，MEC/V2N2V側はduplicate経路としてCPMを送信する．
この二重送信は高重要度ORRやRLを改善し得る一方，MEC下り負荷を増やす可能性
もあるため，提案手法の中核要素であると同時に感度分析対象である．

低重要度情報については，MEC側の `adaptive-probability` によりV2N2V転送
確率を制御する．デフォルトでは最大確率が `0.5` であり，CBRが
`switch-cbr=0.6` から `hybrid-cbr-max=0.8` に近づくほど，低重要度情報の
V2N2V転送確率を下げる．

### 3.4 RMR on V2N2V

本評価では，V2N2V側RMRをfanout送信対象の削減ではなく，CPM payload内の
物標情報削減として扱う．すなわち，MECから誰に転送するかを削るのではなく，
CPMに含める物標数を最大 `10/20/40` 個の段階で削減し，MEC下り送信サイズを
抑制する．

This distinction is important:

- Fanout RMR reduces receiver targets and can make V2N2V look artificially
  lighter by not sending to some vehicles.
- Object-payload RMR keeps the receiver-side fanout model and tests whether CPM
  object reduction alone can make V2N2V viable.

### 3.5 Future Prediction Component

Traffic-flow prediction is planned but not yet the main evaluated component in
the current result set.  The final method should use predicted downstream load
to adjust RMR strength or V2N2V forwarding probability before congestion occurs.
In the current resume, prediction should be written as an extension/ongoing
component unless clean prediction runs are available.

## 4. 評価指標

### ORR

Object Recognition Rate（ORR）は，評価範囲内の期待物標をTTL内に認識できた
割合である．本評価では，高重要度・低重要度ともに認識TTLを `0.2 s` に統一
する．従来の一部結果では低重要度TTLを `0.5 s` としていたが，高速道路では
0.5 sの古い認識状態が約10 m級の位置誤差につながるため，最終評価では用いない．

### MEC Load and Freshness

V2N2Vの有効性は，単なるパケット到達率ではなく，MEC下り負荷とfreshnessで
評価する．主に以下を用いる．

- `mec_dl_required_mbps`
- `mec_dl_required_load_ratio`
- `mec_update_failure_rate`
- `mec_aoi_p99_ms`
- `mec_aoi_violation_rate_200ms`

`mec_dl_busy_ratio` は容量以上で飽和するため，過負荷の大きさを説明する主指標
にはしない．

### Receiver-side RL

Receiver-side RL is defined as the number of valid recognition updates within
the applicable TTL window.

- `RL=0`: 未認識
- `RL=1`: TTL内に1回認識
- `RL=2`: TTL内に2回認識
- `RL>=3`: TTL内に3回以上認識

したがって，同じ優先度・同じ分母で見れば，`RL>=1` はORRに対応し，
`RL>=2` は冗長認識率を表す．平均RLだけでは分布が分からないため，
`RL=0/1/2/>=3` のヒストグラムと p50/p90/p99 を併用する．

## 5. 結果と考察

Status: numbers are interim unless marked as complete.

### 5.1 Method Strengths and Weaknesses

| Method | Strength | Weakness | Interpretation |
|---|---|---|---|
| V2V only | Broadcast is efficient for nearby receivers; no MEC fanout | Sidelink congestion/loss remains high in dense traffic | Good baseline, but insufficient ORR under dense load |
| V2V + RMR | Reduces CPM payload and radio loss | Can delete useful objects and reduce recognition opportunities | RMR alone on V2V does not explain Hybrid gain |
| V2N2V only | Can bypass sidelink loss and use MEC as a relay | Receiver-wise downlink fanout causes large MEC DL load and AoI tail | Not sufficient when capacity/freshness constraints are modeled |
| V2N2V only + object-payload RMR | Tests whether CPM object deletion alone can save V2N2V | Payload reduction may still be too weak; strong deletion can hurt ORR | Key ablation against "V2N2V alone is enough" |
| Hybrid baseline | Combines V2V and V2N2V recognition opportunities | Without RMR, MEC updates can be stale due to downlink load | Better than V2V only, but MEC path is not fully useful |
| Hybrid + RMR | Keeps V2V broadcast as primary path; uses high-priority dual TX and RMR-controlled V2N2V complement | Needs careful TTL/RMR/route-control settings; high dual TX may increase MEC load | Main proposed direction |

### 5.2 Evidence Against "V2N2V Only Is Enough"

The old V2N2V-only baseline reached about `13.9%` ORR and showed severe MEC
freshness problems:

- MEC update failure: about `100%`
- MEC AoI p99: about `84 s`
- MEC required DL p99: about `667 Mbps`
- MEC DL capacity proxy: about `90.5 Mbps`

The current V2N2V-only + object-payload RMR run is still in progress, but early
logs already show the structural bottleneck:

- At `t=3 s`, ORR was about `18.8%`.
- MEC required DL load was about `424 Mbps`, or `469%` of the 90.468 Mbps
  capacity proxy.
- MEC update failure was `100%`.
- MEC AoI violation over 200 ms was about `96.4%`.
- MEC payload deletion was active, while fanout deletion remained `0`.

Interim reading:

V2N2V-only + RMR is configured correctly as object-payload RMR, not fanout RMR.
However, even after deleting CPM payload objects, the all-MEC path still
concentrates many receiver-wise downlink transmissions on the MEC downlink.
This suggests that V2N2V alone cannot simultaneously keep high ORR and low MEC
freshness failure in dense traffic.

### 5.3 Why Stronger RMR Alone May Not Solve V2N2V-only

If RMR is weak, MEC downlink required load remains above capacity and AoI
violations dominate.  If RMR is made too strong, the communication load may
drop, but the CPM payload itself loses useful object information, which can
limit ORR.  This creates a structural tradeoff for V2N2V-only:

```text
weak RMR -> MEC DL overload -> stale updates -> low ORR
strong RMR -> payload/object loss -> missing recognition -> low ORR
```

This is the expected argument to test with stronger RMR settings such as
`20/40/80` or `40/80/160`.

### 5.4 TTL-unified Hybrid+RMR Interim Observation

The old Hybrid+RMR result reached about `98.6%` ORR but used low-priority TTL
`0.5 s`.  A same-time interim comparison at `t=23 s` suggests that the trend is
not explained only by the permissive TTL:

| Metric | Old low TTL 0.5 | New TTL 0.2/0.2 | Difference |
|---|---:|---:|---:|
| ORR | 94.7233 | 94.2256 | -0.4977 pt |
| MEC required DL Mbps | 36.7926 | 36.5148 | -0.2778 |
| MEC DL capacity use | 40.7% | 40.4% | -0.3 pt |
| MEC AoI p99 ms | 267 | 266 | -1 |
| MEC AoI violation >200 ms | 28.6733 | 28.2287 | -0.4446 pt |

Interim reading:

TTL unification reduced ORR only slightly at this checkpoint, while MEC load and
AoI were nearly unchanged.  This suggests that the Hybrid+RMR improvement is
not solely an artifact of low-priority TTL `0.5 s`.  The full 100 s clean run is
still required before this becomes final evidence.

### 5.5 Response to "Isn't Hybrid Just Adding Bandwidth?"

The answer should be no.  The proposed Hybrid is not simple bandwidth addition.
It is role separation between two different communication models.

- V2V is a local broadcast path.  It is efficient when many nearby vehicles need
  the same CPM information, but it is vulnerable to sidelink congestion.
- V2N2V is a complementary path through MEC.  It can recover information missed
  by V2V, but it is expensive because MEC downlink forwarding is receiver-wise
  fanout.
- RMR is used to keep the V2N2V complementary path fresh, not to make V2N2V a
  universal replacement for V2V.

Suggested wording:

本提案の本質は，V2VとV2N2Vの帯域を単純に足し合わせることではない．
V2Vは局所ブロードキャストに適している一方，密集時のsidelink損失に弱い．
V2N2Vは補完経路として有効である一方，MEC下りファンアウトにより高密度時
には負荷が増幅する．提案手法では，V2Vを主経路，V2N2VをRMRで負荷制御
した補完経路として役割分担させることで，両者の弱点を同時に緩和する．

### 5.6 Route-Control Sensitivity Plan

送信経路制御は，最終的に感度分析として分離して示すべきである．特に現在の
結果には，高重要度の二重送信効果が含まれるため，Hybrid+RMRの効果をRMR単体
の効果として説明しない．

Priority sensitivity knobs:

- `hybrid-high-dual-tx=true/false`: 高重要度の二重送信有無．
- `mec-recovery-policy=staged/offload-only`: MEC補完の使い方．
- `mec-adaptive-low-max-prob`: 低重要度V2N2V転送確率の上限．
- `switch-cbr`, `release-cbr`, `hybrid-cbr-max`: V2N2V補完を開始・解除するCBRしきい値．
- `mec-min-hold-time`: 経路切替後にMEC経路を保持する最短時間．
- `RMR_DELETE_LOW/MIDDLE/HIGH`: CPM payload削減強度．

Expected reading:

- High dual TX improves high-priority ORR/RL if MEC freshness is maintained.
- High dual TX can worsen MEC DL load if payload/RMR control is too weak.
- Disabling high dual TX helps isolate whether the improvement comes from
  route diversity or from payload-load reduction.

## 6. まとめ

本稿では，高密度高速道路環境における協調認識を対象に，V2V/V2N2V
ハイブリッド経路制御とRMRを組み合わせる手法を検討した．V2Vのみでは
sidelink混雑により認識率が制限され，V2N2VのみではMEC下りファンアウト
負荷によりAoI違反と有効更新失敗が支配的となる．一方，Hybrid+RMRでは，
V2Vのブロードキャスト効率を活かしつつ，V2N2Vを補完経路として鮮度維持
することで，認識率改善が期待できる．

今後は，TTL `0.2/0.2` に統一したHybrid+RMRの完走結果，V2N2V-only +
object-payload RMRの完走結果，および強RMR条件を取得し，V2N2V単体の限界
とHybridの必要性を定量的に示す．さらに，交通流予測を用いたRMR強度制御
またはV2N2V転送確率制御を評価し，混雑が発生する前に補完経路の負荷を
抑える手法へ拡張する．あわせて，高重要度二重送信の有無や低重要度V2N2V
転送確率を感度分析し，提案手法の効果を経路制御，RMR，予測制御に分解して
説明する．

## Minimal Figure/Table Plan for 2 Pages

1. System overview figure:
   - V2V broadcast path
   - MEC V2N2V path
   - RMR payload reduction
   - MEC downlink fanout bottleneck

2. Main comparison table:
   - V2V only
   - V2N2V only
   - V2N2V only + RMR
   - Hybrid
   - Hybrid + RMR

3. Bottleneck figure:
   - MEC required DL Mbps or required/capacity ratio
   - MEC AoI p99 or AoI violation
   - ORR

4. High-priority redundancy figure if space allows:
   - `receiver_high_rl_eq0/eq1/eq2/ge3`
   - or `receiver_high_rl_ge1/ge2`

5. Route-control sensitivity table if space allows:
   - high dual TX on/off
   - low-priority MEC forwarding probability
   - RMR delete strength

## Claims to Avoid Until Clean Runs Finish

- Do not claim final `98%` ORR under TTL `0.2/0.2` until the clean run
  completes.
- Do not claim V2N2V-only + RMR definitively fails until the run completes or a
  stronger-RMR sensitivity is collected.
- Do not claim traffic-flow prediction improves the current results until
  prediction-enabled runs are available.
- Do not claim Hybrid+RMR is only an RMR effect; current hybrid results also
  include importance-aware route control and high-priority dual transmission.
