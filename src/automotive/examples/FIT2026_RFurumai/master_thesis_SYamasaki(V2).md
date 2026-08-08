## 修士論文

## 車両の走行環境を考慮した協調認識メッセージにおける 冗長性緩和手法

Redundancy Mitigation Rules Considering Vehicle Driving Environment

for Collective Perception Messages

同志社大学 理工学研究科情報工学専攻

2023年度 0160番

山崎 慎也

指導教員

理工学研究科情報工学専攻

ネットワーク情報システム研究室

佐藤健哉教授

2025年1月22日


## Abstract

In recent years, research on Collective Perception Service (CPS) utilizing Vehicle-to-Everything (V2X) communication has progressed to improve vehicle surrounding environment perception. European Telecom- munications Standards Institute (ETSI) is advancing the standardization of CPS and has established the specifications for Collective Perception Message (CPM), enabling vehicles and roadside units to share de- tected object information. However, the periodic broadcasting of CPMs by multiple vehicles can lead to redundant object information sharing, causing communication bandwidth congestion and potentially re- ducing perception accuracy. To address this, ETSI has proposal Redundancy Mitigation Rules (RMR) to eliminate redundant object information, but the evaluation criteria for each RMR vary, raising concerns about potential performance degradation. Additionally, the lack of consideration for Pedestrian-related ob- ject information may pose safety risks. This research proposes a method that combines three types of RMRs to extract redundant object information and prioritize the deletion of vehicle-related object information, thereby mitigating communication bandwidth congestion while enhancing the accuracy of environmental perception.The results of the simulation showed that, compared to the related methods, the proposal method yields a recognition rate for vehicles that is at most 4% lower. However, the recognition rate for pedestrians is up to 28% higher, indicating that the proposal method is effective in environments with an increasing number of pedestrians who lack communication functionality.

## Keywords

- 1. V2X Communication, 2. Collective Perception Service, 3. Redundancy Mitigation


## 目次


## 第1章 はじめに

## 1.1 背景

近年，安全かつ安心のモビリティ社会の実現のために，車両に搭載されたセンサを用いて車両の周

辺環境を認識し自律的な走行を行う自律型自動運転の研究が盛んに行われている [1–3]．しかし，車 両に搭載されたセンサで認識できる領域は限定的であり，建物や他車両などのオブジェクトによって 死角となる領域を認識することができない [4, 5]．そのため，見通しの悪い交差点や歩行者の急な飛 び出しに対して対応することが困難であるという課題がある． [URL 🔗](#page-0)

そこで車両の周辺環境の認識を向上させる技術としてCPS（Collective Perception Service）に関する

研究が注目されている [6,7]．車両や路側に設置され通信機器とセンサを搭載する路側機がセンサで検 知したオブジェクトの座標，速度，加速度，方向などのオブジェクト情報を自車両の位置情報ととも にV2X（Vehicle-to-Everything）通信を用いて他車両や路側機と共有することで，車両の周辺環境の認 識が向上し，交通の安全性の向上が期待されている [8]．現在，ETSI（European Telecommunications Standards Institute）は CPS の標準化に取り組んでおり，車両や路側機がセンサで検知したオブジェク ト情報を共有するためのメッセージである CPM（Collective Perception Message）の規格を定めてい る．CPMに含まれるオブジェクト情報は 2 種類あり，車両がセンサで検知したオブジェクト情報と 以前に他車両から受信したCPMに含まれるオブジェクト情報である．ゆえに，各車両がCPMをブ ロードキャストで送信することによって，車両は新しく検知したオブジェクト情報を送信しながら他 車両から受信したCPMに含まれるオブジェクト情報を中継することとなるため，通信範囲外の他車 両にもオブジェクト情報を共有することが可能となる．

しかし，複数の車両がCPMを定期的に送信することにより，同じオブジェクトに関する冗長なオ

ブジェクト情報を頻繁に共有する可能性がある．同じオブジェクトに関して，位置や絶対速度の変化 量が小さなオブジェクト情報は CPS において冗長である．そのような冗長なオブジェクト情報が頻 繁に送信され，ネットワークリソースを占領することによって，通信帯域が逼迫する可能性がある． 通信帯域が逼迫すると車両が必要とするCPMを受信することができなくなる可能性があり，車両の 周辺環境の認識の低下に繋がる．

この課題に対して，ETSI は冗長なオブジェクト情報をCPMから削除することでCPMのメッセー

ジサイズを縮小する手法であるRMR（Redundancy Mitigation Rule）を複数提案している．しかし，提 案されたそれぞれのRMRでは，オブジェクト情報の冗長性の評価方法が異なるため，車両台数や車 両の速度，加速度の変化によって，CPSの性能が低下する可能性がある [9]．また，提案されたRMR では歩行者を考慮していないため，歩行者に関するオブジェクト情報がCPMから削除される可能性 がある．通信機能がない歩行者がCPMで共有されなければ，車両は歩行者を認識することができな いため，車両の歩行者に対する認識が低下し，歩行者の安全性の低下に繋がる．したがって，車両台 数や歩行者数や，車両や歩行者の速度，加速度などの車両の走行環境を考慮したRMRが必要である．


## 1.2 目的

1.1 項で述べた課題に対処すべく，提案されたRMRを組み合わせて冗長性が大きなオブジェクト [URL 🔗](#page-0)

情報を抽出しCPMから削除することで，通信帯域の逼迫を低減し周辺環境の認識を向上させること を目的とする．また，歩行者の安全性を確保するために，歩行者に関するオブジェクト情報を優先し てCPMに含むことで歩行者に対する車両の認識を向上させることを目的とする．V2V通信を模した ネットワークシミュレーションによる評価を行うことで，車両と歩行者の安全性と通信帯域の逼迫の 低減についての有効性を検証する．

## 1.3 本論文の構成

まず第 2 章で ETSI が提案したRMRについて述べる．第 3 章で提案手法の概要，オブジェクト情 [URL 🔗](#page-0)

報の冗長性の評価方法，オブジェクト情報の削除方法について述べる．第4章では提案手法の有効性 を評価するための評価環境と評価結果ついて述べる．第5章では評価によって得られた結果について 考察を行う．最後に第6 章でまとめとする． [URL 🔗](#page-0)


## 第2章 関連研究

## 2.1 Frequency-based RMR

一定の時間間隔 wにおいて，あるオブジェクトに関するオブジェクト情報について受信し更新し

た回数が閾値N以上である場合に，そのオブジェクト情報をCPMから削除することでCPMのメッ セージサイズを縮小する Frequency-based RMRという手法がある [6]．図 1 に，N = 2 の場合におけ るFrequency-based RMRの概要図を示す．この手法では，受信回数が多いオブジェクト情報は，他車 両にもそのオブジェクト情報が送信されている可能性が高いため，そのようなオブジェクト情報を冗 長性が大きなオブジェクト情報と評価する．しかし，この手法では，速度，加速度や位置などのオブ ジェクト情報の品質に関わらずオブジェクト情報を削除するため，周辺環境の認識が低下する恐れが ある．また，歩行者を考慮していないため，歩行者に関するオブジェクト情報がCPMから削除され る可能性があり，歩行者の安全を確保できない可能性がある．

## 2.2 Dynamics-based RMR

一定の時間間隔 wにおいて，あるオブジェクトに関するオブジェクト情報を受信した最後の更新

から，その位置または絶対速度の変化量がそれぞれ閾値P,Sより小さい場合，そのオブジェクト情報 をCPMから削除することでCPMのメッセージサイズを縮小する Dynamics-based RMRという手法

[6]．図2に，Dynamics-based RMRの概要図を示す．この手法では，位置または絶対速度の変 [URL 🔗](#page-0)

化量が小さなオブジェクト情報は，位置または絶対速度の変化量が大きなオブジェクト情報に比べて 冗長であるため，そのようなオブジェクト情報を冗長性が大きなオブジェクト情報と評価する．しか し，位置または絶対速度の閾値P,S が小さいと同じオブジェクトに関するオブジェクト情報が頻繁に 送信されるため，通信帯域が逼迫する可能性がある．また，歩行者を考慮していないため，歩行者に 関するオブジェクト情報がCPMから削除される可能性があり，歩行者の安全を確保できない可能性 がある．

## 2.3 Distance-based RMR

一定の時間間隔wにおいて，閾値 R の距離内に存在する車両からすでにあるオブジェクトに関す

るオブジェクト情報を受信した場合，そのオブジェクト情報をCPMから削除することでCPMのメッ セージサイズを縮小するDistance-based RMRという手法がある [6]．図3に，Distance-based RMRの 概要図を示す．この手法では，近くに存在する送信元の車両から受信したオブジェクト情報は，周辺 の他車両もそのオブジェクト情報を受信している可能性が高いため，そのようなオブジェクト情報を 冗長性が大きなオブジェクト情報と評価する．しかし，この手法では，閾値が小さい場合，通信帯域 の逼迫を低減することができない．一方，閾値が大きい場合，通信帯域の逼迫を低減することができ ても，オブジェクト情報の品質が低下する可能性がある．また，歩行者を考慮していないため，歩行


*図 1: Frequency-based RMRの概要図（N= 2 の場合）*

者に関するオブジェクト情報がCPMから削除される可能性があり，歩行者の安全を確保できない可

能性がある．


*図 2: Dynamics-based RMRの概要図*

*図 3: Distance-based RMRの概要図*


## 第3章 提案手法

## 3.1 概要

提案手法の概要図を図 4 に示す．本研究では，冗長性が大きなオブジェクト情報を抽出するため [URL 🔗](#page-0)

に，2 章で述べたRMRを組み合わせてオブジェクト情報の冗長性について評価し，冗長性について 降順にオブジェクトリストのソートを行う．次に，歩行者に関するオブジェクト情報を優先してCPM に含むために，車両，歩行者の順でソートを行う．ソートされたオブジェクトリストから冗長性が大 きなオブジェクト情報を削除することで，CPMのメッセージサイズを縮小する．通信帯域逼迫率を 表すCBR（Channel Busy Ratio）[10] に応じて削除するオブジェクト情報の個数を定期的に変更する ことで，冗長性が大きなオブジェクト情報による通信帯域の逼迫を低減し車両の周辺環境の認識を向 上させる． [URL 🔗](#page-0)

## 3.2 オブジェクト情報の冗長性の評価方法

冗長性が大きなオブジェクト情報を抽出するために，オブジェクト情報の冗長性の評価方法につい

て述べる．車両は受信したオブジェクト情報，あるいはセンサで直接検知したオブジェクト情報の冗 長性を評価するために，一定の時間間隔で認識しているそれぞれのオブジェクト情報について以下の 変化量を測定する．

- 一定時間内の受信回数：N

- 最後に受信したオブジェクト情報と現在のオブジェクトの位置と絶対速度の変化量：l,v

- オブジェクトまでの距離：L

2章で述べたRMRのオブジェクト情報の冗長性の評価をもとに，Nが大きく，または l,v,Lが小さな [URL 🔗](#page-0)

オブジェクト情報は冗長性が大きなオブジェクト情報と評価し，冗長性が大きな順にオブジェクト情 報のソートを行う．最後に，車両と歩行者の順でソートを行うことでオブジェクトリストを作成する． 具体的なオブジェクト情報のソート方法を図 5 に示す．まず始めに車両は各変化量 x の単位とス ケールを揃えるために，標準化を行う．次に，それぞれのRMRの性能を考慮するために，各変化量 に対して重み付けを行う．この重みには，通信帯域逼迫率，認識率，冗長性レベルを組み合わせて

RMRの性能を定量化する指標であるSCORE [9]を用いる．全変化量をX，各変化量の標準偏差をsX， 重みをwX とすると，以上の計算過程は式 (1) となる． [URL 🔗](#page-0)

次に，各オブジェクト情報の冗長性を評価するために，オブジェクト情報の冗長性を表す冗長性R

について計算する．Nが大きく，かつ l,v,Lが小さなオブジェクト情報ほど冗長性が大きいと評価す るため，冗長性Rは式 (2) を用いて求める．


| Object ID |
| --- |
| Cc |

*図 4: 提案手法の概要図*

各オブジェクト情報の冗長性Rをもとに，オブジェクトリストを降順にソートを行う．最後に，車

両と歩行者の順でソートを行う．

## 3.3 オブジェクト情報の削除方法

CPMから削除するオブジェクト情報について，車両は定期的にCBRを測定し，ソートされたオブ

ジェクトリストの中からCBRに応じて冗長性が大きなオブジェクト情報を削除する．図 6に通信帯 域逼迫率に応じた削除するオブジェクト情報の個数を示す．CBRに応じて削除するオブジェクト情 報の個数を設定することで，通信帯域の逼迫具合に応じて冗長性が大きなオブジェクト情報を削除し CPMのメッセージサイズを縮小する． [URL 🔗](#page-0)


| Object ID | N | l | 4 | L |
| --- | --- | --- | --- | --- |
| B (pedestrian) | 1 | 5 | 5 | 3 |
| D (vehicle) | 0 | 6 | 6 | 5 |
| C (vehicle) | 2 | 3 | 3 | 10 |

## Standardization, Weighting

| Object ID | N | 1 | 4 | L |
| --- | --- | --- | --- | --- |
| B (pedestrian) | 0.00 | 0.27 | 0.27 | -1.02 |
| D (vehicle) | -1.22 | 1.07 | 1.07 | -0.34 |
| C (vehicle) | 1.22 | -1.34 | -1.34 | 1.36 |

## Calculate redundancy R, sort by vehicle, then pedestrian

| ObjectID | WN | i | V | i | R |
| --- | --- | --- | --- | --- | --- |
| D (vehicle) | -1.22 | 1.07 | 1.07 | -0.34 | 3.37 |
| C (vehicle) | 1.22 | -1.34 | -1.34 | 1.36 | -0.67 |
| B (pedestrian) | 0.00 | 0.27 | 0.27 | -1.02 | 0 |

*図 5: オブジェクト情報のソート方法*


| CBR | The Number of Deletion of Object Information |
| --- | --- |
| high |   |
| middle | Niow) |
| low | Niow |

*図 6: 削除するオブジェクト情報の個数*


## 第4章 評価

## 4.1 評価シナリオ

提案手法の有効性を評価するため，ネットワークシミュレーションを行った．シミュレータとして，

Space-Time Engineering社が提供するネットワークシミュレータScenargieを用いた．シミュレーショ ンで用いられる評価シナリオは，図 7 で示すように 400m 4 車線直線モデルとした．シミュレー ションは 50 秒間行い，計測時間をシミュレーション開始10 秒後から 40 秒後までの 30 秒間とした． 評価の比較対象は，2章で述べたFrequency-based RMR，Dynamics-based RMR，Distance-based RMR とした．なお，シミュレーションは，以下の前提条件を満たすものとする．

- 全ての車両がセンサで周辺環境を認識でき，通信機能を有する．

- センサによる誤検知は考慮しない．

- 車両が送信するCPMには，認識しているオブジェクトごとに，オブジェクトID，位置情報，検 知時刻を含んだオブジェクト情報と車両の位置情報，送信時刻を含む．

また，通信帯域が逼迫する環境と通信帯域が逼迫しない環境についてそれぞれ評価するために，1500

バイトのメッセージを20m秒周期でユニキャストで通信を行うノードを6台追加した場合と1500バ イトのメッセージを 50m秒周期でユニキャストで通信を行うノードを 2 台追加した場合の 2 つの環 境でシミュレーションを行った．通信帯域が逼迫しない環境に関しては，車両台数を300台，歩行者 数を0人，50人でシミュレーションを行い，歩行者が100人の環境においては通信帯域が逼迫する環 境におけるシミュレーションの結果に基づいて机上計算による評価を行った．

## 4.2 パラメータ

シミュレーションにおけるパラメータを表1に示す．車両台数は300台とし，40km/h - 60km/hの [URL 🔗](#page-0)

速度で走行し，歩行者数は0人，50人，100人とし，2km/h - 4km/hの速度で移動させた．RMRに関 して，時間間隔wを 1s，閾値Nを 5，閾値 P を 4m，閾値 R を 50mとした．また，オブジェクト情 報の削除個数に関して，通信帯域逼迫率が33%までにおいてNlow を10，通信帯域逼迫率が34%から

*図 7: 評価環境*


*表 1: 評価に用いたパラメータ*

パラメータ 設定

| パラメータ 設定 |   |
| --- | --- |
| 車両台数 | 300台 |
| 歩行者数 | 0人，50人，100人 |
| 車両速度 | 40km/h - 60km/h |
| 歩行者速度 | 2km/h - 4km/h |
| 時間間隔w | 1s |
| 閾値N | 5 |
| 閾値P | 4m |
| 閾値R | 50m |
| Nlow | 10 |
| Nmiddle | 20 |
| Nhigh センシング範囲 | 40 100m |
| センシング間隔 | 1s |
| 通信規格 | IEEE802.11p |
| 電波伝搬 | Two Ray Ground |

66%までにおいてNmiddle を20，通信帯域逼迫率が67%以上においてNhigh を40とした．車両のセン

サのセンシング範囲を100m，センシング間隔を1s とした．車両の通信規格には，ETSIがV2X通信 の規格として標準化している IEEE802.11p [11] を用いた． [URL 🔗](#page-0)

## 4.3 評価指標

## 4.3.1 認識率

認識率を求めることで，車両が周辺環境をどれほど認識しているかを評価し車両と歩行者の安全性

について考察する．1秒毎に評価シナリオ内のオブジェクトの総数に対する各車両がセンサあるいは CPMで認識しているオブジェクトの総数の割合を算出し，車両台数と計測時間で割った 1 台あたり の時間平均を算出した．

## 4.3.2 パケットロス率

パケットロス率を求めることで，通信帯域の逼迫についての評価を行う．車両はCPMをブロード

キャストで送信するため，通信帯域が逼迫していないパケットロス率が0%の場合において，車両が 送信した送信パケットは通信範囲内に存在する全ての他車両が受信する．したがって，i 番目の車両 の送信パケット数を si，i 番目の車両の通信範囲内に存在する他車両の台数をni とすると，パケット ロス率が0%の場合において，i 番目の車両から受信した受信パケット数 ri は，式 (3) で求まる．


パケットロス率が0%の場合における全ての車両の受信パケット数をRideal，評価シナリオ内の全ての

車両の台数をNとすると，全ての車両の受信パケット数Rideal は，式 (4) で求まる．

本研究では，全ての車両の送信パケット数をパケットロス率が0%の場合における全ての車両の受信

パケット数をRideal とした．したがって，実際の全ての車両の受信パケット数をRtrue とすると，全て の車両のパケットロス数Lall は式 (5) で求まる．

よって，全ての車両のパケットロス率 lall は，式 (6) で求まる．

本研究では，車両の通信範囲内に存在する他車両の台数 ni を評価シナリオ内の全ての車両台数N

とした．1秒毎にパケットロス率を算出し，車両台数と計測時間で割った1台あたりの時間平均を算 出した．

## 4.4 評価結果

## 4.4.1 車両に関する認識率

通信帯域が逼迫する環境において，歩行者数が0人，50人，100人のそれぞれの場合における，提案

手法，関連研究の車両に関する認識率の結果を図 8に示す．歩行者数が0人の場合，Dynamics-based RMRは74%を示し，その他の手法に関しては73%を示した．歩行者数が50人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ71%，73%，72%，71%を 示し，提案手法の車両に関する認識率が関連研究と比較すると低くなった．歩行者数が100人の場合， Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ68%，72%， 73%，69%を示した．

続いて，通信帯域が逼迫しない環境において，歩行者数が 0 人，50 人，100 人のそれぞれの場合

における，提案手法，関連研究の車両に関する認識率の結果を図 9 に示す．歩行者数が 0 人の場合， 全ての手法において 99%を示した．歩行者が 50 人の場合，Distance-based RMR は 99%を示し，そ の他の手法は 98%を示した．歩行者が 100 人の場合，Frequeny-based RMR，Dynamics-based RMR， Distance-based RMR，提案手法はそれぞれ94%，97%，99%，95%を示し，Frequeny-based RMRが他 の手法と比較すると低い結果となった．

## 4.4.2 歩行者に関する認識率

通信帯域が逼迫する環境において，歩行者数が0人，100人のそれぞれの場合における，提案手法，

関連研究の歩行者に関する認識率の結果を図 10 に示す．歩行者数が 50 人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ53%，42%，51%，68%を示 した．歩行者数が100人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR， [URL 🔗](#page-0)


提案手法はそれぞれ47%，38%，49%，66%を示した．歩行者数が50人，100人どちらの場合におい

ても，提案手法は他の手法と比較すると高くなった．

続いて，通信帯域が逼迫しない環境において，歩行者数が50人，100人のそれぞれの場合における，

提案手法，関連研究の歩行者に関する認識率の結果を図 11に示す．歩行者数が50人の場合，Frequeny- based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ95%，94%，94%，95% を示した．歩行者が100人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR， 提案手法はそれぞれ85%，86%，90%，92%を示し，Frequeny-based RMRとDynamics-based RMRは 90%を下回った．通信帯域が逼迫する環境と通信帯域が逼迫しない環境のどちらの環境においても提 案手法の歩行者に関する認識率は他の手法と比較すると高くなった． [URL 🔗](#page-0)

## 4.4.3 パケットロス率

通信帯域が逼迫する環境において，歩行者数が0人，50人，100人のそれぞれの場合における，提

案手法，関連研究のパケットロス率の結果を図 12 に示す．歩行者数が 0 人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ42%，46%，44%，49%を 示し，提案手法は他の手法と比較して高くなった．歩行者数が 50 人の場合，Frequeny-based RMR， Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ43%，48%，45%，51%を示した． 歩行者数が100人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案 手法はそれぞれ44%，49%，46%，53%を示した． [URL 🔗](#page-0)

続いて，通信帯域が逼迫しない環境において，歩行者数が0人，50人，100人のそれぞれの場合にお

ける，提案手法，関連研究のパケットロス率の結果を図 13に示す．歩行者数が0人の場合，Frequeny- based RMR，Dynamics-based RMR，Distance-based RMR，提案手法はそれぞれ2%，1%，1%，2%を示 した．歩行者が50人の場合，Frequeny-based RMR，Dynamics-based RMR，Distance-based RMR，提案 手法はそれぞれ3%，2%，2%，3%を示した．歩行者が100人の場合，Frequeny-based RMR，Dynamics- based RMR，Distance-based RMR，提案手法はそれぞれ3%，2%，2%，3%を示した．通信帯域が逼 迫する環境と通信帯域が逼迫しない環境のどちらの環境においても提案手法のパケットロス率は他の 手法と比較すると高くなった． [URL 🔗](#page-0)


*図 8: 通信帯域が逼迫する環境における車両に関する認識率*

*図 9: 通信帯域が逼迫しない環境における車両に関する認識率*


*図 10: 通信帯域が逼迫する環境における歩行者に関する認識率*

*図 11: 通信帯域が逼迫しない環境における歩行者に関する認識率*


*図 12: 通信帯域が逼迫する環境におけるパケットロス率*

*図 13: 通信帯域が逼迫する環境におけるパケットロス率*


## 第5章 考察

## 5.1 車両と歩行者の安全性に関する考察

車両と歩行者の安全性に関する考察を行う．車両に関する認識率について，提案手法では通信帯域

が逼迫する環境において歩行者数が 100 人の場合に限り，Frequeny-based RMRより高い結果となっ た．しかし，提案手法では，CPMから車両に関するオブジェクト情報を優先して削除するために，他 の環境において関連研究より低くなり車両の安全性が低下する結果となった．一方で，歩行者に関す る認識率について，提案手法では全ての環境において関連研究よりも高い結果となり，歩行者の安全 性の向上に繋がる．

全ての手法において，車両に関する認識率が歩行者に関する認識率より高くなったが，これは車両

がCPMとともに自車両の位置情報を共有するためである．通信帯域が逼迫する環境において，関連 研究と比較すると，提案手法は車両に関する認識率が最大で4%低くなるが，歩行者に関する認識率 は最大で28%高くなり，通信機能がない歩行者数が増加する環境において提案手法は有効である．

## 5.2 通信帯域の逼迫に関する考察

CPMの通信帯域の逼迫に関する考察を行う．提案手法のパケットロス率は全ての環境において関連

研究より高くなった．これは，提案手法において，通信帯域の逼迫に応じて削除するオブジェクト情 報の個数が変動することで，通信帯域が逼迫する状況に対して未然に対応できていないためである． 削除するオブジェクト情報の個数を増加させると各車両が送信するCPMのメッセージサイズは縮小 するため，通信帯域の逼迫が低減し，パケットロス率が低下する．しかし，歩行者を優先してCPM に含む提案手法では，走行環境おけるオブジェクトの歩行者の割合が増加すると，車両に関するオブ ジェクト情報がCPMに含まれないことになるため，車両に関する認識が低下する可能性がある．


## 第6章 おわりに

近年，V2X通信により車両がセンサで検知した車両や歩行者などのオブジェクトに関するオブジェ

クト情報を共有するCPSの研究が行われている．オブジェクト情報をCPMに含んで共有することで 車両の周辺環境の認識を高め，交通の安全性の向上に繋がる．しかし，複数の車両が冗長なオブジェ クト情報を送信することで通信帯域が逼迫してしまう課題がある．この課題に対処すべく，ETSIは冗 長なオブジェクト情報をCPMから削除することで CPM のメッセージサイズを縮小する手法である RMRを複数提案している．しかし，提案されたそれぞれの RMR では，オブジェクト情報の冗長性 の評価方法が異なるため，車両台数や車両の速度，加速度の変化によって，CPSの性能が低下する可 能性がある．また，提案されたRMRでは歩行者を考慮していないため，歩行者に関するオブジェク ト情報が CPMから削除される可能性があり，歩行者の安全を確保できない可能性がある．したがっ て，車両台数や歩行者数や，車両や歩行者の速度，加速度などの車両の走行環境を考慮したRMRが 必要である．

本研究では，提案された3つのRMRを組み合わせて冗長性が大きな車両に関するオブジェクト情

報を抽出しCPMから削除する手法を提案した．シミュレーションにより，提案手法，関連研究の 4 つの手法の評価を行った．車両台数が300台，歩行者数が0人，50人，100人の場合における認識率 とパケットロス率を求めることで，車両と歩行者の安全性と通信帯域の逼迫を評価した．

シミュレーションの結果により，提案手法は関連研究と比較すると，車両に関する認識率が最大で

4%低い結果となるが，歩行者に関する認識率は最大で28%高くなり，通信機能がない歩行者数の増 加する環境において提案手法は有効であることが示された．


## 謝辞

本研究を進めるにあたって，多大なご指導とご支援を賜りました同志社大学理工学部の佐藤健哉教

授に心より感謝いたします．また，研究内容について親身にアドバイスをくださった先輩方や苦楽を 共にし研鑽に勤しんだ同期をはじめとするネットワーク情報システム研究室のみなさまには，心より 感謝いたします．最後に，学校生活や研究活動を支えてくれた家族と友人への感謝を持って謝辞を締 めさせていただきます．


## 参考文献

- [1] 須田義大, 青木啓二, 自動運転車の開発動向と技術課題, 情報管理, Vol.57, No.11, pp.809-817, 2015．

- [2] 青木啓二, 自動運転車の開発動向と技術課題：2020年の自動化実現を目指して, 情報管理, Vol.60, No.4, pp.229-239, 2017.

- [3] 鎌田実, 自動運転技術・取組の最近の動向, 学術の動向, Vol.27, No.7, pp.51-55, 2022.

- [4] C.T.Chen and Y.S.Chen, Real-time approaching vehicle detection in blind-spot area, In 2009 12th International IEEE Conference on Intelligent Transportation Systems, IEEE, 2009．

- [5] Bin-Feng Lin, Yi-Ming Chan, Li-Chen Fu, Pei-Yung Hsiao, Li-An Chuang, Shin-Shinh Huang, and Min-Fang Lo, Integrating appearance and edge features for sedan vehicle detection in the blind-spot area, IEEE Transactions on Intelligent Transportation Systems, Vol. 13, No. 2, pp.737–747, 2012．

- [6] ETSI, Intelligent Transport Systems (ITS); Vehicular Communications; Basic Set of Applications; Analysis of the Collective Perception Service (CPS); Release 2, Standard, TR 103 562, V2.1.1, 2019.

- [7] Florian A. Schiegg, Ignacio Llotser, Daniel Bischoff and Georg Volk, Collective Perception: A Safety Perspective, Sensors 2021, vol.21, no.1, 2021.

- [8] Ameni Chtourou, Pierre Merdrignac and Oyunchimeg Shagdar, Collective Perception service for Con- nected Vehicles, 2021 IEEE 93rd Vehicular Technology Conference (VTC2021-Spring), 2021.

- [9] Quentin Delooz, Alexander Willecke, Keno Garlichs, Andreas-Christian Hagau, Lars Wolf, Alexey Vinel, and Andreas Festag, Analysis and evaluation of information redundancy mitigation for v2x collective perception, IEEE Access, Vol.10, pp.47076–47093, 2022.

- [10] ETSI, Intelligent Transport Systems (ITS); Decentralized Congestion Control Mechanisms for Intel- ligent Transport Systems operating in the 5 GHz range; Access layer part. TS 102 687, 14 pages, 2018.

- [11] ETSI, Intelligent Transport Systems (ITS); Access layer specification for Intelligent Transport Systems operating in the 5GHz frequency band, EN 302 663 v1.2.1, 24 pages, 2013.


## 研究業績

- [1] Shinya Yamasaki，Onur Alparslan, Kenya Sato, "Redundancy Mitigation Rules Considering Vehicle Driving Environment for Collective Perception Messages", International Conference on Intelligent Computing and its Emerging Applications (ACM ICEA2024), pp.1-3, 2024/11.

- [2] 野田虎之介，東田悠希，山崎慎也，Onur Alparslan，佐藤健哉．"ダイナミックマップシステムの ための車両走行環境に基づくエッジサーバ動的負荷分散手法，" 情報処理学会第87回全国大会， 2025．（発表予定）
