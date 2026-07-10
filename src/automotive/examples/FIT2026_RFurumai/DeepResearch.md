# **5G NR-V2X環境におけるV2N2V通信モデルの現実的評価と近似モデリングに関する研究報告**

## **序論：評価環境におけるV2N2Vの理想化と研究の背景**

5G New Radio (NR) V2X技術の標準化に伴い、高度な協調運転（自動運転、隊列走行など）の実装に向けた通信インフラの検討が活発化している 。その中で、基地局（gNB）を経由するVehicle-to-Network-to-Vehicle（V2N2V）通信は、広範なカバレッジと既存の携帯電話インフラを利用できる信頼性の高さから、車車間直達通信（PC5 Sidelinkを用いたV2V）を完全に代替、あるいは単体で機能させることができるのではないかという議論が存在する 。学術的な指導や研究計画の選定において「V2N2V単体のみで十分ではないか」という問いが提起される背景には、既存のシミュレーション評価環境が内包する「V2N2Vの理想化」という重大な欠陥が存在するためである 。  
現在の一般的なシステムレベル・シミュレータや評価モデルでは、V2N2V通信の遅延を一定の固定値（定数）として処理したり、パケットロス率をゼロ（あるいは極めて低い静的な値）と仮定してシミュレーションを簡略化しているケースが多々見受けられる 。これは、5Gの無線プロトコルスタック全体（SDAP、PDCP、RLC、MAC、PHY）やバックホール、コアネットワークをパケットレベルで厳密に模擬しようとすると、車両数やトラフィック量の増加に伴いシミュレーションの計算量が指数関数的に増大し、実用的な時間内での評価が困難になるためである 。しかし、このような理想化モデルや過度に単純化された評価は、5G実ネットワークが直面する確率的ボトルネックを完全に無視しており、結果としてV2N2Vが「常に安定して超低遅延を提供する最良の通信経路」であるかのような誤った結論を導き出してしまう 。  
本報告では、3GPP標準規格（TR 38.885、TS 22.186等）および実際の都市環境における測定データを体系的に整理し、現実の5G NR-V2XネットワークにおいてV2N2V通信が直面する遅延・ロス率の劣化メカニズムを解き明かす 。さらに、計算量過多となるフルスタックのシミュレーションに代わる、軽量かつ現実的な「確率的V2N2V近似遅延・ロスモデル」を構築するための論理的根拠を提示する 。これにより、直達V2V（PC5）通信の必要性を擁護し、ハイブリッドV2X通信環境の妥当性を論理的に説明するための学術的基盤を提供する 。

## **5G NR-V2XにおけるV2Xサービス要件と通信路の比較分析**

自動運転をはじめとする高度なV2Xアプリケーションは、通信の遅延と信頼性に対して極めて厳しいサービスレベル目標（SLO）を課している 。3GPP TS 22.186において定義されているeV2X（Enhanced V2X）の主要なユースケース要件は、アプリケーションごとに要求されるペイロードサイズ、許容遅延、および信頼性が大きく異なることを示している 。

| ユースケースカテゴリ | ペイロードサイズ (Bytes) | 送信頻度 (msg/sec) | 最大エンドツーエンド遅延 (ms) | 要求信頼性 (%) | 最小通信範囲 (m) |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **隊列走行 (Platooning)** | 50 – 6000 | 2 – 50 | 10 – 25 | 90.0% – 99.99% | 80 – 350 |
| **高度運転 (Advanced Driving)** | 300 – 12000 | 10 – 100 | 3 – 100 | 90.0% – 99.999% | 360 – 700 |
| **拡張センサ (Extended Sensors)** | 最大 1600 | 10 – 1000 | 3 – 100 | 90.0% – 99.999% | 50 – 1000 |
| **遠隔運転 (Remote Driving)** | 可変（ビデオ等） | 10 – 1000 | 5 | 99.999% | — |

これらの厳しいKPIに対し、PC5インターフェースを介した「V2V直達通信」と、Uuインターフェースおよびインフラストラクチャを経由する「V2N2V通信」は、トレードオフの関係にある相補的な特性を有している 。ソウル市の都市部における大規模な実走行測定データ（5G V2N2VテストベッドとPC5直達通信の比較）からは、両経路の現実の通信品質に明確な乖離が存在することが実証されている 。

| 通信経路 | 平均遅延 (Mean Latency) | 90パーセンタイル遅延 (P\_{90}) | 99パーセンタイル遅延 (P\_{99}) | 平均パケット受信率 (PRR) | 特徴とボトルネック |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **PC5直達通信 (V2V Sidelink)** | 2.99 ms | \< 5.0 ms | \< 10.0 ms | 95.77% | シングルホップによる極小のジッター。ただし、遮蔽物や干渉に弱い |
| **Uuインフラ経由通信 (V2N2V)** | 39.40 ms | 59.86 ms | 120.33 ms | \~99.9% | 高い空間的カバレッジ。ただし、ネットワーク負荷時の遅延分散（テール遅延）が極めて大きい |

この測定結果が示すように、V2N2Vの「平均遅延 39.40\\text{ ms}」という数値自体は一見許容範囲内にあるように見える 。しかし、協調運転の安全性確保において極めて重要な「最悪値」を示す99パーセンタイル遅延（P\_{99}）に目を向けると、120.33\\text{ ms} にまで跳ね上がっている 。これは、ビデオストリーミングやリアルタイムの危険回避に必要な遅延SLO（50\\text{ ms}、信頼性 99\\%）を大幅に超過しており、V2N2V単体では最悪ケースにおける安全担保（フェイルセーフ）が機能しないことを明確に裏付けている 。

## **V2N2V通信におけるエンドツーエンド遅延要素のブレイクダウン**

V2N2V通信における現実的な遅延モデルを構築するためには、送信側車両から受信側車両に至るエンドツーエンド（E2E）遅延 (l\_{E2E}) を物理およびプロトコルレイヤの各要素に分解して理解する必要がある 。E2E遅延は、上りリンク（UL）遅延 (l\_{UL})、V2Xアプリケーションサーバー（AS）での処理遅延 (L\_{AS})、および下りリンク（DL）遅延 (l\_{DL}) の総和として定義される 。  
l\_{E2E} \= l\_{UL} \+ L\_{AS} \+ l\_{DL}  
さらに、UL遅延およびDL遅延は、無線アクセスネットワーク（RAN）、トランスポートネットワーク（TN）、およびコアネットワーク（CN）内の各要素遅延に分解される 。

l\_{UL} \= L\_{UL\\\_RAN} \+ L\_{UL\\\_TN} \+ L\_{UL\\\_CN} l\_{DL} \= L\_{DL\\\_CN} \+ L\_{DL\\\_TN} \+ L\_{DL\\\_RAN}

### **1\. 無線アクセスネットワーク遅延（L\_{RAN}）**

RANにおける遅延は、基地局（gNB）と端末（UE）間の無線物理レイヤの制御方式およびプロトコルの影響を最も強く受ける 。

* **送信要求およびスケジューリング遅延（ULのみ）：** 端末が上りデータを送信する際、リソースが事前に割り当てられていなければ、まずMAC層でスケジューリングリクエスト（SR）を送信し、gNBからの送信許可（UL Grant）を待つ必要がある 。このプロセスにより、数ミリ秒の初期遅延が発生する 。  
* **バッファステータス報告（BSR）：** 端末はgNBに対し、送信バッファに溜まっているデータ量をBSRとして報告し、gNBはこれに基づいて適切な物理上りリンク共有チャネル（PUSCH）のリソース（PRB）を割り当てる 。  
* **スロット構成と送信時間間隔（TTI）：** 5G NRではスロットが送信の基本単位となる 。サブキャリア間隔（SCS）を規定するニューメロロジー（\\mu）に依存してスロット長が変化し、一般的なサブ6 GHz帯（FR1）で用いられる \\mu \= 1 (30\\text{ kHz} SCS) の場合はスロット長が 0.5\\text{ ms}、\\mu \= 0 (15\\text{ kHz} SCS) の場合は 1\\text{ ms} となる 。  
* **再送制御（HARQ）：** 無線区間のフェージングや干渉によってパケットエラー（BLER）が発生した場合、PHY/MAC層の再送機構（HARQ）が作動する 。1回の再送ごとに、スロット周期の数倍に及ぶ遅延が追加される 。

### **2\. トランスポートおよびコアネットワーク遅延（L\_{TN}, L\_{CN}）**

gNBから送り出されたパケットは、フロントホール／バックホールを構成する光ファイバ回線（トランスポート網）を通り、5Gコアネットワーク（5GC）へと到達する 。光伝搬遅延は 1\\text{ km} あたり約 5\\ \\mu\\text{s} と極めて小さいが、ネットワークスイッチやルータでのパケット処理、およびバッファリング（キューイング遅延）が主要な変動要因となる 。5GC内のユーザプレーン機能（UPF）におけるパケット処理は、SA（スタンドアロン）環境でおよそ 100\\ \\mu\\text{s}、NSA（ノンスタンドアロン）環境でおよそ 200\\ \\mu\\text{s} の固定処理遅延を発生させるが、コアネットワーク内のトラフィック負荷によって変動する 。

### **3\. アプリケーションサーバー処理遅延（L\_{AS}）**

遠隔自動運転（ToD）や協調マニューバなどの制御判断を行うV2Xアプリケーションサーバー（V2X AS）自体が、無視できない処理遅延を生じさせる 。MEC（Multi-access Edge Computing）技術を適用してサーバーをgNBの直近に配置した場合でも、アプリケーション層でのセンサーデータの融合処理や、制御ロジックの演算には、平均して約 10\\text{ ms}（条件によってはそれ以上）の固定処理時間が消費される 。  
ネットワークトポロジーの配備（デプロイメント）手法によって、これらの遅延要素がどのように変化するかを以下の表に整理する 。

| ネットワーク構成方式 | V2X ASの物理的配置 | 平均E2E遅延 (l\_{E2E}) の目安 | トランスポート＋コア遅延 (L\_{TN} \+ L\_{CN}) | AS内処理遅延 (L\_{AS}) | 主な遅延・分散ボトルネック |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **中央集権型クラウド (Centralized)** | インターネット網外部のパブリッククラウド | 53.8 ms | 10.0 – 12.0 ms (Internet区間のみ) | \~10.0 ms | パブリックインターネットのルーティングジッター、長距離トランスポート |
| **MEC内包型コア (MEC@CN)** | 5GCコアネットワーク内 (UPFと併設) | 17.8 ms | 7.8 ms (5G網内合計) | \~10.0 ms | コアネットワーク内でのバッファリング、コアノード負荷 |
| **MEC完全エッジ型 (MEC@gNB)** | 基地局 (gNB) へのコロケーション | 15.0 – 20.0 ms | \< 1.0 ms (極小) | \~10.0 ms | AS処理遅延が支配的。および無線区間のTDDスロット待ち |
| **複数事業者跨り (Multi-MNO)** | オペレータ間の相互接続点経由 | 23.9 – 201.0 ms | 13.9 – 191.0 ms | \~10.0 ms | 事業者間の相互接続点（リモートピアリング）における経路迂回とバッファ詰まり |

## **現実の5G NRネットワークにおける遅延・パケットロス劣化要因**

「V2N2V通信が理想的である」とする評価環境が犯している最大の誤りは、下りリンク（DL）におけるMECからの送信遅延やパケットロスを静的な一定値、あるいは無視できるレベルとして扱っている点にある 。実際の5G NR物理レイヤおよびMACレイヤの仕様を考慮すると、現実の通信路には避けることのできない劣化要因が複数存在する 。

### **1\. TDDフレーム構造と構造的待機遅延**

5G NR-V2N2Vが動作するUuリンク（特に周波数帯 FR1）の多くは、Time Division Duplex（TDD：時分割複信）で運用される 。TDDでは同一の周波数帯を時間軸で分割し、上りスロット（UL）と下りスロット（DL）を交互に切り替える 。端末（車両）が「送信したいパケット」を生成した瞬間、無線チャネルが下りスロットやシンボルであった場合、端末は次の上り送信可能スロットが到来するまで無線バッファ内にパケットを保持し続けなければならない 。この「構造的待機時間」は無線物理レイヤの設定（ニューメロロジーやTDDパターン）に依存して決定論的かつ確率的に発生し、低遅延通信（URLLC）における遅延性能の下限（レイテンシーフロア）を形成する 。

### **2\. V2IとI2Vにおける劣化特性の非対称性**

セルラー網を介した通信において、上りリンク（V2I）と下りリンク（I2V）は完全に非対称な劣化要因を有している 。

* **上りリンク（V2I）のパケットロス制限：** 車載端末（UE）の最大送信電力は通常 23\\\[span\_69\](start\_span)\[span\_69\](end\_span)\[span\_71\](start\_span)\[span\_71\](end\_span)text{ dBm}（約 200\\text{ mW}）に制限されている 。そのため、車両がセル境界（基地局から離れた位置）に移動すると、急激なパスロスやシャドーイングによる受信SINRの低下に直面する 。これに伴い、パケットロス（ブロック誤り率：BLER）が急激に増大する 。つまり、上りリンクは「パケットロス」が通信品質の最大の制約となる 。  
* **下りリンク（I2V）のMAC遅延制限：** 基地局（gNB）は通常 43\\text{ dBm} から 49\\text{ dBm}（数十ワット）の極めて高い電力で送信を行うため、下りリンクにおける受信信号強度は十分に確保されやすく、接続限界付近を除き、ピュアな熱雑音によるパケットロスは発生しにくい 。しかし、gNBは配下のセル内に存在する全ての端末に対する下りデータを単一の送信機からマルチプレクスして送信しなければならない 。多数の車両に対し同時にMECからダウンリンクパケット（制御コマンドなど）が押し寄せた場合、gNBの送信MACバッファにおいて深刻な「キューイング（行列待ち）」が発生する 。結果として、下りリンク（I2V）はパケットロスではなく、スケジューラでの競合に起因する「MACアクセス遅延」および「遅延ジッター」が支配的な品質劣化要因となる 。

### **3\. マルチオペレータ（Multi-MNO）環境のルーティングボトルネック**

「道路上の全ての車両が同一の携帯キャリア（MNO）と契約している」という仮定もシミュレーション特有の理想化である 。実社会では、通信を行う車両同士が異なるMNOに接続している場合が標準的である 。MNO Aに接続された送信車両からのパケットは、MNO AのgNBとUPFを通り、さらに遠隔に設置されたパブリックなピアリングポイント（相互接続点）を通過して、MNO BのコアネットワークおよびgNBへとルーティングされる 。  
このオペレータ間の物理的なルーティング経路の迂回、および相互接続点における帯域制限やパケットバッファリングにより、平均エンドツーエンド遅延は 58\\text{ ms} 以上に達し、ネットワークの輻輳時には 201\\text{ ms} を超える致命的な遅延スパイクを発生させる 。

### **4\. トラフィックパターンの変動性とスケジューラ負荷**

自動運転車が周辺車両と環境情報を共有するために送信する協調認知メッセージ（CPM：Collective Perception Message）は、自身のセンサー（ミリ波レーダー、カメラ、LiDAR）が検知した物体の数や動的状態に応じて、パケットサイズが動的に激しく変化する 。  
一方、5Gの準持続的スケジューリング（SPS：Semi-Persistent Scheduling）は、パケットサイズが一定である「周期的なトラフィック」を想定して最適化されている 。CPMのようにパケットサイズが変動するトラフィックが流入すると、事前確保したリソースブロック（PRB）が不足し、MAC層での再スケジューリングや、変調符号化方式（MCS）の無理な変更を余儀なくされる 。これにより、チャネル使用率（CBR：Channel Busy Ratio）が急上昇し、下りリンクのキューイング遅延がさらに悪化するという悪循環に陥る 。

## **現実的なV2N2V通信特性を再現するための確率的近似モデル**

教授へ「V2N2V単体では不十分であり、評価モデルの現実化が必要である」と論理的に説明し、かつシミュレーションの「計算量過多」を回避するためには、5G NRスタック全体を精緻にシミュレーションするのではなく、確率論とキューイング理論に基づいた「現実的なV2N2V近似遅延・ロスモデル」を実装することが極めて有効である 。

### **1\. MACレイヤにおけるバッファ状態のマルコフ連鎖モデル**

端末および基地局の送信バッファにおけるパケットのキューイング挙動は、離散時間マルコフ連鎖（DTMC）を用いてモデル化できる 。バッファ内のキュー長の状態を記述する際、バッファが空（状態0）からパケットが1つ存在する（状態1）への遷移確率を \\alpha'、それ以上の状態におけるパケットの到着確率（新規生成）を \\alpha、無線チャネルを介したパケットの成功送信確率（サービス率）を \\beta と置く 。  
また、上りリンクおよび下りリンクにおける各プロトコルレイヤ（例えばSDAP層）での滞留時間 W\_{SDAP} は、到着率 \\lambda\_{SDAP} とサービス処理率 \\mu\_{SDAP} を用いた M/G/1 または G/G/1 キューイングモデルにより、以下のように定式化される 。  
W\_{SDAP} \= \\frac{1}{\\mu\_{SDAP} \- \\lambda\_{SDAP}}  
これに、複数のネットワークルータや中継スイッチをモデル化した K 個のホップノードにおけるPollaczek-Khinchineの式を組み合わせることで、コアネットワークおよびトランスポート網内の確率的遅延特性を数学的に厳密に統合できる 。

### **2\. E2E遅延の確率密度関数（PDF）モデル**

V2N2Vの累積遅延は、個々の独立したネットワーク要素（無線区間、トランスポート、コア、AS処理）における遅延の「畳み込み（Convolution）」として現れるため、非対称で裾の長い（ヘビーテールな）確率分布を示す 。

* **対数正規分布（Log-normal Distribution）モデル：** 実際の商用5Gネットワークにおける測定データや広域トランスポート遅延を最も良くフィッティングできるモデルとして、対数正規分布が広く採用されている 。遅延 t に対する確率密度関数（PDF）は以下のように定義される 。

f\_{LN}(t; \\mu\_{LN}, \\sigma\_{LN}^2) \= \\frac{1}{t \\sigma\_{LN} \\sqrt{2\\pi}} \\exp \\left( \-\\frac{(\\ln t \- \\mu\_{LN})^2}{2\\sigma\_{LN}^2} \\right), \\quad t \> 0  
ここで、\\mu\_{LN} は遅延の基礎値（レイテンシーフロア）を決定する位置パラメータ、\\sigma\_{LN} はネットワークの動的な混雑やジッターを反映する形状パラメータである 。  
\* **ガンマ分布（Gamma Distribution）モデル：** ネットワーク内の中継スイッチにおけるキューイング処理が、同一の平均到着率を持つ指数分布に従う独立なプロセスであると仮定できる場合、トータル遅延はガンマ分布を用いて表現できる 。  
f\_G(t; \\alpha\_G, \\beta\_G) \= \\frac{\\beta\_G^{\\alpha\_G}}{\\Gamma(\\alpha\_G)} t^{\\alpha\_G-1} e^{-\\beta\_G t}, \\quad t \\ge 0  
ここで、\\alpha\_G はホップ数（形状パラメータ）、\\beta\_G は各ホップのサービスレート（尺度パラメータ）である 。

* **異なるパラメータを持つガンマ分布の畳み込み（Convoluted Gamma）：** 無線区間（高ジッター）とトランスポート網（低ジッター）など、性能の異なる複数の区間を畳み込む場合、合算遅延の分布は、以下のように第1種合流型超幾何関数（\_{1}F\_{1}）を用いた解析的な表現へと展開される 。

h(z) \= \\int\_{0}^{z} \\left( \\frac{\\beta\_1^{\\alpha\_1}}{\\Gamma(\\alpha\_1)} t^{\\alpha\_1-1} e^{-\\beta\_1 t} \\right) \\cdot \\left( \\frac{\\beta\_2^{\\alpha\_2}}{\\Gamma(\\alpha\_2)} (z-t)^{\\alpha\_2-1} e^{-\\beta\_2(z-t)} \\right) dt  
この確率分布モデルをシミュレータ上に実装することで、複雑なプロトコルスタック全体を陽に計算することなく、実測データに基づいた「高精度かつ超軽量なV2N2Vの振る舞い」を再現可能となる 。これにより、現在の評価環境の「計算量過多」の問題をクリアしつつ、「不正確な理想化遅延」を排した公正な比較評価を行う道が開かれる 。

## **結論と研究設計への提言**

本調査報告が明らかにしたように、指導教員からの「V2N2V単体で十分ではないか」という問いに対しては、現行の評価環境が内包する「不正確な理想化（静的遅延と損失の無視）」という事実を定量的なデータと共に示し、論理的に反論することが可能である 。  
実際の5G NR-V2N2V環境では、以下の事象が同時に発生するため、単一の通信経路のみに依存した安全設計は極めてリスクが高い 。

1. 都市部での P\_{99} 遅延が 120.33\\text{ ms} に達し、安全制約である 50\\text{ ms} を大幅に超えること 。  
2. 上りリンク（V2I）の端末送信電力制限（23\\text{ dBm}）によるパケットロスと、下りリンク（I2V）の基地局MACにおけるキューイング遅延という「劣化の非対称性」が存在すること 。  
3. 異キャリア（Multi-MNO）を跨ぐルーティングが生じた際に、最大 201\\text{ ms} に及ぶ致命的な遅延スパイクが発生すること 。  
4. CPMなどの高度な協調運転メッセージがもたらすパケットサイズの変動が、セルのチャネルリソースを圧迫し、準持続的スケジューリングの安定性を失わせること 。 以上の論理的根拠に基づき、今後の評価環境および研究設計においては、簡略化された定数遅延を廃止し、本報告で定式化した「対数正規分布またはガンマ分布に基づく確率的V2N2V遅延近似モデル」を実装することが強く推奨される 。これにより、計算時間を現実的なレベルに抑えつつも、5G実ネットワークの不確実性を高精度に反映した評価が可能となり、最終的には「低遅延・高信頼な直達車車間通信（V2V Sidelink）と、広域カバレッジを誇るV2N2V通信を組み合わせた、相補的ハイブリッドV2X通信アーキテクチャ」の必要性と優位性を強固に実証することができる 。

#### **引用文献**

1\. 3GPP NR V2X Mode 2: Overview, Models and System-Level Evaluation \- PMC, https://pmc.ncbi.nlm.nih.gov/articles/PMC10350958/ 2\. A Comprehensive Study and Analysis of 3GPP's 5G New Radio for V2X Communication, https://www.preprints.org/manuscript/202311.1562 3\. Enhancing V2V Communication by Parsimoniously Leveraging V2N2V Path in Connected Vehicles \- PMC, https://pmc.ncbi.nlm.nih.gov/articles/PMC12899173/ 4\. msepulcre/5G-E2E-V2N2V-Latency-Models \- GitHub, https://github.com/msepulcre/5G-E2E-V2N2V-Latency-Models 5\. Direct-V2X Support with 5G Network-Based Communications: Performance, Challenges and Solutions \- ResearchGate, https://www.researchgate.net/publication/374954977\_Direct-V2X\_Support\_with\_5G\_Network-Based\_Communications\_Performance\_Challenges\_and\_Solutions 6\. End-to-End V2X Latency Modeling and Analysis in 5G Networks \- arXiv, https://arxiv.org/pdf/2201.06082 7\. ChronoRAN: Analyzing Latency in 5G Systems \- arXiv, https://arxiv.org/html/2511.21277v2 8\. 3GPP NR V2X Mode 2: Overview, Models and System-level Evaluation \- ResearchGate, https://www.researchgate.net/publication/352622021\_3GPP\_NR\_V2X\_Mode\_2\_Overview\_Models\_and\_System-level\_Evaluation 9\. Latency analysis of mobile transmission \- IETF, https://www.ietf.org/archive/id/draft-varga-detnet-mobile-latency-analysis-00.html 10\. End-to-End V2X Latency Modeling and Analysis in 5G Networks \- ResearchGate, https://www.researchgate.net/publication/365765549\_End-to-End\_V2X\_Latency\_Modeling\_and\_Analysis\_in\_5G\_Networks 11\. V2X communications within the 3GPP standards \- RIMEDO Labs, https://rimedolabs.com/blog/v2x-communications-within-the-3gpp-standards/ 12\. (PDF) Network Delay Modeling for Assisted GPS \- ResearchGate, https://www.researchgate.net/publication/260789373\_Network\_Delay\_Modeling\_for\_Assisted\_GPS 13\. Models of Network Delay \- Amazon Science, https://cdn.amazon.science/a3/a1/252dcdbb4b0193b3c0524d377f0e/models-of-network-delay.pdf 14\. Summary of V2X Performance Requirements in 3GPP TS 22.186. \- ResearchGate, https://www.researchgate.net/figure/Summary-of-V2X-Performance-Requirements-in-3GPP-TS-22186\_tbl2\_341418952 15\. 5G communication delay dataset for cloud-based vehicle planning and control \- PubMed, https://pubmed.ncbi.nlm.nih.gov/41986395/ 16\. Cross-layer latency analysis for 5G NR in V2X communications | PLOS One, https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0313772 17\. 5G NR MAC DL/UL Scheduling Algorithms \- NXG Connect, https://www.nxgconnect.com/post/5g-nr-mac-dl-ul-scheduling-algorithms 18\. How Does 5G NR V2X Mode 2 Handle Aperiodic Packets and Variable Packet Sizes? \- IRIS Unimore, https://iris.unimore.it/bitstream/11380/1274345/1/m872-molina-galan%20paper.pdf 19\. Performance evaluation of 5G New Radio low-density parity check codes over different scenarios of lognormal fading channel \- ResearchGate, https://www.researchgate.net/publication/381435838\_Performance\_evaluation\_of\_5G\_New\_Radio\_low-density\_parity\_check\_codes\_over\_different\_scenarios\_of\_lognormal\_fading\_channel 20\. (PDF) Application of Experimental Method for the Delay Estimation in Cellular Mobile Networks \- ResearchGate, https://www.researchgate.net/publication/390888431\_Application\_of\_Experimental\_Method\_for\_the\_Delay\_Estimation\_in\_Cellular\_Mobile\_Networks 21\. Incorporating telecommunication features for V2I and I2V in microscopic traffic simulation \- Search for publications in DiVA, https://liu.diva-portal.org/smash/get/diva2:2073974/FULLTEXT01.pdf 22\. ATIS.3GPP.37.885.V1530 \- AWS, https://atisorg.s3.amazonaws.com/archive/3gpp-documents/Rel15/ATIS.3GPP.37.885.V1530.pdf 23\. The Potential of Millimeter Waves for Future 5G Cellular and Vehicular Networks, https://research.unipd.it/retrieve/e14fb26f-f7f0-3de1-e053-1705fe0ac030/tesi-giordani-minor.pdf 24\. Stochastic modelling of delays and buffering in 5G-IoT ecosystems with programmable P4 switches based on BMAP \- PMC, https://pmc.ncbi.nlm.nih.gov/articles/PMC12396653/ 25\. Improving the Latency of 5G V2N2V Communications in Multi-MNO Scenarios using MEC Federation \- Uwicore \- UMH, https://uwicore.umh.es/files/paper/2022\_internacional/5G-V2N2V-MECFederation\_VTC2022\_UMHToyota\_webUwicore.pdf 26\. 16 \- ITU, https://www.itu.int/dms\_pub/itu-r/md/19/wp5a/c/R19-WP5A-C-0221\!N16\!MSW-E.docx 27\. 5G NR-V2X Scheduling Approaches for CPM Variable Size Traffic \- arXiv, https://arxiv.org/html/2606.26746v1 28\. Collective Perception: A Safety Perspective \- PMC, https://pmc.ncbi.nlm.nih.gov/articles/PMC7795526/ 29\. Analytical Model of NR-V2X Mode 2 with Re-Evaluation Mechanism \- arXiv, https://arxiv.org/html/2510.27108v1 30\. On-Ramp Merging Strategies of Connected and Automated Vehicles Considering Communication Delay \- ResearchGate, https://www.researchgate.net/publication/357759251\_On-Ramp\_Merging\_Strategies\_of\_Connected\_and\_Automated\_Vehicles\_Considering\_Communication\_Delay 31\. Obtaining a log-normal waiting time via sequential exponential or gamma distributions \- is it possible? \- Cross Validated \- Stats StackExchange, https://stats.stackexchange.com/questions/47676/obtaining-a-log-normal-waiting-time-via-sequential-exponential-or-gamma-distribu 32\. Radio Resource Allocation for Collective Perception in 5G-NR Vehicle-to-X Communication Systems \- Florian Klingler, https://fklingler.net/bib/hegde2023resource/hegde2023resource.pdf