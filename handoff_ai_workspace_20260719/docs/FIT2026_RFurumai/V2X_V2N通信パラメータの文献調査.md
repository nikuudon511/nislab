# **5G NR Uuリンクにおける伝送・再送パラメータの数理モデル化とV2N2V通信遅延の特性解析：学術文献及び3GPP標準規格に基づく実証評価**

5G New Radio (NR)の無線アクセスネットワーク（RAN）において、Uuインターフェースを介した通信品質のモデル化は、高度道路交通システム（C-ITS）や協調型自動運転のシステムレベルシミュレーションを行う上で極めて重要な要素である。従来、シミュレータ上での計算量削減や初期段階の感度分析においては、Uuリンクの初発パケット損失率やHybrid Automatic Repeat reQuest (HARQ)に起因する再送遅延パラメータが「未固定」の暫定値として扱われることが多い。  
しかし、実動的なシステム評価においては、これらの理想化された簡易モデルは不正確な遅延分布やパケットロス率を導き出し、特定の通信アーキテクチャ（例えば、ネットワークを経由するV2N2V単体）を過剰に理想化してしまうリスクを孕んでいる。本報告では、3GPP標準規格および先端学術文献に記載された実測データを紐解き、Uuリンクの伝送・再送パラメータの標準的な数値、具体的なモデル化の数理的根拠、および現実的なV2N2V遅延分布を近似的に構築するための設計手法を詳細に解説する。

## **1\. 5G NR Uuリンクにおける初発パケット損失率と残留ブロック誤り率の標準値**

無線通信リンクの信頼性を決定する物理レイヤおよびMACレイヤの主要パラメータは、初発送信時のブロック誤り率（Initial BLER）と、制限回数内のHARQ再送を実行した後に生じる残留ブロック誤り率（Residual BLER）である 。これらはリンクアダプテーション（回線設計）の制御目標値として標準化されている。

### **ブロック誤り率の数理的定義**

ブロック誤り率（BLER）は、ある一定期間内に受信側で正しく復号できなかった（CRCエラーとなった）トランスポートブロック（TB）の数を、総送信トランスポートブロック数で除したものである 。  
BLER \= \\frac{N\_{\\text{err}}}{N\_{\\text{total}}}  
復号前（FEC前）のビット誤り率（BER）を p 、1トランスポートブロックあたりの情報ビット数を k とすると、独立誤りを仮定した単純モデルにおけるBLERは以下のように表される 。  
BLER \= 1 \- (1 \- p)^k  
実システムにおいては、低密度奇偶検査（LDPC）符号等の先進的エラー訂正技術が適用されるため、ある信号対雑音干渉比（SINR）の閾値において、BLERは急峻なウォーターフォール特性（滝特性）を示す 。

### **リンクアダプテーション（LLA）による制御目標値**

5G NR Uuリンクでは、チャネル品質インジケータ（CQI）から変調符号化方式（MCS）へのマッピングを行うインナーループリンクアダプテーション（ILLA）と、HARQフィードバック（ACK/NACK）を用いてSINRにオフセットを加えるアウターループリンクアダプテーション（OLLA）が協調動作する 。

* **初発パケット損失率（Initial BLER）の標準値:** 通常のエンハンスドモバイルブロードバンド（eMBB）データチャネルにおいては、スペクトル効率とロバスト性の最適なトレードオフを実現するため、ILLA/OLLAの収束目標値（BLER\_{\\text{target}}）として **10%**（0.10）が一般的に設定される 。これに対し、超高信頼低遅延通信（URLLC）プロファイルやV2Xの一部の安全制御機能では、より低効率な専用MCSテーブルを使用することで、初発BLERターゲットを **1%**（0.01）または 10^{-5} 以下に制御する 。  
* **残留パケット損失率（Residual BLER）の標準値:** 初発送信が失敗した場合、MACレイヤのHARQエンティティがバッファされたソフトビットを合成（Incremental Redundancy等）しながら再送を行う 。一般的にHARQの最大再送回数は4回に制限されており、この制限回数内に復号が成功しない確率（残留パケット損失率）は **1%未満**（一般には **0.1%から0.5%** の間）を目標値として回線が設計される 。HARQで救済できなかったパケットは、より上位のアラーム・プロトコルであるRLCレイヤのARQ（Acknowledged Mode時）やTCPによって再送されるが、これには十数ミリ秒から数百ミリ秒の大きな遅延オーバーヘッドが伴う 。

## **2\. 5G NR UuおよびV2N2V通信におけるエンドツーエンド遅延の構成要素**

ネットワークを経由して車両間を結ぶV2N2V（Vehicle-to-Network-to-Vehicle）通信では、シミュレーション時にUuリンクを「遅延ゼロ」または「一律固定遅延」として理想化すると、実環境における遅延のばらつき（ジッター）や一時的なサービス遮断を再現できない。現実的な遅延分布をモデル化するためには、下表に示す構成要素に分解して評価する必要がある。

### **表1: V2N2V通信におけるエンドツーエンド遅延（D\_{\\text{E2E}}）の構成要素と標準数値**

| 遅延区分 | 発生源および物理的要因 | 伝送方向 | 一般的な遅延範囲 / 代表値 | 標準規格・学術文献の根拠 |
| :---- | :---- | :---- | :---- | :---- |
| **アップリンクRAN遅延 (D\_{\\text{UL\\\_RAN}})** | スケジューリング要求（SR）、送信権限（Grant）付与待ち、PUSCH送信。HARQ再送が発生した場合はそのRTT。 | 車両 \\rightarrow 基盤局 (gNB) | 1.0 ms ～ 5.0 ms（再送なし時） アップリンクはリソース制限により遅延が長期化しやすい。 |  |
| **コアネットワーク伝送遅延 (D\_{\\text{CN}})** | ユーザープレーン機能（UPF）およびスイッチングノード間の伝送・キューイング遅延。 | gNB \\rightarrow V2Xサーバー | 100 \\mus（SA構成） / 200 \\mus（NSA構成） （物理的距離およびノード負荷に依存） |  |
| **アプリケーション処理遅延 (D\_{\\text{AS}})** | V2Xアプリケーションサーバー（AS）におけるメッセージの解析、配信車両の選定、配信キューへの挿入。 | AS内部処理 | 10.0 ms（MEC配置時） 10.0 ms ～ 12.0 ms（インターネット経由時） |  |
| **ダウンリンクRAN遅延 (D\_{\\text{DL\\\_RAN}})** | PDSCH伝送、DCIスケジューリング、およびHARQフィードバック（PUCCHでのACK/NACK返送）と再送。 | gNB \\rightarrow 受信車両 | 0.23 ms ～ 3.78 ms（再送回数およびSCSに依存） |  |

### **HARQ再送に伴うRTT（ラウンドトリップタイム）**

シミュレーション上で再送制御を簡略化してモデル化する場合、HARQのRTT（1回失敗した際の追加遅延時間）は重要な指標となる。

* **LTE-Advanced FDD規格における標準値:** サブフレーム長（TTI）が1 msに固定されているため、送受信処理時間（各3 ms）と往復伝送時間（各1 ms）を合計した **8 ms** がFDDモードの固定RTTとなる 。  
* **5G NR規格におけるスケーラブルなRTT:** 5G NRではサブキャリア間隔（SCS）の柔軟な変更が可能であり、これによりRTTは大幅に短縮される 。例えば、30 kHz SCS（スロット長0.5 ms）や60 kHz SCS（スロット長0.25 ms）の適用、さらにスロット内を細分化したミニスロット（2〜7シンボル）での送信制御を行うことで、1回のHARQ再送に要する時間は **1.5 msから3.0 ms** 程度にまで圧縮される 。

## **3\. 実測値に基づくV2N2V遅延・損失特性モデルの近似構築手法**

車両物理レイヤのシミュレーションを全て動的に走らせることは、計算負荷（CPUリソースおよびシミュレーション時間）が極めて高くなる。すでに実測・検証された論文データをメタモデルとして応用し、確率分布関数（PDF）として近似することで、シミュレーションの計算量を大幅に低減しつつ、信頼性の高いV2N2Vモデルを構築することが可能である。

### **遅延特性の近似確率分布モデル**

MEC（マルチアクセスエッジコンピューティング）を利用した近接配信と、パブリッククラウドを利用した遠隔配信における実測エンドツーエンド遅延データを基に、以下の近似モデルを適用できる 。

* **MECベース配備におけるV2N2V遅延近似 (ローカルサーバー配置):** サーバーが基地局近傍（MEC）に配置されている場合、バックホール遅延が最小化される 。実測平均遅延は **15 ms ～ 20 ms**（代表値 17.8 ms、AS処理10 msを含む）と測定されている 。これは以下の対数正規分布、もしくは一様分布で近似可能である。 D\_{\\\[span\_39\](start\_span)\[span\_39\](end\_span)text{E2E, MEC}} \\sim \\text{Lognormal}(\\mu \\approx 2.8, \\sigma \\approx 0.2) \\text{ ms} あるいは、単純化モデルとして U \\text{ ms} の一様分布を用いる。  
* **クラウドベース配備におけるV2N2V遅延近似 (中央サーバー/インターネット経由):** パブリッククラウドを経由する場合、インターネット接続遅延（10 ms〜12 ms）が加算され、さらに通信事業者間の接続を跨ぐ場合は最大201 msまで遅延が長期化する 。一般的な平均遅延は **53.8 ms ～ 150 ms** である 。学術的な先行研究においては、この広域V2N2V遅延を以下の一様分布で近似的にモデリングしている 。 D\_{\\text{E2E, Cloud}} \\sim U \\text\[span\_42\](start\_span)\[span\_42\](end\_span){ ms}

### **HARQ再送およびハンドオーバによる損失・遅延のモデリング**

Uuリンクの非理想的な損失要因を近似するために、以下の2つの確率的イベントをシミュレータに実装する。

1. **HARQ再送イベントの近似:** パケットごとに、初発パケット損失率（iBLER \= 10%）の確率で第1回送信が失敗すると判定する 。失敗と判定されたパケットには、アクティブな5G NRのNumerologyに応じたHARQ RTT（例：30 kHz SCSでは3.0 ms）を追加遅延として加算し、次の再送試行を判定する 。残留パケット損失率（rBLER \= 0.5%）に達した時点でパケット破棄とする 。  
2. **ハンドオーバ（Mobility）損失イベントの近似:** 車両が基地局セル間を移動する際、ハンドオーバ処理に伴い一時的にUuリンクの伝送能力が喪失する（ハンドオーバ遅延） 。この移行期間（数十ミリ秒から百数十ミリ秒）に発生するパケット送信は、100%の確率で強制的に損失するか、あるいは最大100 ms以上の大きなジッターを伴って受信される 。シミュレーション上では、車両の移動速度とセル半径からハンドオーバ頻度を数理的に算出し、周期的またはポアソンプロセスに従ってUuリンクの接続断状態（Out-of-Sync）を発生させることで、極めて現実的なパケットロス特性を再現できる 。

## **4\. 「V2N2V単体で十分ではないか」という学術的問いに対する論理的反論**

研究内容を対外的に発表、あるいは学位論文として審査される際、審査委員（または指導教授）から投げかけられる代表的な質問が「なぜ通信制限のあるV2Vダイレクト通信（PC5）との協調が必要なのか？インフラカバレッジの広いV2N2V（Uu）単体のシステム構築で十分ではないか？」という指摘である。  
この問いに対して、これまでに収集された実測値や規格上の限界データを提示し、学術的・実証的に反論するための論理構成を以下に整理する。

### **表2: V2V (PC5) と V2N2V (Uu) の実測通信特性比較（ソウル市街地等における実測検証に基づく）**

| 評価指標 | V2Vダイレクト通信 (PC5) | V2N2V通信 (Uu \+ MEC/Cloud) | 協調システム（ハイブリッド）としての優位性 | 根拠となる文献データ |
| :---- | :---- | :---- | :---- | :---- |
| **平均遅延時間 (Mean Latency)** | **2.99 ms**（極めて低遅延） | **39.40 ms** | V2Vが極めて低い基礎遅延を提供し、V2N2Vは失われたパケットのオンデマンドなバックアップとしてのみ機能する。 |  |
| **遅延のばらつきとテール特性** | 極めて低い（ジッター僅少） | **\\text{P}\_{95}: 59.86 ms** **\\text{P}\_{99}: 120.33 ms** | V2N2Vは再送やキューイングにより遅延のばらつきが激しく、5%以上の確率で安全支援の要件（50 ms以下）から逸脱する。 |  |
| **実環境におけるパケット受信率** | 95.77%（遮蔽物や干渉によるロス） | **100%**（最終的なTCP/ARQ再送完了時） | ハイブリッド方式では、双方を動的に選択・補完することで、**99.96%** という極めて高いパケット受信率を達成する。 |  |
| **セルラー通信帯域（リソース）消費量** | 0%（ライセンス帯域を消費せず） | 100%（配信負荷が全車両に比例） | ハイブリッド方式の「選択的再送（損失時のみV2N2V利用）」により、セルラーリソース消費量をわずか **5.54%** に抑制可能。 |  |

### **反論展開の3大ロジック**

#### **1\. 5GAA安全要件に対するV2N2Vのテール遅延限界**

リアルタイム自動運転や協調型安全運転（See-Through等）において、5GAAが規定するサービス品質（SLO）は「遅延50 ms以下、信頼性99%以上」である 。  
実環境における測定データが示す通り、V2N2Vは平均遅延こそ約39 msと要件を満たすように見えるが、無線環境の変動、ハンドオーバ、コアネットワーク内でのキューイングによって遅延分布に長いテール（裾野）が生じる 。その結果、95パーセンタイル遅延は 59.86 ms、99パーセンタイルに達すると 120.33 ms となり、50 ms の許容限界を大幅に超過する 。つまり、V2N2V単体のアーキテクチャでは、最悪値付近の遅延が原因で衝突回避システムなどの超低遅延アプリケーションを安全に駆動させることができない 。

#### **2\. セルラー帯域（周波数リソース）の枯渇問題とスケーラビリティ限界**

V2N2Vによるデータ配信を「ユニキャスト」で実施する場合、送信車両からアップリンクで上げられた1つのセンサー情報（Collective Perception Messageなど）を、基地局は周辺の N 台の車両に対して個別に送信（複製してユニキャストダウンリンク）しなければならない 。  
これは、車両密度の上昇に伴ってダウンリンクのトラフィックが幾何級数的に増大することを意味する 。周波数帯域は有限（リミテッド・スペクトル）であるため、高密度シナリオにおけるV2N2V単体の構成は、容易に無線チャネルを飽和させ、さらなるパケットロスと再送遅延の悪循環を引き起こす 。  
これに対し、V2Vをプライマリパスとして使い、V2Vが遮蔽等の理由で届かなかった「一部の損失パケットのみ」をオンデマンドでV2N2Vから補填するハイブリッド方式をとれば、セルラーネットワークのデータ負荷を7分の1に削減できるなど、通信リソースの利用効率とシステムの持続可能性において圧倒的な差が生じる 。

#### **3\. リアルタイム協調認識（共同認知）における情報の「鮮度（Age of Information）」**

共同認識（Collective Perception）やセンサー共有システムにおいて、自動運転の制御判断に使用するオブジェクト情報の価値は、情報の「鮮度（Freshness）」に完全に依存する 。  
AutowareV2X等を用いた自動運転のフィールド実証実験では、V2Vダイレクト通信による直接交換が情報の鮮度を担保する最も適したアプローチであることが証明されている 。V2N2Vのみで全情報を仲介するアプローチでは、MQTT等のパブリッシュ・サブスクライブプロトコルやブローカーの処理を仲介することになり、これだけで30 ms以上の付加的な処理遅延が日常的に発生する 。ミリ秒単位の判断が求められる交差点の衝突回避等のシナリオにおいて、V2N2V単体への依存は自律走行制御における致命的なリスク因子となり得る 。

## **5\. 本分析の結論とシミュレータへの適用推奨値**

本調査に基づき、5G NR Uuリンクの不確実性を考慮した、正確かつ計算効率の高いシステムレベルシミュレータ（または数理モデル）を構築する際のパラメータ推奨設定値を以下にまとめる。

1. **初発パケット損失率の適用:** 一般通信（Infotainment等）をシミュレートする場合は初発BLER \= 10%を基準とし、アウターループ（OLLA）の実装時はステップサイズ比 \\delta\_{\\text{up}}/\\delta\_{\\text{down}} \\approx 0.111 とする 。URLLCや重要安全情報をシミュレートする場合は、初発BLER \= 1%を適用し、ステップサイズ比を約 0.0101 に設定する 。  
2. **Uu再送プロセスの簡略化表現:** シミュレーション時間短縮のため物理レイヤの詳細な復号処理を省略する場合、初発失敗時に一律 3 ms の追加遅延（60 kHz SCSを想定した5G高速HARQ）を付加し、残留パケットロス率 0.5% に達した時点で完全にパケットをロストさせるモデルが現実の実挙動に極めて近い近似となる 。  
3. **V2N2V遅延の確率分布モデルの採用:** 物理層のシミュレーションを全て走らせる代わりに、MECサーバーとの通信を前提とする近傍配信シナリオでは、実測値を反映した U \\text{ ms} の遅延分布を使用する 。クラウドサーバーを経由する遠隔通信シナリオでは、バックプレーンやインターネット伝送を模擬した U \\text{ ms} の一様分布を適用することで、物理層の演算処理を省略しながら正確なエンドツーエンドのテール遅延特性を再現可能である 。

#### **引用文献**

1\. What is BLER in 5G? (Block Error Rate Explained) \- WirelessBrew, https://wirelessbrew.com/5g-nr/difference-between-bler-for-rlm-and-throughput/ 2\. Block Error Rate \- Grokipedia, https://grokipedia.com/page/block\_error\_rate 3\. LTE Throughput Optimization: Part 2 – Spectral Efficiency – Our ..., https://ourtechplanet.com/lte-throughput-optimization-part-2-spectral-efficiency/ 4\. Block Error Rate in LTE \- Telecompedia, https://telecompedia.net/block-error-rate-in-lte/ 5\. Rethinking Reliability Using Network Coding: a Practical 5G Evaluation \- arXiv, https://arxiv.org/pdf/2508.10247 6\. Link Adaptation in 5G NR \- NXG Connect, https://www.nxgconnect.com/post/link-adaptation-in-5g-nr 7\. Impact of Link Adaptation Strategies in Ray-Traced 5G ... \- DergiPark, https://dergipark.org.tr/tr/download/article-file/5051416 8\. US20200259896A1 \- Industrial Automation with 5G and Beyond \- Google Patents, https://patents.google.com/patent/US20200259896A1/en 9\. What is BLER? | Block Error Rate | RF Essentials, https://rfessentials.com/resources/rf-glossary/bler/ 10\. US10681584B2 \- Multiple CQI reporting processes \- Google Patents, https://patents.google.com/patent/US10681584B2/en 11\. 5G HARQ Explained: Soft Combining, HARQ Processes, ACK/NACK Timing, Codebook Types, Spatial Bundling, and 4G vs 5G Differences (Article) \- Mohamed ELAdawi, https://www.mohamedeladawi.com/5g-harq-explained-soft-combining-harq-processes-ack-nack-timing-codebook-types-spatial-bundling-and-4g-vs-5g-differences-2/ 12\. HARQ Mechanisms in LTE Protocols | PDF | Data Transmission \- Scribd, https://www.scribd.com/document/890309892/20-HARQ 13\. LTE-Advanced Radio Layer 2 and RRC aspects \- 3GPP, https://www.3gpp.org/ftp/workshop/2009-12-17\_ITU-R\_IMT-Adv\_eval/docs/pdf/REV-090004%20Radio%20layer%202%20and%20RRC%20aspects.pdf 14\. RLC Protocol in 5G NR \- NXG Connect, https://www.nxgconnect.com/post/rlc-protocol-in-5g-nr 15\. Ultra-Low Latency for 5G-A Lab Trial, https://arxiv.org/pdf/1610.04362 16\. An Analytical Latency Model and Evaluation of the Capacity of 5G NR to Support V2X Services using V2N2V Communications \- arXiv, https://arxiv.org/pdf/2201.06083 17\. 5G NR Transmission Time Interval \- Devopedia, https://devopedia.org/5g-nr-transmission-time-interval 18\. End-to-End V2X Latency Modeling and Analysis in 5G Networks \- arXiv, https://arxiv.org/pdf/2201.06082 19\. 5G communication delay dataset for cloud-based vehicle planning and control \- PubMed, https://pubmed.ncbi.nlm.nih.gov/41986395/ 20\. On-Ramp Merging Strategies of Connected and Automated Vehicles Considering Communication Delay \- ResearchGate, https://www.researchgate.net/publication/357759251\_On-Ramp\_Merging\_Strategies\_of\_Connected\_and\_Automated\_Vehicles\_Considering\_Communication\_Delay 21\. 5G standalone network's reliability, one-way latency and packet loss rate analysis for URLLC implementation \- UTUPub, https://www.utupub.fi/bitstreams/78d6fc97-e321-43d4-91db-99aeff68a6fb/download 22\. Handover Delay and Packet Loss: Metrics That Matter in 5G Mobility \- PatSnap Eureka, https://eureka.patsnap.com/article/handover-delay-and-packet-loss-metrics-that-matter-in-5g-mobility 23\. Enhancing V2V Communication by Parsimoniously Leveraging V2N2V Path in Connected Vehicles \- PMC, https://pmc.ncbi.nlm.nih.gov/articles/PMC12899173/ 24\. (PDF) Latency Analysis for Real-Time Sensor Sharing Using 4G/5G C-V2X Uu Interfaces, https://www.researchgate.net/publication/369888268\_Latency\_Analysis\_for\_Real-Time\_Sensor\_Sharing\_Using\_4G5G\_C-V2X\_Uu\_Interfaces 25\. Feasibility Study of Enabling V2X Communications by LTE-Uu Radio Interface \- arXiv, https://arxiv.org/pdf/1708.00747 26\. Enhancing Reliability in Infrastructure-Based Collective Perception: A Dual-Channel Hybrid Delivery Approach With Real-Time Monitoring \- ResearchGate, https://www.researchgate.net/publication/383162326\_Enhancing\_Reliability\_in\_Infrastructure-based\_Collective\_Perception\_A\_Dual-Channel\_Hybrid\_Delivery\_Approach\_with\_Real-Time\_Monitoring 27\. Assessing the Challenges of Collective Perception via V2I Communications in High-Speed Scenarios with Open Road Testing \- arXiv, https://arxiv.org/html/2604.20489v1 28\. ETSI standard-compliant Collective Perception Service for Connected Automated Driving \- Uwicore, https://uwicore.umh.es/files/paper/2023\_internacional/FISITA2023\_posprint.pdf 29\. ken-system: AutowareV2X: Enabling V2X Communication and Collective Perception for Autonomous Driving \- IEICE, https://ken.ieice.org/ken/paper/202302223CR2/eng/ 30\. General Report on 5G System Trials in Japan from 2017 to 2020 \- 5GMF, https://5gmf.jp/wp/wp-content/uploads/2021/07/5G-TPG\_General\_Report\_2020\_R2.pdf