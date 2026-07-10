# 2026-07-08 V2N2V Dense Smoke Progress

## 目的

- 200台規模の密集車両シナリオで，V2V/V2N2Vログの不自然な点を切り分ける．
- Van3Twin公式ライブラリは変更せず，example側と評価スクリプト側で確認・修正する．

## これまでに確認したこと

- 既存の `cars_250_density_wave` シナリオでは，同じ `v2v-hybrid-nr-v2n2v.cc` でCPM送信が発生する．
- 独自2クラスタシナリオでは，`nr_cpm_start_calls` は増えるため，`startCpmDissemination()` の呼び出し自体は行われている．
- 同じ独自シナリオでは，`nr_cpm_objects` は増えるため，CPM生成処理内で物標候補は作られている．
- しかし `nr_cpm_wannabe_sent=0`，`nr_cpm_sent=0`，`nr_tx=0`，`nr_cpm_size_bytes=0` のままで，実際のCPMパケット送信に到達していない．
- よって，C++の起動スクリプトや経路開始処理が主因ではなく，CPM生成後から送信前までのどこかで止まっている可能性が高い．
- 200台独自2クラスタSUMOはSUMO単体FCDでは正常に動いている．t=1から200台すべてが移動し，角度は90度/270度で正常だった．
- 古い90秒CSVでは，最後まで `nr_tx=0`，`nr_cpm_size_bytes=0`，`cpm_rx=0` で，V2V CPMは実送信されていなかった．
- 最新バイナリで200台20秒runを再取得しようとしたが，`Setup progress: SUMO setup begin` 後に進まず，実行セッションを中断した．
- sandbox内ではSUMO TraCIのremote port起動で `Operation not permitted` が出た．権限昇格で実行すると `Sumo: wait for socket` と `SUMO setup complete` まで進んだため，`SUMO setup begin` 停止はコードではなくローカルソケット制限が原因とみられる．
- 最新バイナリの20台25秒runでも，`nr_cpm_start_calls=20`，`nr_cpm_objects>0` に対し，`nr_cpm_wannabe_sent=0`，`nr_cpm_sent=0`，`nr_tx=0` が継続した．初回送信遅延ではない．

## ログ指標の読み方

- `nr_cpm_wannabe_sent` は `CPBasicService::generateAndEncodeCPM()` 内で `sendBTP()` を呼んだ直後に増える．
- `nr_cpm_sent` は `sendBTP()` の戻り値が `ACCEPTED` だった場合に増える．
- `nr_tx` / `nr_rx` は `MetricSupervisor` 側の下位通信メトリクスであり，実際に下位層へ渡った送受信数を表す．
- したがって `nr_cpm_objects>0` かつ `nr_cpm_wannabe_sent=0` は，BTP送信拒否ではなく，`sendBTP()` 呼び出し前で止まっていることを示す．
- `nr_cpm_wannabe_sent>0` かつ `nr_cpm_sent=0` ならBTP送信要求後の拒否，`nr_cpm_sent>0` かつ `nr_tx=0` ならMetricSupervisor側の計測不整合を疑う．今回の症状はそのどちらでもない．

## 現時点の仮説

- 独自2クラスタSUMOの位置，進行方向，相対座標，車線配置などが，CPM ASN.1エンコード制約に合わない値を作っている可能性がある．
- その場合，公式 `CPBasicService` 内でCPM候補は作られるが，エンコードまたは送信直前で中断される．
- 公式ライブラリは変更せず，ログとシナリオ側の値を比較して原因を特定する方針．
- `NS_LOG` にはエンコード失敗が出ていないため，ログマクロで拾えていないか，別の送信前条件で止まっている可能性も残る．

## 追加した確認用ログ

- `legacy_cpm_wannabe_sent`
- `legacy_cpm_sent`
- `nr_cpm_wannabe_sent`
- `nr_cpm_sent`
- `mec_cpm_wannabe_sent`
- `mec_cpm_sent`
- `legacy_cpm_start_calls`
- `nr_cpm_start_calls`
- `mec_cpm_start_calls`

## MEC/V2N2Vログの注意

- `mec_tx` / `mec_rx` は既存MetricSupervisor由来の値で，現在の軽量V2N2Vモデルの実送信量を直接表していない可能性が高い．
- V2N2V評価では以下を主に見る．
  - `mec_uplink_packets`
  - `mec_uplink_bytes`
  - `mec_forwarded_packets`
  - `mec_forwarded_bytes`
  - `mec_aoi_*`
  - `mec_capacity_delay_*`
  - `mec_ul_*losses`
  - `mec_dl_*losses`

## 次に確認すること

- `NS_LOG=CPBasicService=level_error` 付き短時間実行の `stdout.txt` を確認したが，`unable to encode CPM` は出ていなかった．
- ただし観測CSVでは，`nr_cpm_start_calls=20`，`nr_cpm_objects>0` に対して，`nr_cpm_wannabe_sent=0`，`nr_cpm_sent=0`，`nr_tx=0` が継続していた．
- したがって，CPM開始処理と物標生成までは到達しているが，Van3Twin側のCPM実送信カウンタ更新前で停止している状態は変わらない．
- 次は公式ライブラリを変更せず，既存シナリオと独自2クラスタシナリオのSUMO車両状態・CPM入力値の差分を比較する．

## 未解決

- 独自2クラスタシナリオでV2V CPMが送信されない根本原因．
- 200台密集シナリオを，本評価に使える安定した形へ整えること．
- V2N2V軽量モデルのログ指標と論文・発表で使う指標の対応関係の整理．

## 2026-07-09 continuation
- Added NR LDM value-range columns to cpm_input_diag.csv to diagnose CPM encode-before-send failures without editing Van3Twin official libraries.

- Built after adding VDP mandatory value diagnostics.
- Ran 4-vehicle 8s smoke: results/v2n2v_debug_4_vdp_diag_8s.

- 20-vehicle geo V2V-only smoke completed: results/v2n2v_step20_geo_v2vonly_10s.

- 20-vehicle geo V2N2V adaptive smoke completed: results/v2n2v_step20_geo_adaptive_10s.

- 20-vehicle forced-MEC adaptive smoke completed: results/v2n2v_step20_geo_adaptive_forced_mec_10s. MEC/AoI counters moved.

- Added v2n2v-only method and script mapping. 20-vehicle V2N2V-only smoke completed: results/v2n2v_step20_geo_v2n2v_only_10s.

- 50-vehicle V2N2V-only and hybrid-forced smokes completed: results/v2n2v_step50_geo_v2n2v_only_10s and results/v2n2v_step50_geo_hybrid_forced_10s.

- 100-vehicle V2N2V-only smoke was intentionally stopped early due to runtime. At t=2s: AoI_p99=839 ms, capacity_delay_p99=1267 ms, DL=68826 packets. This gives evidence that V2N2V-only tail delay grows quickly with density.

- 100-vehicle short comparison at t=2s:
  - V2N2V-only: MEC_routes=100, DL=68826, AoI_p99=839 ms, capacity_delay_p99=1267 ms.
  - Hybrid forced: MEC_routes=64, DL=12420, AoI_p99=201 ms, capacity_delay_p99=178 ms.
  - Interpretation: V2N2V-only creates much larger DL fanout and tail delay; hybrid keeps the V2N2V path as supplemental.
## Professor-report comparison artifacts

- Added `v2n2v_only_vs_hybrid_prelim_comparison.csv`.
- Added `v2n2v_only_vs_hybrid_prelim_report.md`.
- Main current evidence:
  - 50-vehicle completed runs show V2N2V-only can reach high ORR, but uses more MEC downlink packets.
  - 100-vehicle early observations show V2N2V-only has much larger downlink fanout and tail delay than Hybrid forced.
- Remaining caveat: 100-vehicle data is preliminary observation data, not a completed summary run.

## 2026-07-09 100-vehicle rerun notes

- SUMO/TraCI runs in the tool sandbox hang before `Sumo: wait for socket` because local socket creation is blocked.
- Running outside the sandbox allowed SUMO setup to complete.
- 100-vehicle V2N2V-only at t=2:
  - MEC routes: 100
  - DL packets: 68826
  - AoI p99: 839 ms
  - capacity-delay p99: 1267 ms
- 100-vehicle normal adaptive at t=2:
  - MEC routes: 0
  - DL packets: 0
  - normal threshold had not activated V2N2V yet.
- 100-vehicle Hybrid forced at t=2:
  - MEC routes: 64
  - DL packets: 16862
  - AoI p99: 206 ms
  - capacity-delay p99: 182 ms
- Interpretation: V2N2V-only already shows downlink fanout and tail-delay growth at 100 vehicles. Hybrid forced reduces the downlink fanout and tail delay, but these are early-stop stress-window observations, not completed-run summaries.
