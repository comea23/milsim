# 域六·通信 量化对拍（链路通断 / 端到端时延 / 跳数）

配对场景：§5.15.15（milsim `scenarios/comm_track_jam.txt` ↔ AFSIM
`demos/afsim_comm_track_jam/`），只比 TRACK 消息（AFSIM 侧
`WSF_TRACK_MESSAGE`，milsim 侧 `TRACK_REPORT`）。

| 侧 | 入队 | 送达 | 丢弃 | 时延均值 (s) | 时延最大 (s) |
|---|---|---|---|---|---|
| AFSIM | 59 | 59 | 0 | 0.000130 | 0.000130 |
| milsim | 30 | 30 | 0 | 0.000 | 0.000 |

**链路通断**：Jaccard=1.000 / SMC=1.000（两边逐消息全 1——milsim 不丢包是设计口径、AFSIM DISCARDED=0，结构性一致而非精度证据）。

**端到端时延**：对齐 n=30，MAE=0.000 s / TIC=1.0000。实测 milsim share 写入（sent_at=到期拍）与 comm 投递**同拍**完成（t=12 入队、t=12 投递）⇒ 端到端时延 0 s（§5.15.15 表里的'② 12.000 s'是**首次 share 时刻**，不是时延）；AFSIM radio 对 0 bits 消息近似即时（~0.13 ms）。两侧在各自节拍下都是'当拍达'——时延差不在投递机制上，在节拍设计上（share_interval / RF_COMM period vs frame_time 驱动）；§5.15.15 已裁定'只比值、不比机制'。

**跳数**：AFSIM event_output 无逐跳事件 ⇒ **不可对拍**（如实标注）；milsim 侧跳数由 `path_exists` 内部 BFS 可得，本场景单网直连恒为 1。
