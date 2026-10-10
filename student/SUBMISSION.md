# Báo cáo bài nộp — Day 23 Sensor Fusion Lab

> Điền file này rồi commit. Cách nộp: [hướng dẫn nộp](../SUBMISSION.md).

## Thông tin học viên

- Họ tên: Nguyễn Đặng Nam Khánh
- MSSV: 2A202602741
- Email: khanhndn2005@gmail.com
- Link repo (fork): https://github.com/KhanhNamYeh/K4-L2L3-DAY23-NguyenDangNamKhanh-2A202602741-SensorFusion
- Commit hash nộp (`git rev-parse HEAD`): commit code + artifacts là `32af3f5ae5311d80f328aae26b0059c040cd7b45` (CP5); hash commit cuối cùng ghi trên LMS.

## Tóm tắt kết quả

- `fusion_mode` (bắt buộc `compare`), `frames`, `segment`, `seed`: `compare`, `[0, 198]`, `training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord`, seed `0`
- `detection.precision`, `detection.recall`, `detection.tp/fp/fn`: precision 0.9701, recall 0.7004; tp 519, fp 16, fn 222
- `tracking.lidar.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1503 m, matches 502, sum_sq_err 11.3436, ghost 0, missed 239, mean confirmed 2.5226
- `tracking.fused.rmse`, `matches`, `sum_sq_err`, `ghost_track_frames`, `missed_gt_frames`, `mean_confirmed_tracks`: rmse 0.1359 m, matches 502, sum_sq_err 9.2668, ghost 0, missed 239, mean confirmed 2.5226
- Giải thích khác biệt hai mode, đọc RMSE cùng số ghép và ghost/miss: hai mode có cùng detection, cùng 502 cặp ghép, cùng 0 ghost và 239 miss, cùng số confirmed track trung bình, vì camera không cộng/trừ score, không tạo và không xoá track nên vòng đời track giống hệt nhau. Khác biệt chỉ nằm ở trạng thái EKF: các lần update camera làm `sum_sq_err` giảm từ 11.34 xuống 9.27 m², RMSE giảm từ 0.150 xuống 0.136 m trên cùng tập cặp ghép nên so sánh là công bằng. 239 miss đến từ recall của detector (0.70), không phải từ bước fusion. Lưu ý đo camera là tâm hộp 2D ground-truth có nhiễu, nên cải thiện này không chứng minh chất lượng một camera detector độc lập.

Chạy từ root repo:

```bash
fusion-run-lab --config student/config/paths.yaml --fusion compare --seed 0
```

`rmse = sqrt(sum_sq_err/matches)` trên vị trí 3D của confirmed tracks ghép
một-một với GT xe trong cửa sổ BEV, gate XY **2.0 m**; `null` nếu không có cặp.
Camera dùng tâm hộp 2D ground-truth FRONT có nhiễu seeded, **không** dùng camera
detector. Kết quả này không đo hiệu quả một perception system độc lập với GT.

`grade_run.log` là JSONL, mỗi `(mode,frame)` đúng một record với các trường:
`mode`, `frame`, `det_tp`, `det_fp`, `det_fn`, `valid_gt`, `confirmed`, `matches`,
`sum_sq_err`, `ghosts`, `misses`. Đảm bảo `matches+ghosts==confirmed` và
`matches+misses==valid_gt`; tổng/trung bình record phải khớp `metrics.json`.
File per-mode `metrics_lidar.json`, `metrics_fused.json`, `grade_run_lidar.log`,
`grade_run_fused.log` được giữ để đối chiếu.

## Giải thích ngắn (Parts E–H — tự viết)

1. **Lidar 3D vs camera 2D.** Lidar đo trực tiếp vị trí: `z = (x, y, z)` (3×1), `h(x)` tuyến tính (đổi sang hệ cảm biến), `H` là ma trận 3×6 hằng, `R = diag(0.1², 0.1², 0.1²)` m². Camera đo pixel: `z = (u, v)` (2×1), `h(x)` là phép chiếu pinhole phi tuyến `u = c_i − f_i·y_s/x_s`, `v = c_j − f_j·z_s/x_s`, nên `H` là Jacobian (đạo hàm chiếu × ma trận quay) và phải tính lại theo `x`; `R = diag(5², 5²)` px². Camera không đo độ sâu và không đo vận tốc nên cột vận tốc của `H` bằng 0.
2. **Gating Mahalanobis.** Khoảng cách Euclid bỏ qua độ bất định: cùng một sai lệch mét có thể bình thường với track có `P` lớn nhưng vô lý với track chắc chắn. `d² = γᵀS⁻¹γ` chuẩn hoá theo `S = HPHᵀ + R` nên so sánh được giữa các sensor và các track. Cổng χ² (p = 0.995, bậc tự do = số chiều đo) loại các cặp không thể cùng một vật, tránh update EKF bằng đo sai (gây lệch trạng thái) và giảm gán nhầm trong greedy. Cặp ngoài FOV bị loại trước để không chiếu điểm sau lưng camera.
3. **Track-then-fuse.** Chỉ có một danh sách track; mỗi frame predict một lần, rồi `AssocL` (update bằng lidar, quản lý vòng đời), rồi `AssocC` (update bằng camera). Không gộp raw sensor trước detection. Trên log `grade_run.log`, hai mode `lidar` và `fused` có cùng `det_tp/fp/fn`, `confirmed`, `matches`, `ghosts`, `misses` ở mỗi frame (vòng đời do lidar quyết định), chỉ `sum_sq_err` khác nhau do camera update bổ sung ở mode `fused`.
4. **Camera lệch calibration.** Pixel dự đoán `h(x)` bị dịch/xoay hệ thống so với đo, nên innovation `γ` có trung bình khác 0 và cùng dấu qua nhiều frame (bias), không còn là nhiễu trắng quanh 0. `d²` tăng, nhiều cặp bị chi² gate loại, camera update kéo state lệch khỏi lidar, RMSE fused xấu đi, và sai lệch thường lớn dần khi xe ra xa hoặc gần rìa FOV.
5. **Sensor tường minh.** Khi frame rỗng, `meas_list` không cho biết đang chạy lượt lidar hay camera; vẫn phải gọi `manage_tracks(unassigned_tracks, [], sensor)` để lidar ghi miss (trong FOV) và xoá track yếu. Nếu suy ra sensor từ đo thì frame lidar rỗng không bao giờ trừ điểm và track ma tồn tại mãi. Lidar quyết định score/init/delete vì nó cho vị trí 3D tin cậy, một lượt/frame; camera 2D chỉ có góc nên không đủ để khởi tạo, và việc để camera cộng/trừ score sẽ đếm hai lần một frame. Camera chỉ hiệu chỉnh trạng thái EKF.
6. **Vòng đời track.** Track mới: `score = 1/6`, trạng thái `initialized`. Mỗi lidar hit cộng `1/6` (tối đa 1) và chuyển `tentative`; `score > 0.8` thì `confirmed`. Mỗi lidar miss trong FOV trừ `1/6`; track đã confirmed không bị hạ trạng thái chỉ vì một miss (5/6 ≈ 0.83 vẫn confirmed). Xoá khi `P[0,0]` hoặc `P[1,1] > 9` (bất kể score), hoặc confirmed có `score < 0.6`, hoặc chưa confirmed có `score <= 0`. Camera không bao giờ xoá.

## Bonus (không bắt buộc)

Liệt kê phần bonus đã làm, file bằng chứng trong `student/bonus/` và kết quả chính
(xem [RUBRIC.md](../RUBRIC.md) mục 2). Không làm thì ghi "Không".

- 

## Khai báo sử dụng AI (bắt buộc)

Ghi rõ, kể cả khi không dùng ("Không dùng AI"). Xem [RULES.md](../RULES.md) mục 2.

- Công cụ đã dùng (ChatGPT, Copilot, Claude, …): Claude Code (Claude Sonnet 5.5)
- Dùng cho phần nào (hàm, câu hỏi, debug): viết code Part E–H (`kalman.py`, `camera_fusion.py`, `association.py`, `track_management.py`), debug lỗi `float()` trên ma trận 1×1 với numpy 2.x, chạy `fusion-run-lab`, và soạn nháp báo cáo này từ `metrics.json`/`grade_run.log` thật.
- Cách bạn đã kiểm tra lại (pytest, chạy Waymo, đối chiếu công thức): `pytest student/tests -q` → 128 passed; chạy `fusion-run-lab --fusion compare --seed 0` trên frame 0–198; đối chiếu công thức EKF (`F`, `Q = dt·q·I`, `K = PHᵀS⁻¹`), pinhole và gating χ² với tài liệu; số liệu trong báo cáo lấy trực tiếp từ `metrics.json`.

## Checklist nộp

- [x] **Part E–H** trong `workspace/` đã implement; `pytest student/tests -q` không còn `failed`/`xfailed`
- [x] Part A–D: không bắt buộc sửa (hoặc ghi chú nếu bạn đã sửa)
- [x] Lần chạy chấm điểm: `--fusion compare --seed 0`, `frame_start: 0`, `frame_end: 198`
- [x] Đã commit `student/artifacts/metrics*.json` và `student/artifacts/grade_run*.log` (không sửa tay)
- [x] Đã điền đủ file này, gồm khai báo AI
- [x] Không commit dữ liệu Waymo, weights, `paths.yaml`, API key
- [x] `python tools/check_submission.py` báo `KẾT QUẢ: SẴN SÀNG NỘP`
- [ ] Đã push và nộp link repo + commit hash trên LMS ([hướng dẫn nộp](../SUBMISSION.md))
