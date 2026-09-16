# Tự tune Adaptive MPC cho tay máy 3-DOF

Bộ mã bổ sung cho thư mục urdf_MPC của repo minhuster68/matlab-sim,
đối chiếu cấu trúc nguồn ở commit a1718d09181251126d7747cafed299012b25d43c.
Đây là chương trình tìm tham số bằng mô phỏng ngoại tuyến, không kết nối robot thật.

**Trạng thái:** đã kiểm tra cú pháp MATLAB bằng MISS_HIT và cấu trúc SLX nguồn.
Chưa chạy MATLAB/Simulink tại môi trường tạo bộ mã. Không có bộ tham số
“đã tối ưu” đính kèm; kết quả sẽ được tính trên máy bạn.
Bước smoke test là bắt buộc để phát hiện khác biệt phiên bản/toolbox.

## 1. Cài vào thư mục hiện có

1. Sao lưu setup_mpc.m hiện tại của bạn bằng tên khác.
2. Giải nén, chép các file .m, thư mục +mpctune và hướng dẫn này vào urdf_MPC,
   ngang hàng với urdf.slx.
3. Giữ urdf.slx, urdf.urdf, trajectory_new.mat, meshes/ và plot_mpc_trajectory.m.
4. Chọn MATLAB Current Folder là urdf_MPC. Chỉ thêm thư mục MPC vào path;
   tránh thêm cả repo bằng genpath vì PID/LQR/SMC cũng có model tên urdf.

Cần các sản phẩm vốn có của mô hình: MATLAB, Simulink, Control System Toolbox,
Model Predictive Control Toolbox, Robotics System Toolbox, Simscape,
Simscape Multibody. Không cần Optimization Toolbox, Global Optimization Toolbox,
Statistics and Machine Learning Toolbox hay Parallel Computing Toolbox.
Bộ mã chưa được xác nhận chạy đầu-cuối trên bất kỳ phiên bản MATLAB nào.

## 2. Chạy từng bước

### A. Chuẩn bị

~~~matlab
setup_mpc
~~~

Nhập robot, kiểm tra quỹ đạo, tạo A0/B0 có ý nghĩa vật lý, tính bảng mô hình
dự báo và feedforward cho toàn quỹ đạo một lần. Tiến độ in mỗi 250 mẫu.

Tạo model **mới, tên bắt đầu bằng mpc_tune_**, trong tuning_runs/.
Không ghi đè urdf.slx. Tên model mới nằm trong biến mpcModel.
Dùng model mới này cho mọi bước phía dưới, không mở nhầm model urdf gốc.

### B. Kiểm tra trước khi chạy hàng loạt

~~~matlab
test_mpc_autotune
~~~

Kiểm tra kích thước ma trận, ràng buộc mô-men, hàm chấm điểm, độ nhạy của
sai phân rồi chạy Simulink trong 2 giây mô phỏng.
Phải thấy PASS cuối cùng mới tiếp tục. Nếu lỗi, gửi nguyên thông báo;
không bỏ các assert hoặc bỏ qua lỗi QP.

### C. Thử ngân sách nhỏ

~~~matlab
result = autotune_mpc('quick');
~~~

Mặc định 12 bộ tham số, gồm baseline. Mỗi bộ chạy **đủ quỹ đạo 37 giây**.
Sau tìm kiếm, baseline và tối đa hai bộ tốt được chạy lại ở các tải kiểm định
với dung sai solver chặt hơn.
Mặc định có tối đa 12 + 3*3 = 21 lượt đầy đủ, không kể smoke test.
37 giây mô phỏng không có nghĩa là 37 giây tính toán; xem thời gian lượt đầu
để ước lượng. Khi quick chạy ổn:

~~~matlab
result = autotune_mpc('full');    % 40 ứng viên; kiểm định tính thêm
% hoặc
result = autotune_mpc(60);        % ngân sách tùy chọn >= 2
~~~

Mỗi lần gọi bắt đầu tìm kiếm mới với seed cố định; chưa có chức năng tự nối tiếp
checkpoint cũ. Ctrl+C để dừng; checkpoint giữ các lượt đã hoàn thành.

### D. Áp dụng bộ đã kiểm định

~~~matlab
apply_mpc_tuning(result.file);
out = sim(mpcModel);
plot_mpc_trajectory
~~~

Không chạy lại setup giữa apply và sim vì setup khôi phục baseline.
Mở MATLAB lại thì chạy setup rồi apply với đường dẫn file kết quả đã lưu.
Apply kiểm tra đúng quỹ đạo, mô hình, giới hạn và scale.

Không cải thiện đủ lớn thì giữ baseline; tất cả lượt không hợp lệ thì
selectedOK=false, apply từ chối áp dụng.

## 3. Tham số được tune

Tìm trên thang logarit năm nhóm:

| Nhóm | Vector đưa vào MPC |
| --- | --- |
| Tích phân sai số | Qz = Wz*[1 1 1] |
| Vị trí | Qe = We*[1 2.5 1] |
| Vận tốc | Qed = Wv*[1 1 1] |
| Mô-men phản hồi | R = Wu*[1 1 1] |
| Thay đổi mô-men phản hồi | Rdu = Wdu*[1 1 1] |

Đồng thời tìm Np trong [15 20 30 40 50], Nc trong [2 3 5 8 10],
luôn bảo đảm 1 <= Nc <= Np.
Đây là tune theo nhóm, chưa tìm độc lập cả 15 trọng số của ba khớp.
Giữ tỷ lệ các khớp giúp giảm số lượt; năm hệ số có dư thừa về hệ số nhân chung
nên không khẳng định bộ số tìm được là duy nhất.

Thuật toán: thử một số hướng có chủ đích, tìm ngẫu nhiên trong miền giới hạn,
rồi tìm cục bộ quanh bộ tốt. Không bảo đảm tối ưu toàn cục.
Mỗi lượt tạo mpcobj mới và tắt Fast Restart; không dùng lại trạng thái
ước lượng hoặc trạng thái vật lý của lượt trước.

## 4. Chấm điểm và tải

~~~matlab
cfg.trainMasses = 0.5;
cfg.validationMasses = [0 0.5 1];
~~~

Tải là **lực hướng xuống tương đương trọng lượng**, không thêm khối lượng/
quán tính vật mang theo. Chưa chứng minh kết quả đúng với tải 2/5 kg,
ma sát, nhiễu cảm biến hoặc quỹ đạo khác. Muốn khảo sát các mức khác:
sửa danh sách tải trong setup, chạy lại setup rồi tune.

Điểm ngoài MPC với hệ số cố định:

J = MSE vị trí chuẩn hóa + 0.5*MSE vận tốc chuẩn hóa
+ 0.10*đỉnh sai số vị trí chuẩn hóa bình phương
+ 0.02*thay đổi mô-men tổng chuẩn hóa bình phương
+ 0.005*mức sử dụng mô-men tổng chuẩn hóa bình phương.

Vị trí/vận tốc chia scale tương ứng; mô-men chia tauMax.
MSE tính theo tích phân thời gian trên cùng lưới mẫu. Có nhiều tải thì dùng
mean(J)+0.25*max(J). Không dùng cost nội bộ của các MPC có trọng số khác nhau
để so sánh chất lượng.

Loại lượt khi thiếu log/NaN, timeout/kết thúc sớm, QP status <= 0, cắt/vượt
mô-men, vượt rate phản hồi, sai số vị trí > 1 rad hoặc vận tốc thực > 10 rad/s.
Hai ngưỡng cuối là tiêu chí loại kết quả, không phải ràng buộc OV.

## 5. Ràng buộc bổ sung

~~~matlab
cfg.tauMax = [5 40 5];          % N*m, mô-men TỔNG
cfg.fbSlewMax = [50 200 50];    % N*m/s, chỉ phần PHẢN HỒI
~~~

**Đây là giả định mô phỏng, chưa xác minh theo động cơ thật.**
Không chuyển trực tiếp bộ điều khiển này sang phần cứng.
Chương trình không tự tăng giới hạn để làm đẹp điểm.

Với tau_total(t)=tau_FF(t)+u(t), tính giới hạn cố định bảo thủ:

~~~text
u_min = -tauMax - min_t(tau_FF(t))
u_max =  tauMax - max_t(tau_FF(t))
~~~

Lấy giao trên toàn quỹ đạo bảo đảm mọi u trong khoảng này giữ mô-men tổng
trong giới hạn; FF được giữ ZOH giữa các mẫu.
Cách này có thể hạn chế hơn ràng buộc biến thiên theo thời gian.
FF vượt giới hạn/không còn khoảng chứa 0 thì setup dừng.
Không chỉ tăng giới hạn nếu chưa kiểm tra mô hình, đơn vị, quỹ đạo và động cơ.

MV.RateMin/Max dùng fbSlewMax*Ts, đơn vị N*m mỗi mẫu.
Không bảo đảm giới hạn jerk hay rate mô-men tổng.
Saturation ở đầu vào plant là lớp chặn dự phòng; ứng viên bị cắt đều bị loại.
Do chỉ chấp nhận lượt không cắt và QP luôn thành công, kết quả chấp nhận không
dựa vào đầu vào thực khác đầu ra MPC. Đây không phải bộ điều khiển chịu lỗi
QP/saturation đã được thiết kế để triển khai thực tế.

Không tự đặt giới hạn góc vật lý vì chưa có thông số khớp. Output MPC là sai số:
giới hạn OV cho e không tương đương giới hạn q tuyệt đối.

## 6. Thay đổi trên bản Simulink riêng

- Giữ plant, dấu actual-reference, Kalman mặc định và MPC sai số.
- Thay tuyến tính hóa theo tham chiếu bằng bảng A/B tính trước, cập nhật mỗi mẫu.
  Chưa dự báo LTV qua horizon, chưa nhận dạng tải. Không dùng cách cache này nếu
  đổi sang tuyến tính hóa theo trạng thái thực.
- Feedforward không tải tính bằng inverseDynamics và giữ ZOH theo Ts.
  q/qd cũng lấy mẫu đồng bộ. Đây là phiên bản điều khiển số đã chuẩn hóa,
  không hoàn toàn giống nhánh FF nội suy liên tục của model gốc.
  Baseline và mọi ứng viên đều chạy trên cùng phiên bản mới.
- ZOH cho sáu trạng thái cơ học, Forward Euler cho tích phân sai số như block thật.
- Đồng bộ gravity/Ts, khởi tạo q=dq=0, thêm log và QP status.
- Không sửa khối lượng/quán tính robot.

Phiên bản này chỉ hỗ trợ compensatePayload=false để tune khử ngoại lực chưa biết.
Đặt true sẽ báo lỗi rõ, không giả vờ đã bù tải.

## 7. Kết quả

Trong thư mục được in ở Command Window:

- checkpoint.mat: lượt đã xong; chưa kiểm định thì không apply.
- mpc_tuning_result.mat: bộ chọn, lịch sử, cấu hình, dữ liệu kiểm định.
- trial_history.csv: điểm, tham số, thời gian và lý do loại từng lượt.
- validation_errors.png và validation_torque.png: đồ thị kiểm định.

result.parameters chứa trọng số và horizons. Phải dùng cùng ScaleFactor;
không chỉ chép riêng Q/R sang setup cũ.
Sửa cấu hình trong setup thì chạy lại setup; sửa biến cfg đang ở workspace
không tự cập nhật context đã tính.

## 8. Tài liệu API đối chiếu

- [Adaptive MPC Controller](https://www.mathworks.com/help/mpc/ref/adaptivempccontroller.html)
- [Hàm mục tiêu MPC](https://www.mathworks.com/help/mpc/ug/optimization-problem.html)
- [Discrete-Time Integrator](https://www.mathworks.com/help/simulink/slref/discretetimeintegrator.html)
- [SimulationInput.setVariable](https://www.mathworks.com/help/simulink/slref/simulink.simulationinput.setvariable.html)
- [To Workspace](https://www.mathworks.com/help/simulink/slref/toworkspace.html)
