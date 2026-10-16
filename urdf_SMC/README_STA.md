# STA rời rạc — bản thay thế STA liên tục

Giải nén vào thư mục mới. Đặt MATLAB Current Folder vào urdf_SMC.
Dừng mô phỏng cũ trước khi chạy:

```matlab
setup_sta
build_sta_model
outSTA = sim('urdf_sta_discrete', 'StopTime', '0.1');
% Khi đoạn ngắn chạy được:
% outSTA = sim('urdf_sta_discrete', 'StopTime', '2');
% outSTA = sim('urdf_sta_discrete');
```

Builder chỉ chạy lần đầu. Những lần sau chạy setup_sta rồi sim.
Không cần sửa/xóa urdf_sta.slx cũ. Mô hình mới tên urdf_sta_discrete.slx.
Nếu cần dựng lại: đóng mô hình mới và đổi tên file đó trước khi chạy builder.

## Thay đổi
- TsSTA = 0.001 s, thử nghiệm 1 kHz; không phải khẳng định phần cứng đạt 1 kHz.
- Lấy mẫu e, de, ddqRef tại TsSTA.
- Lấy mẫu q,dq vào Inverse Dynamics tại cùng TsSTA.
- z[k+1] = z[k] - TsSTA*k2STA.*sign(s[k]).
- Đầu ra v[k] dùng z[k]; Unit Delay khởi tạo z0STA và reset mỗi lần sim bình thường.
- Giữ mô-men bằng Zero-Order Hold trước plant.
- Không dùng persistent state hoặc Integrator liên tục cho STA.
- Giữ plant, trajectory và inverse dynamics gốc, không thêm sat.
- Giữ lambda/k1/k2 bản thử trước để khảo sát tác động của cách thực hiện; chưa tune.
- Bỏ InitFcn sao chép nếu có gọi setup_smc/setup_sta để tránh reset biến.
  Phải chạy setup_sta rõ ràng trước mỗi lần mô phỏng.
- Solver plant ode15s, MaxStep=maxStepSTA=TsSTA; RelTol=1e-3, AbsTol=1e-5.
  Solver vẫn có thể dùng bước nhỏ hơn do yêu cầu của plant.

## Tham số
Trong setup_sta.m: lambdaSTA, k1STA, k2STA; chu kỳ TsSTA.
Ts của trajectory (0.01 s) khác TsSTA của controller (0.001 s).
Khi thay TsSTA cần kiểm tra lại đáp ứng và gain. Không mặc định tăng TsSTA sẽ giữ ổn định.
Đây là Forward Euler tường minh: vẫn có thể gây dao động số, không kế thừa tự động
bảo đảm trượt chính xác của STA liên tục. Không có bão hòa mô-men/anti-windup mới.

## Log
outSTA.sta_e, sta_s, sta_v, sta_z, sta_tau dạng timeseries.
So sánh sai số và mô-men trên cùng lưới thời gian, cùng tải và quỹ đạo.
Muốn kiểm tra độ chính xác plant: thử maxStepSTA=TsSTA/2 và so đáp ứng.

## Kiểm tra
Đã kiểm tra cấu trúc file và phép cập nhật số trên hệ vô hướng có nhiễu hằng.
Môi trường soạn thảo không có MATLAB/Simulink: CHƯA chạy builder, compile hoặc
mô phỏng tay máy. Builder sẽ gọi Update Diagram trước khi lưu trên máy bạn.
Không khẳng định đã xử lý hết nguyên nhân chậm: cần kiểm tra tốc độ thực tế.
