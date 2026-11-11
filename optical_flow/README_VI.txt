QDRONE1 - OPTICAL FLOW MONITOR / MATLAB R2019a + QUARC

Pham vi da trien khai:
- Block matching anh xam camera 2, 640x480, chu ky 0.033 s.
- Lay trung binh 4x4 de giam anh ve 160x120, 35 vung so khop.
- Loai vung it texture, lap lai, khop kem, cham bien tim kiem.
- Trung vi va dong thuan de loai ngoai lai; tu huy valid khi mat frame.
- Tao ban Stabilizer_QDrone_048378_OF.slx rieng, them monitor va log IMU.
- Khong thay doi controller goc, khong tu Build/Start/Arm.

CAI DAT MOT LAN (trong MATLAB, drone Disarm va Stabilizer dung):
addpath(fullfile(tempdir,'QDroneOpticalFlow'));
test_qdrone_of;
install_qdrone_optical_flow;

CHAY:
1. Dung target Read_Cameras de tranh chiem camera 2.
2. QUARC Build model Stabilizer_QDrone_048378_OF.
3. Chay Mission Server va model OF thay cho Stabilizer goc.
   Khong chay 2 Stabilizer cung luc: chung truy cap mot phan cung dong co.
4. Giu Disarm / thao canh de kiem tra du lieu.
5. Mo STABILIZER - QDRONE / Optical Flow Monitor.
6. Nhap dup Optical Flow Camera feed de xem hinh.

FLOW STATUS (8 phan tu):
1 du: dich chuyen anh sang phai, pixel tren anh 640x480.
2 dv: dich chuyen anh xuong duoi, pixel tren anh 640x480.
3 quality: so vung inlier chia 35, trong [0,1].
4 valid: anh hop le va dong thuan; KHONG phai cho phep giu vi tri.
5 dt: khoang thoi gian giua hai frame duoc xu ly, giay.
6 age: tuoi ket qua hop le gan nhat, giay.
7 textured: so vung co texture.
8 inliers: so vung dong thuan.

LOG: To Host File duoc dat o chu ky 0.033 s.
Payload: [t; du; dv; quality; valid; dt; age; textured; inliers;
          gyro_body_x; gyro_body_y; gyro_body_z;
          attitude_x; attitude_y; attitude_z].
To Host File co the them cot timestamp tuy cau hinh QUARC.
Luoc do lay mau IMU tai chu ky frame, chua phai dong bo theo exposure camera.

GIOI HAN:
- Chi do chuyen dong anh, chua bu xoay IMU, chua do van toc m/s.
- Chua hieu chuan noi tai camera / ma tran camera-to-body / do tre chup anh.
- Chua co cam bien khoang cach xuong san trong model duoc kiem tra.
- Khong co position hold. Angle mode va Throttle van la dieu khien tay.
- Khong duoc dung du,dv lam lenh Roll/Pitch truc tiep.
- Bien tim kiem +-24 pixel/frame, chi chap nhan cuc tieu nam trong bien.
- Chua benchmark tren Intel Aero; kiem tra computation time/overrun truoc bay.

TIEP THEO DE GIU VI TRI:
Cam bien range + hieu chuan camera/IMU + bu xoay va dong bo timestamp,
kiem chung dau truc tren ban -> van toc m/s -> vong van toc -> vong vi tri,
chuyen che do thu cong/giu vi tri, xu ly mat flow/range va thu nghiem co kiem soat.
