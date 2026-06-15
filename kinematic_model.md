# แบบจำลองพลศาสตร์ 2-Body (Tractor + Full Trailer)
### 1.3 แบบจำลองคณิตศาสตร์ 2 วัตถุ (2-Body Mathematical Model)
เพื่อให้สอดคล้องกับพฤติกรรมทางกายภาพของรถพ่วงแบบก้านลาก (Drawbar Trailer หรือ Full Trailer) เราสามารถมองรถพ่วงเป็น **"รถ 1 คันที่มีล้อหน้าหมุนเลี้ยวได้"** โดยที่:
- **Drawbar (ก้านลาก)** ทำหน้าที่เป็นเพียงแขนคาน (Arm) หรือโครงสร้างที่ยึดติดกับเพลาล้อหน้า ไม่มีมวลในตัวมันเอง
- การเคลื่อนที่และมุมของ Drawbar ($\theta_d$) จะเป็นตัวกำหนดมุมเลี้ยว (Steering Angle) ของล้อหน้ารถพ่วง
- มวลทั้งหมดจะถูกรวมไว้ที่ตัวถังรถพ่วง (Trailer Body)

#### สมการความเร็วเชิงจลนศาสตร์ (Kinematic Velocity Equations)
ให้ $H_1$ เป็นจุดพ่วงท้ายรถลากจูง และ $P_f$ เป็นเพลาหน้ารถพ่วง ความเร็วของจุดทั้งสองต้องสอดคล้องกันผ่านก้านลากที่มีความยาว $L_{bar}$:

ความเร็วของจุดพ่วง $H_1$ ในกรอบอ้างอิงของรถลากจูง:
$v_{H1} = [v_x, v_y - d_h r]^T$

ความเร็วของเพลาหน้ารถพ่วง $P_f$ ในกรอบอ้างอิงรถพ่วง:
$v_{Pf} = [v_{xt}, v_{yt} + l_{tf} r_t]^T$

สมการข้อจำกัดทางความเร็วจากก้านลาก (Velocity Constraints from Drawbar):
$C_1: v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1 - v_{xt} \cos\Delta\theta_2 + (v_{yt} + l_{tf} r_t) \sin\Delta\theta_2 = 0$
$C_2: v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1 - v_{xt} \sin\Delta\theta_2 - (v_{yt} + l_{tf} r_t) \cos\Delta\theta_2 - L_{bar} r_d = 0$
เมื่อ $\Delta\theta_1 = \theta_0 - \theta_d$ และ $\Delta\theta_2 = \theta_t - \theta_d$

#### สมดุลแรงแบบนิวตัน-ออยเลอร์ (Newton-Euler Equations of Motion)
ระบบประกอบด้วย 8 ตัวแปรไม่ทราบค่า: $[\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t, \dot{r}_d, F_d]^T$
โดยที่ $F_d$ คือแรงดึง/อัด (Tension/Compression) ตามแนวแกนของ Drawbar

**สมการรถลากจูง (Tractor):**
1. $m_0 \dot{v}_x - F_d \cos(\theta_d - \theta_0) = F_{xr} + m_0 v_y r - F_{drag\_x}$
2. $m_0 \dot{v}_y - F_d \sin(\theta_d - \theta_0) = F_{yf} + F_{yr} - m_0 v_x r$
3. $I_0 \dot{r} + d_h F_d \sin(\theta_d - \theta_0) = l_f F_{yf} - l_r F_{yr}$

**สมการรถพ่วง (Full Trailer):**
4. $m_1 \dot{v}_{xt} + F_d \cos(\theta_d - \theta_t) = F_{yf\_front} \sin(\theta_d - \theta_t) + m_1 v_{yt} r_t - F_{drag\_t}$
5. $m_1 \dot{v}_{yt} + F_d \sin(\theta_d - \theta_t) = -F_{yf\_front} \cos(\theta_d - \theta_t) + F_{y\_rear} - m_1 v_{xt} r_t$
6. $I_1 \dot{r}_t + l_{tf} \sin(\theta_d - \theta_t) F_d = -l_{tf} F_{yf\_front} \cos(\theta_d - \theta_t) - l_{tr} F_{y\_rear}$

ระบบสมการนี้จะถูกจัดรูปในเมทริกซ์ 8x8 $\mathbf{A} \mathbf{x} = \mathbf{b}$ ซึ่งมีความแม่นยำสูงและไม่มีการแยกคำนวณมวล Dolly ซ้ำซ้อน


### 1.4 การจัดรูประบบสมการเมทริกซ์ 8x8 (8x8 Matrix Formulation)
เพื่อแก้สมการระบบ 8 ตัวแปรพร้อมกัน เราจัดรูปให้อยู่ในรูปแบบ $\mathbf{A} \mathbf{x} = \mathbf{b}$

เวกเตอร์ตัวแปรไม่ทราบค่า (Unknowns):
$\mathbf{x} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t, \dot{r}_d, F_d]^T$

**เมทริกซ์ $\mathbf{A}$ (ขนาด 8x8):**
$$
\mathbf{A} = \begin{bmatrix}
m_0 & 0 & 0 & 0 & 0 & 0 & 0 & -\cos(\theta_d - \theta_0) \\
0 & m_0 & 0 & 0 & 0 & 0 & 0 & -\sin(\theta_d - \theta_0) \\
0 & 0 & I_0 & 0 & 0 & 0 & 0 & d_h \sin(\theta_d - \theta_0) \\
0 & 0 & 0 & m_1 & 0 & 0 & 0 & \cos(\theta_d - \theta_t) \\
0 & 0 & 0 & 0 & m_1 & 0 & 0 & \sin(\theta_d - \theta_t) \\
0 & 0 & 0 & 0 & 0 & I_1 & 0 & l_{tf} \sin(\theta_d - \theta_t) \\
c_1 & -s_1 & d_h s_1 & -c_2 & s_2 & l_{tf} s_2 & L_{bar} r_d & 0 \\
s_1 & c_1 & -d_h c_1 & -s_2 & -c_2 & -l_{tf} c_2 & -L_{bar} & 0
\end{bmatrix}
$$
*(โดยที่ $c_1 = \cos(\theta_0 - \theta_d), s_1 = \sin(\theta_0 - \theta_d)$ และ $c_2 = \cos(\theta_t - \theta_d), s_2 = \sin(\theta_t - \theta_d)$)*

**เวกเตอร์ $\mathbf{b}$ (ขนาด 8x1):**
$$
\mathbf{b} = \begin{bmatrix}
F_{xr} + m_0 v_y r - F_{drag\_x} \\
F_{yf} + F_{yr} - m_0 v_x r \\
l_f F_{yf} - l_r F_{yr} \\
F_{yf\_front} \sin(\theta_d - \theta_t) + m_1 v_{yt} r_t - F_{drag\_t} \\
-F_{yf\_front} \cos(\theta_d - \theta_t) + F_{y\_rear} - m_1 v_{xt} r_t \\
-l_{tf} F_{yf\_front} \cos(\theta_d - \theta_t) - l_{tr} F_{y\_rear} \\
v_x s_1 r + (v_y - d_h r) c_1 r - v_{xt} s_2 r_t - (v_{yt} + l_{tf} r_t) c_2 r_t \\
-v_x c_1 r + (v_y - d_h r) s_1 r + v_{xt} c_2 r_t - (v_{yt} + l_{tf} r_t) s_2 r_t
\end{bmatrix}
$$

**ข้อดีของโมเดลใหม่เมื่อเทียบกับโมเดล 3-Body ตัวเก่า:**
1. **เสถียรภาพทางตัวเลขสูงขึ้นมาก:** ไม่มีมวลก้านลาก (Dolly mass) เล็กๆ ที่ก่อให้เกิดแรงเหวี่ยงมหาศาลและอาการ Jackknife
2. **ความสมจริงทางฟิสิกส์:** ก้านลาก (Drawbar) ทำหน้าที่เป็นเพียงตัวส่งผ่านแรงตึง (Tension) และกำหนดมุมเลี้ยวล้อหน้า (Kinematic Steering Constraint) ซึ่งตรงกับหลักการทำงานของ Full Trailer ในโลกความเป็นจริงอย่างสมบูรณ์
3. **ลดความซับซ้อน:** สมการลดลงจากเมทริกซ์ 13x13 (หรือ 9x9) เหลือเพียง 8x8 ทำให้โปรแกรมคำนวณได้เร็วและมีโอกาสเกิด Error ต่ำลงมาก
