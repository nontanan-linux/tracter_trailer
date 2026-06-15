# แบบจำลองพลศาสตร์ 2-Body (Tractor + Full Trailer)
### 1.3 แบบจำลองคณิตศาสตร์ 2 วัตถุ (2-Body Mathematical Model)
เพื่อให้สอดคล้องกับพฤติกรรมทางกายภาพของรถพ่วงแบบก้านลาก (Drawbar Trailer หรือ Full Trailer) เราสามารถมองรถพ่วงเป็น **"รถ 1 คันที่มีล้อหน้าหมุนเลี้ยวได้"** โดยที่:
- **Drawbar (ก้านลาก)** ทำหน้าที่เป็นเพียงแขนคาน (Arm) หรือโครงสร้างที่ยึดติดกับเพลาล้อหน้า ไม่มีมวลในตัวมันเอง
- การเคลื่อนที่และมุมของ Drawbar ($	heta_d$) จะเป็นตัวกำหนดมุมเลี้ยว (Steering Angle) ของล้อหน้ารถพ่วง
- มวลทั้งหมดจะถูกรวมไว้ที่ตัวถังรถพ่วง (Trailer Body)

#### สมการความเร็วเชิงจลนศาสตร์ (Kinematic Velocity Equations)
ให้ $H_1$ เป็นจุดพ่วงท้ายรถลากจูง และ $P_f$ เป็นเพลาหน้ารถพ่วง ความเร็วของจุดทั้งสองต้องสอดคล้องกันผ่านก้านลากที่มีความยาว $L_{bar}$:

ความเร็วของจุดพ่วง $H_1$ ในกรอบอ้างอิงของรถลากจูง:
$v_{H1} = [v_x, v_y - d_h r]^T$

ความเร็วของเพลาหน้ารถพ่วง $P_f$ ในกรอบอ้างอิงรถพ่วง:
$v_{Pf} = [v_{xt}, v_{yt} + l_{tf} r_t]^T$

สมการข้อจำกัดทางความเร็วจากก้านลาก (Velocity Constraints from Drawbar):
$C_1: v_x \cos\Delta	heta_1 - (v_y - d_h r) \sin\Delta	heta_1 - v_{xt} \cos\Delta	heta_2 + (v_{yt} + l_{tf} r_t) \sin\Delta	heta_2 = 0$
$C_2: v_x \sin\Delta	heta_1 + (v_y - d_h r) \cos\Delta	heta_1 - v_{xt} \sin\Delta	heta_2 - (v_{yt} + l_{tf} r_t) \cos\Delta	heta_2 - L_{bar} r_d = 0$
เมื่อ $\Delta	heta_1 = 	heta_0 - 	heta_d$ และ $\Delta	heta_2 = 	heta_t - 	heta_d$

#### สมดุลแรงแบบนิวตัน-ออยเลอร์ (Newton-Euler Equations of Motion)
ระบบประกอบด้วย 8 ตัวแปรไม่ทราบค่า: $[\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t, \dot{r}_d, F_d]^T$
โดยที่ $F_d$ คือแรงดึง/อัด (Tension/Compression) ตามแนวแกนของ Drawbar

**สมการรถลากจูง (Tractor):**
1. $m_0 \dot{v}_x - F_d \cos(	heta_d - 	heta_0) = F_{xr} + m_0 v_y r - F_{drag\_x}$
2. $m_0 \dot{v}_y - F_d \sin(	heta_d - 	heta_0) = F_{yf} + F_{yr} - m_0 v_x r$
3. $I_0 \dot{r} + d_h F_d \sin(	heta_d - 	heta_0) = l_f F_{yf} - l_r F_{yr}$

**สมการรถพ่วง (Full Trailer):**
4. $m_1 \dot{v}_{xt} + F_d \cos(	heta_d - 	heta_t) = F_{yf\_front} \sin(	heta_d - 	heta_t) + m_1 v_{yt} r_t - F_{drag\_t}$
5. $m_1 \dot{v}_{yt} + F_d \sin(	heta_d - 	heta_t) = -F_{yf\_front} \cos(	heta_d - 	heta_t) + F_{y\_rear} - m_1 v_{xt} r_t$
6. $I_1 \dot{r}_t + l_{tf} \sin(	heta_d - 	heta_t) F_d = -l_{tf} F_{yf\_front} \cos(	heta_d - 	heta_t) - l_{tr} F_{y\_rear}$

ระบบสมการนี้จะถูกจัดรูปในเมทริกซ์ 8x8 $\mathbf{A} \mathbf{x} = \mathbf{b}$ ซึ่งมีความแม่นยำสูงและไม่มีการแยกคำนวณมวล Dolly ซ้ำซ้อน
