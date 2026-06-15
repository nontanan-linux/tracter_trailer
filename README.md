# แบบจำลองพลศาสตร์ของยานยนต์ลากจูง (Dynamic Model of a Towing Vehicle)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วงแบบดอลลี่และตัวพ่วง (Tractor + Drawbar Dolly + Trailer Body) โดยเน้นไปที่การประยุกต์ใช้ **วิธีลากรานเจียน (Lagrangian Dynamics)** ในการจัดรูปสมการการเคลื่อนที่

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
![แผนภาพพิกัดและขนาดของระบบรถลากจูงพร้อมระบบพ่วง 1 ตัว (Tractor + Drawbar Trailer)](kinematic_diagram_1trailer.png)

แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 2 ชิ้นหลัก เชื่อมต่อกันด้วยจุดพ่วงแบบหมุนได้ (Revolute Joints / Hitch Joints):
1. **รถลากจูง (Tractor)**: มีมวล $m$ มีจุดศูนย์กลางมวล (CG) อยู่ที่พิกัด $(x_0, y_0)$ ในพิกัดโลก และมีทิศทางมุมหัวรถ (Yaw Angle) เท่ากับ $\theta_0$
2. **ดอลลี่ (Drawbar Dolly)**: มีมวล $m_d$ เชื่อมต่อกับท้ายรถลากจูงที่จุดพ่วง $H_1$ และมีมุมหัวดอลลี่เท่ากับ $\theta_1$
3. **ส่วนพ่วงหลัก (Trailer Body)**: มีมวล $m_t$ เชื่อมต่อกับดอลลี่ที่จุดพ่วงตัวที่สอง $H_2$ (Fifth Wheel Joint) และมีมุมหัวรถพ่วงเท่ากับ $\theta_2$

#### การอธิบายพารามิเตอร์จากแผนภาพ (Parameters Explanation)
| สัญลักษณ์ (Symbol) | ประเภท (Category) | คำอธิบายภาษาไทย (Thai Description) | คำอธิบายภาษาอังกฤษ (English Description) |
| :---: | :--- | :--- | :--- |
| **$(x_0, y_0)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถลากจูง (จุดอ้างอิงหลัก) | Tractor Rear Axle Center |
| **$(x_1, y_1)$** | พิกัด / ตำแหน่ง | จุดศูนย์กลางเพลาของดอลลี่ | Dolly Axle Center |
| **$(x_2, y_2)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถพ่วงหลัก | Trailer Rear Axle Center |
| **$H_1 (x_{h1}, y_{h1})$** | จุดต่อ / ข้อต่อ | จุดพ่วงแรก ระหว่างรถลากจูงและคานลากดอลลี่ | Hitch 1 (Tractor to Dolly) |
| **$H_2 (x_{h2}, y_{h2})$** | จุดต่อ / ข้อต่อ | จุดพ่วงที่สอง ระหว่างดอลลี่และรถพ่วงหลัก | Hitch 2 (Dolly to Trailer) |
| **$L_0$** | เรขาคณิต / ขนาด | ระยะฐานล้อของรถลากจูง | Tractor Wheelbase |
| **$L_1$** | เรขาคณิต / ขนาด | ความยาวของคานลากจูงดอลลี่ จาก $H_1$ ถึงเพลา | Drawbar Length |
| **$L_2$** | เรขาคณิต / ขนาด | ระยะฐานล้อของตัวพ่วงหลัก จาก $H_2$ ถึงเพลา | Trailer Wheelbase |
| **$d_h$** | เรขาคณิต / ขนาด | ระยะยื่นจากเพลาหลังรถลากจูงถึงจุดพ่วง $H_1$ | Tractor Rear Overhang |
| **$\theta_0$** | มุม / ทิศทาง | มุมหัวรถลากจูง เทียบกับแกนระดับโลก $X$ | Tractor Yaw Angle |
| **$\theta_1$** | มุม / ทิศทาง | มุมคานลากจูงดอลลี่ เทียบกับแกนระดับโลก $X$ | Dolly Yaw Angle |
| **$\theta_2$** | มุม / ทิศทาง | มุมหัวรถพ่วงหลัก เทียบกับแกนระดับโลก $X$ | Trailer Yaw Angle |
| **$\delta$** | มุม / ทิศทาง | มุมเลี้ยวของล้อหน้าเทียบกับแกนตามยาวของรถ | Front Wheel Steer Angle |

### 0.2 ระบบพิกัดตามมาตรฐาน ISO 8855-2011 (Vehicle Axis System ISO 8855-2011)
![Vehicle Axis System ISO 8855-2011](iso_8855_coordinate_system.png)

ระบบพิกัดที่ใช้อ้างอิงตามมาตรฐานสากล **ISO 8855-2011** สำหรับพลศาสตร์ยานยนต์ เป็นระบบพิกัดมือขวา (Right-Handed Coordinate System):
*   **พิกัดโลก (Global Inertial Frame: $OXY$)**: พิกัดอ้างอิงเฉื่อยบนพื้นระนาบโลกสำหรับการคำนวณตำแหน่งสัมบูรณ์
*   **พิกัดอ้างอิงตัวรถ (Body-Fixed Frame: $Cxyz$)**: พิกัดที่ยึดติดอยู่กับวัตถุแข็งเกร็งแต่ละชิ้น
    *   **แกน $x$ (Longitudinal Axis)**: ชี้ไปทางด้านหน้าของตัวรถ
    *   **แกน $y$ (Lateral Axis)**: ชี้ไปทางด้านซ้ายของตัวรถ
    *   **แกน $z$ (Vertical Axis)**: ชี้ขึ้นด้านบนในแนวตั้งฉากกับพื้นโลก
*   **มุมและการหมุน**:
    *   **มุมหัวรถ (Yaw Angle: $\theta$ / $\psi$)**: หมุนรอบแกน $z$ (Yaw Rate คือ $r = \dot{\theta}$) มีทิศทางเป็นบวกเมื่อหมุนทวนเข็มนาฬิกา

---

## 1. Origin and Principles of the Method

การวิเคราะห์พลศาสตร์ของระบบหลายชิ้นส่วน (Multi-Body Dynamics) ที่มีการเชื่อมต่อกันด้วยจุดพ่วง สามารถทำได้โดยใช้วิธีพลังงานเพื่อลดความซับซ้อนของการคำนวณแรงปฏิกิริยาภายใน

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบหลายชิ้นส่วน (Multi-Body Systems)

วิธีลากรานเจียนอาศัยการวิเคราะห์พลังงานรวมของระบบ ซึ่งมีความสะดวกและเป็นระบบสำหรับการวิเคราะห์โครงสร้างแบบลูกโซ่ (Kinematic Chains)

#### การออกแบบพิกัดทั่วไป (Generalized Coordinates Design)
เพื่อให้สามารถอธิบายสถานะการเคลื่อนที่ของระบบได้อย่างสมบูรณ์ในระดับพื้นฐานที่สุดก่อนพิจารณาแรงกระทำหรือข้อต่อเชื่อม เราจำเป็นต้องกำหนดพิกัดทั่วไป (Generalized Coordinates) เนื่องจากระบบประกอบด้วยวัตถุแข็งเกร็ง (Rigid Bodies) 3 ชิ้น (รถลากจูง, ดอลลี่, รถพ่วงหลัก) โดยวัตถุแต่ละชิ้นเคลื่อนที่อิสระบนระนาบ 2 มิติ (Planar Motion) จะมีระดับความอิสระ (Degrees of Freedom) 3 ระดับ ได้แก่ ตำแหน่งตามแนวแกน X, แนวแกน Y, และการหมุน (Yaw) 

ดังนั้น ระบบตั้งต้นนี้มีพิกัดทั่วไปทั้งหมด 9 ตัวแปร:
$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T \in \mathbb{R}^9$$

#### สมการออยเลอร์-ลากรานจ์ (Euler-Lagrange Equation)
สมการพื้นฐานสอดคล้องกับการอนุรักษ์พลังงานในระบบที่มีเพียงพลังงานจลน์ $T$ และพลังงานศักย์ $V$:
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = 0$$
โดยที่ $L = T - V$

สำหรับระบบของยานพาหนะที่มีแรงสัมผัสยางภายนอกที่ไม่ใช่อนุรักษ์พลังงาน (Non-conservative Forces) $Q_j$ และมีแรงปฏิกิริยาที่จุดพ่วง (Hitch Forces) กระทำอยู่ สมการจะขยายรูปแบบเพื่อรวมแรงภายนอกเหล่านี้เข้าไป:
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + F_{h,j}$$
โดยที่ $F_{h,j}$ คือแรงปฏิกิริยาพ่วงรวมที่กระทำต่อพิกัด $j$ ซึ่งเป็นตัวแทนเชิงคณิตศาสตร์ของแรงดึงพ่วง ($\lambda$) ที่ตำแหน่งข้อต่อต่างๆ

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
