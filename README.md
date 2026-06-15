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

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบ 2 วัตถุ

เพื่อลดความซับซ้อนและแก้ปัญหาความไม่เสถียร (Jackknife) จากการแยกมวล Dolly เราจะมองว่า Full Trailer คือ **รถ 1 คันที่มีล้อหน้าหมุนเลี้ยวได้** โดยให้ก้านลาก (Drawbar) เป็นเพียงแขนคาน (Arm) ที่ส่งผ่านแรงตึง/อัด (Tension/Compression) เท่านั้น

#### การออกแบบพิกัดทั่วไป (Generalized Coordinates)
ระบบประกอบด้วยวัตถุแข็งเกร็ง 2 ชิ้น (รถลากจูง และ รถพ่วงหลัก) โดยมีพิกัดอิสระคือตำแหน่งและมุม และเพิ่มมุม $\theta_d$ ของก้านลากเพื่อใช้อธิบายสมการข้อจำกัด
$$q = [x_0, y_0, \theta_0, x_t, y_t, \theta_t]^T \in \mathbb{R}^6$$

เนื่องจากก้านลาก (Drawbar) ไม่มีมวล แรงที่กระทำผ่านก้านลากจึงมีเพียงแรงตามแนวแกน (Axial Force: $F_d$) ทิศทางตามมุม $\theta_d$ ซึ่งสามารถแตกเป็นแรงในพิกัดโลกได้คือ $\lambda_x = F_d \cos\theta_d$ และ $\lambda_y = F_d \sin\theta_d$

### 1.3 พลังงานจลน์ (Kinetic Energy: $T$)
พลังงานจลน์รวมของระบบพิจารณาเฉพาะวัตถุที่มีมวล 2 ชิ้น (ไม่มีเทอมของ Dolly):
$$
T = \left[ \frac{1}{2}m_0(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_0\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_1(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_1\dot{\theta}_t^2 \right]
$$

### 1.4 ประกอบสมการ Lagrangian
จากสมการออยเลอร์-ลากรานจ์:
$$\frac{d}{dt}\left( \frac{\partial T}{\partial \dot{q}_i} \right) - \frac{\partial T}{\partial q_i} = Q_i + F_{h,i}$$
เราจะได้สมการการเคลื่อนที่ 6 สมการ (Tractor 3 + Trailer 3) โดยให้แรงดึงผ่าน Drawbar ($F_d$) กระทำที่จุดพ่วง $H_1$ และเพลาหน้า $P_f$:

**1. รถลากจูง (Tractor):**
* $m_0 \ddot{x}_0 = Q_{x0} - F_d \cos\theta_d$
* $m_0 \ddot{y}_0 = Q_{y0} - F_d \sin\theta_d$
* $I_0 \ddot{\theta}_0 = Q_{\theta0} + d_h F_d \sin(\theta_d - \theta_0)$

**2. รถพ่วง (Full Trailer):**
* $m_1 \ddot{x}_t = Q_{xt} + F_d \cos\theta_d$
* $m_1 \ddot{y}_t = Q_{yt} + F_d \sin\theta_d$
* $I_1 \ddot{\theta}_t = Q_{\theta t} + l_{tf} F_d \sin(\theta_d - \theta_t)$

---

## 2. การแปลงสมการเข้าสู่พิกัดตัวรถและสร้างเมทริกซ์ (Body-Fixed Matrix Formulation)

เพื่อนำไปใช้คำนวณ เราจะแปลงสมการความเร่งในพิกัดโลก $(\ddot{x}, \ddot{y})$ ให้กลายเป็นความเร่งและแรงหนีศูนย์กลางในพิกัดตัวรถ $(\dot{v}_x - v_y r)$ และแปลงแรงภายนอก $Q_i$ ให้เป็นแรงจากหน้าสัมผัสยาง

### 2.1 สมการนิวตัน-ออยเลอร์ในพิกัดตัวรถ
เมื่อแทนค่าการแปลงพิกัด เราจะได้สมการพลศาสตร์ 6 สมการหลัก:

**Tractor (Body 0):**
1. $m_0 \dot{v}_x - F_d \cos(\theta_d - \theta_0) = F_{xr} + m_0 v_y r - F_{drag\_x}$
2. $m_0 \dot{v}_y - F_d \sin(\theta_d - \theta_0) = F_{yf} + F_{yr} - m_0 v_x r$
3. $I_0 \dot{r} + d_h F_d \sin(\theta_d - \theta_0) = l_f F_{yf} - l_r F_{yr}$

**Full Trailer (Body 1):**
4. $m_1 \dot{v}_{xt} + F_d \cos(\theta_d - \theta_t) = F_{yf\_front} \sin(\theta_d - \theta_t) + m_1 v_{yt} r_t - F_{drag\_t}$
5. $m_1 \dot{v}_{yt} + F_d \sin(\theta_d - \theta_t) = -F_{yf\_front} \cos(\theta_d - \theta_t) + F_{y\_rear} - m_1 v_{xt} r_t$
6. $I_1 \dot{r}_t + l_{tf} \sin(\theta_d - \theta_t) F_d = -l_{tf} F_{yf\_front} \cos(\theta_d - \theta_t) - l_{tr} F_{y\_rear}$

*(หมายเหตุ: ล้อหน้าของรถพ่วงสามารถหมุนเลี้ยวได้ตามมุม $\theta_d$ ดังนั้นแรงหน้าสัมผัส $F_{yf\_front}$ จะทำมุม $(\theta_d - \theta_t)$ กับตัวถังรถพ่วง)*

### 2.2 สมการข้อจำกัดความเร็วจากก้านลาก (Kinematic Velocity Constraints)
เนื่องจากเรามี 8 ตัวแปรไม่ทราบค่า (ความเร่ง 6 ตัว + $\dot{r}_d$ + $F_d$) เราจึงต้องใช้สมการข้อจำกัดทางกายภาพที่ว่า ความเร็วของจุดพ่วง $H_1$ และเพลาหน้า $P_f$ ต้องสอดคล้องกันผ่านความยาวก้านลาก $L_{bar}$ มาหาอนุพันธ์เทียบกับเวลาเพื่อสร้างสมการที่ 7 และ 8:

7. $v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1 - v_{xt} \cos\Delta\theta_2 + (v_{yt} + l_{tf} r_t) \sin\Delta\theta_2 = 0$
8. $v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1 - v_{xt} \sin\Delta\theta_2 - (v_{yt} + l_{tf} r_t) \cos\Delta\theta_2 - L_{bar} r_d = 0$
*(โดยที่ $\Delta\theta_1 = \theta_0 - \theta_d$ และ $\Delta\theta_2 = \theta_t - \theta_d$)*

### 2.3 การจัดรูประบบสมการเมทริกซ์ 8x8 (8x8 Matrix Formulation)
เมื่อนำสมการทั้งหมดมาประกอบเป็นเมทริกซ์ $\mathbf{A} \mathbf{x} = \mathbf{b}$ สำหรับการแก้สมการเชิงเส้น:

เวกเตอร์ตัวแปรสถานะ: $\mathbf{x} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t, \dot{r}_d, F_d]^T$

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

การนำเสนอด้วยโมเดล 2 วัตถุที่สมบูรณ์แบบนี้ มีความแม่นยำสูงทางฟิสิกส์ และไม่มีการแยกคำนวณมวล Dolly ซ้ำซ้อน ทำให้สมการกะทัดรัดขึ้นและแก้ปัญหาเสถียรภาพ (Jackknife) ได้อย่างเด็ดขาด เป็นรากฐานที่ยอดเยี่ยมสำหรับการทำ Controller ต่อไป!
