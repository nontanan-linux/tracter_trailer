# แบบจำลองพลศาสตร์ของยานยนต์ลากจูง (Dynamic Model of a Towing Vehicle)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วง (Tractor + Full Trailer) โดยเน้นไปที่การประยุกต์ใช้ **วิธีลากรานเจียน (Lagrangian Dynamics)** ในการจัดรูปสมการการเคลื่อนที่

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 2 ชิ้นหลัก เชื่อมต่อกันด้วยก้านลาก (Drawbar) และจุดพ่วงแบบหมุนได้ (Revolute Joints):
1. **รถลากจูง (Tractor)**: มีมวล $m$ มีจุดศูนย์กลางมวล (CG) อยู่ที่พิกัด $(x_0, y_0)$ ในพิกัดโลก และมีทิศทางมุมหัวรถ (Yaw Angle) เท่ากับ $\theta_0$
2. **รถพ่วงหลัก (Trailer Body)**: มีมวล $m_t$ มองเป็นรถ 1 คันที่ล้อหน้าสามารถหมุนเลี้ยวได้ตามมุมของก้านลาก (Drawbar) ก้านลากจะเชื่อมจากจุดพ่วงท้ายรถลากจูง $H_1$ ไปยังเพลาหน้าของรถพ่วง $P_f$

#### การอธิบายพารามิเตอร์จากแผนภาพ (Parameters Explanation)
| สัญลักษณ์ (Symbol) | ประเภท (Category) | คำอธิบายภาษาไทย (Thai Description) | คำอธิบายภาษาอังกฤษ (English Description) |
| :---: | :--- | :--- | :--- |
| **$(x_0, y_0)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถลากจูง (จุดอ้างอิงหลัก) | Tractor Rear Axle Center |
| **$(x_t, y_t)$** | พิกัด / ตำแหน่ง | จุดศูนย์กลางมวลของรถพ่วงหลัก | Trailer Center of Gravity |
| **$H_1 (x_{h1}, y_{h1})$** | จุดต่อ / ข้อต่อ | จุดพ่วง ระหว่างรถลากจูงและคานลาก | Hitch (Tractor to Drawbar) |
| **$P_f$** | จุดต่อ / ข้อต่อ | กึ่งกลางเพลาหน้าของรถพ่วง (จุดหมุนเลี้ยว) | Trailer Front Steerable Axle |
| **$l_f, l_r$** | เรขาคณิต / ขนาด | ระยะจาก CG ถึงเพลาหน้าและหลังของรถลากจูง | Tractor Front/Rear Lengths |
| **$l_{tf}, l_{tr}$** | เรขาคณิต / ขนาด | ระยะจาก CG ถึงเพลาหน้าและหลังของรถพ่วง | Trailer Front/Rear Lengths |
| **$L_{bar}$** | เรขาคณิต / ขนาด | ความยาวของคานลากจูง จาก $H_1$ ถึง $P_f$ | Drawbar Length |
| **$d_h$** | เรขาคณิต / ขนาด | ระยะยื่นจากเพลาหลังรถลากจูงถึงจุดพ่วง $H_1$ | Tractor Rear Overhang |
| **$\theta_0$** | มุม / ทิศทาง | มุมหัวรถลากจูง เทียบกับแกนระดับโลก $X$ | Tractor Yaw Angle |
| **$\theta_t$** | มุม / ทิศทาง | มุมหัวรถพ่วงหลัก เทียบกับแกนระดับโลก $X$ | Trailer Yaw Angle |
| **$\theta_d$** | มุม / ทิศทาง | มุมคานลากจูง เทียบกับแกนระดับโลก $X$ | Drawbar Angle |
| **$\delta$** | มุม / ทิศทาง | มุมเลี้ยวของล้อหน้ารถลากจูง | Tractor Front Wheel Steer Angle |
| **$\delta_t$** | มุม / ทิศทาง | มุมเลี้ยวล้อหน้ารถพ่วง ($\theta_d - \theta_t$) | Trailer Front Wheel Steer Angle |

---

## 1. Origin and Principles of the Method

การวิเคราะห์พลศาสตร์ของระบบทำได้โดยใช้วิธีพลังงานเพื่อลดความซับซ้อนของการคำนวณแรงปฏิกิริยาภายใน

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบ 2 ชิ้นส่วน

วิธีลากรานเจียนอาศัยการวิเคราะห์พลังงานรวมของระบบ ซึ่งจะพิจารณารถพ่วง (Trailer) เป็นวัตถุเกร็งชิ้นเดียว (1 Body) โดยให้ก้านลาก (Drawbar) ทำหน้าที่เป็นเพียงก้านบังคับเลี้ยว (Steering Arm) ที่ส่งผลต่อมุมเลี้ยวล้อหน้า $\delta_t$ เท่านั้น ทำให้ไม่มีมวล Dolly แยกออกมา

#### การออกแบบพิกัดทั่วไป (Generalized Coordinates Design)
ระบบตั้งต้นนี้มีพิกัดทั่วไปทั้งหมด 6 ตัวแปร (พิกัดรถลากจูง 3 ตัว และพิกัดรถพ่วง 3 ตัว):
$$q = [x_0, y_0, \theta_0, x_t, y_t, \theta_t]^T \in \mathbb{R}^6$$

ส่วนความเร็วเชิงมุมของก้านลาก ($\dot{\theta}_d = r_d$) จะใช้ระบุเงื่อนไขข้อจำกัดความเร็วของจุดพ่วง

#### สมการออยเลอร์-ลากรานจ์ (Euler-Lagrange Equation)
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + F_{h,j}$$

### 1.3 ที่มาของความเร็วเชิงเส้นในพิกัดโลก (Derivation of Linear Velocities)

**1. รถลากจูง (Tractor):**
$$\dot{x}_0 = v_x \cos\theta_0 - v_y \sin\theta_0$$
$$\dot{y}_0 = v_x \sin\theta_0 + v_y \cos\theta_0$$
$$\dot{\theta}_0 = r$$

**2. รถพ่วงหลัก (Trailer Body):**
$$\dot{x}_t = v_{xt} \cos\theta_t - v_{yt} \sin\theta_t$$
$$\dot{y}_t = v_{xt} \sin\theta_t + v_{yt} \cos\theta_t$$
$$\dot{\theta}_t = r_t$$

### 1.5 Kinetic and Potential Energy (พลังงานจลน์และพลังงานศักย์)

**พลังงานจลน์ (Kinetic Energy: $T$)**
พลังงานจลน์รวมพิจารณาเฉพาะมวล 2 ก้อนหลัก:
$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_t^2 \right]
$$

**พลังงานศักย์ (Potential Energy: $V$)**
สำหรับการเคลื่อนที่บนระนาบระดับ พลังงานศักย์ของระบบมีค่าคงที่และอ้างอิงเป็นศูนย์:
$$V = 0 \implies L = T$$

---

## 2. การอนุพันธ์สมการการเคลื่อนที่ด้วยพิกัดทั่วไป (Derivation of Equations of Motion)

### 2.1 การหาอนุพันธ์ของสมการลากรานจ์ $L$
จากสมการลากรานจ์ $L = T$ หาอนุพันธ์เทียบกับตัวแปรอัตราเร็วของพิกัดทั่วไปจะได้มวลความเร่งตามปกติ:

**1. รถลากจูง (Tractor):**
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_0}\right) = m \ddot{x}_0$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_0}\right) = m \ddot{y}_0$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_0}\right) = I_z \ddot{\theta}_0$

**2. รถพ่วงหลัก (Trailer Body):**
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_t}\right) = m_t \ddot{x}_t$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_t}\right) = m_t \ddot{y}_t$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_t}\right) = I_{zt} \ddot{\theta}_t$

### 2.2 การหาแรงทั่วไป (Generalized Forces $Q_i$)
กำหนดให้แรงที่ส่งผ่านจุดพ่วง $H_1$ และดึงเพลาหน้ารถพ่วง $P_f$ คือ $F_{hx}$ และ $F_{hy}$ (อ้างอิงในพิกัดรถลากจูง)
ล้อหน้ารถพ่วงมีแรงสัมผัสหน้ายางด้านข้างคือ $F_{ytf}$ โดยล้อหน้าถูกบังคับเลี้ยวตามมุม $\theta_d$ ดังนั้นมุมเลี้ยวเทียบกับตัวถังคือ $\delta_t = \theta_d - \theta_t$

แรงภายนอกด้านข้างของเพลาหน้ารถพ่วงเมื่อแตกแรงเข้าสู่แกน $x, y$ ของตัวถังรถพ่วงจะได้:
*   แกน $x_t$: $-F_{ytf} \sin(\theta_d - \theta_t)$
*   แกน $y_t$: $F_{ytf} \cos(\theta_d - \theta_t)$

### 2.3 และ 2.4 สมการการเคลื่อนที่ในพิกัดตัวรถ (Body-Fixed Equations of Motion)
แปลงความเร่งเข้าสู่พิกัดตัวรถด้วยความสัมพันธ์ $(\dot{v}_x - v_y r)$ และจัดรูปจะได้ 6 สมการดังนี้:

#### 1. รถลากจูง (Tractor)
$$\text{Longitudinal:} \quad m(\dot{v}_x - v_y r) = F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx}$$
$$\text{Lateral:} \quad m(\dot{v}_y + v_x r) = F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy}$$
$$\text{Yaw:} \quad I_z \dot{r} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy}$$

#### 2. รถพ่วงหลัก (Trailer Body)
กำหนดความแตกต่างของมุมระหว่างรถลากและรถพ่วงคือ $\Delta\theta_t = \theta_0 - \theta_t$:
$$\text{Longitudinal:} \quad m_t(\dot{v}_{xt} - v_{yt} r_t) = -F_{ytf}\sin(\theta_d - \theta_t) + F_{hx}\cos\Delta\theta_t + F_{hy}\sin\Delta\theta_t$$
$$\text{Lateral:} \quad m_t(\dot{v}_{yt} + v_{xt} r_t) = F_{ytr} + F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t$$
$$\text{Yaw:} \quad I_{zt} \dot{r}_t = l_{tf} \left[ F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \right] - l_{tr} F_{ytr}$$

### 2.5 สมการเงื่อนไขบังคับเชิงความเร่ง (Hitch Acceleration Constraints)
เพื่อให้ระบบครบถ้วนสมบูรณ์ ต้องเพิ่มเงื่อนไขว่าความเร็วของจุดพ่วงที่ท้ายรถลากจูง ($H_1$) ต้องสัมพันธ์กับความเร็วของกึ่งกลางเพลาหน้ารถพ่วง ($P_f$) ผ่านคานลากจูง (Drawbar) ความยาว $L_{bar}$ เมื่อหาอนุพันธ์เทียบกับเวลาจะได้เงื่อนไขความเร่งที่ต้องบังคับ (คล้ายกับรุ่นก่อนหน้า แต่รวมจุด $H_1$ และ $H_2$ เข้าด้วยกันผ่านมุม $r_d$):

กำหนดให้ $\Delta\theta_1 = \theta_0 - \theta_d$ และ $\Delta\theta_2 = \theta_t - \theta_d$:
$$\dot{v}_{xt} \cos\Delta\theta_2 - \dot{v}_{yt} \sin\Delta\theta_2 - l_{tf} \dot{r}_t \sin\Delta\theta_2 - \dot{v}_x \cos\Delta\theta_1 + \dot{v}_y \sin\Delta\theta_1 - d_h \dot{r} \sin\Delta\theta_1 = \dots (Coriolis\_X)$$
$$\dot{v}_{xt} \sin\Delta\theta_2 + \dot{v}_{yt} \cos\Delta\theta_2 + l_{tf} \dot{r}_t \cos\Delta\theta_2 - \dot{v}_x \sin\Delta\theta_1 - \dot{v}_y \cos\Delta\theta_1 + d_h \dot{r} \cos\Delta\theta_1 + L_{bar} \dot{r}_d = \dots (Coriolis\_Y)$$

### 2.6 การจัดรูประบบสมการเมทริกซ์ (Matrix Formulation)
จากสมการการเคลื่อนที่ในพิกัดตัวรถ เราสามารถจัดรูปรวมสมการทั้ง 6 ให้เป็นสมการเชิงอนุพันธ์เมทริกซ์บรรทัดเดียว (Single-line Matrix Equation) ตามรูปแบบมาตรฐานลากรานเจียนได้ดังนี้:

$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q $$

โดยมีรายละเอียดของแต่ละเทอมดังต่อไปนี้:

#### 1. เวกเตอร์ความเร่งและความเร็ว ($\ddot{q}$ และ $\dot{q}$)
เวกเตอร์ตัวแปรสถานะขนาด $6 \times 1$:
*   $\ddot{q} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t]^T$
*   $\dot{q} = [v_x, v_y, r, v_{xt}, v_{yt}, r_t]^T$

#### 2. เมทริกซ์มวลและความเฉื่อย $M(q)$
เมทริกซ์ทแยงมุม (Diagonal Matrix) ขนาด $6 \times 6$:
$$ M(q) = \text{diag}(m, m, I_z, m_t, m_t, I_{zt}) $$

#### 3. เมทริกซ์คอริโอลิสและแรงหนีศูนย์กลาง $C(q, \dot{q})$
เมทริกซ์ขนาด $6 \times 6$ ที่บรรจุเทอมความเร็วหนีศูนย์กลาง:
$$
C(q, \dot{q}) = \begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

#### 4. เวกเตอร์แรงทั่วไป $Q$
เวกเตอร์ขนาด $6 \times 1$ ซึ่งรวมเอาแรงภายนอกหน้ายางและแรงปฏิกิริยาพ่วง:
$$
Q = \begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy} \\
-F_{ytf}\sin(\theta_d - \theta_t) + F_{hx}\cos\Delta\theta_t + F_{hy}\sin\Delta\theta_t \\
F_{ytr} + F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \\
l_{tf} \left[ F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \right] - l_{tr} F_{ytr}
\end{bmatrix}
$$

#### 5. สมการเต็มรูปแบบ (Full Expanded Equation)
เมื่อนำเมทริกซ์ทั้งหมดมาประกอบกันจะได้สมการในรูปแบบเต็ม 1 บรรทัดดังนี้:

$$
\begin{bmatrix}
m & 0 & 0 & 0 & 0 & 0 \\
0 & m & 0 & 0 & 0 & 0 \\
0 & 0 & I_z & 0 & 0 & 0 \\
0 & 0 & 0 & m_t & 0 & 0 \\
0 & 0 & 0 & 0 & m_t & 0 \\
0 & 0 & 0 & 0 & 0 & I_{zt}
\end{bmatrix}
\begin{bmatrix}
\dot{v}_x \\ \dot{v}_y \\ \dot{r} \\ \dot{v}_{xt} \\ \dot{v}_{yt} \\ \dot{r}_t
\end{bmatrix}
+
\begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
v_x \\ v_y \\ r \\ v_{xt} \\ v_{yt} \\ r_t
\end{bmatrix}
=
\begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy} \\
-F_{ytf}\sin(\theta_d - \theta_t) + F_{hx}\cos\Delta\theta_t + F_{hy}\sin\Delta\theta_t \\
F_{ytr} + F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \\
l_{tf} \left[ F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \right] - l_{tr} F_{ytr}
\end{bmatrix}
$$

การนำเสนอในรูปสมการ $M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q$ แบบลดรูปเหลือ 2-Body นี้ ช่วยขจัดปัญหาความไม่เสถียร (Jackknife) จากการแยกมวล Dolly ซ้ำซ้อน และยังคงรักษารูปแบบดั้งเดิมที่เหมาะสำหรับการนำแบบจำลองไปใช้ออกแบบระบบควบคุมเชิงพลศาสตร์ (Dynamical Controller) เช่น LQR หรือ MPC ต่อไปได้อย่างสมบูรณ์แบบ
