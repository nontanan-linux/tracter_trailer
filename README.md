# แบบจำลองพลศาสตร์ของยานยนต์ลากจูง (Dynamic Model of a Towing Vehicle)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วงแบบ Full Trailer (Tractor + Trailer Body) โดยเน้นไปที่การประยุกต์ใช้ **วิธีลากรานเจียน (Lagrangian Dynamics)** ในการจัดรูปสมการการเคลื่อนที่

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 2 ชิ้นหลัก เชื่อมต่อกันผ่านก้านลาก (Drawbar) แบบไม่มีมวล:
1. **รถลากจูง (Tractor)**: มีมวล $m$ มีจุดศูนย์กลางมวล (CG) อยู่ที่พิกัด $(x_0, y_0)$ ในพิกัดโลก และมีทิศทางมุมหัวรถ (Yaw Angle) เท่ากับ $\theta_0$
2. **ส่วนพ่วงหลัก (Trailer Body)**: มีมวล $m_t$ เชื่อมต่อกับรถลากจูงผ่านก้านลาก (Drawbar) โดยที่ก้านลากทำหน้าที่เชื่อมจุดพ่วง $H_1$ เข้ากับ **เพลาหน้าของรถพ่วง** ($P_f$) ซึ่งเพลาหน้านี้สามารถหมุนเลี้ยวได้ตามมุมของก้านลาก

#### การอธิบายพารามิเตอร์ (Parameters Explanation)
| สัญลักษณ์ | คำอธิบายภาษาไทย | English Description |
| :---: | :--- | :--- |
| **$(x_0, y_0)$** | กึ่งกลางเพลาหลังของรถลากจูง (จุดอ้างอิงหลัก) | Tractor Rear Axle Center |
| **$(x_t, y_t)$** | จุดศูนย์กลางมวลของรถพ่วงหลัก | Trailer Center of Gravity |
| **$H_1$** | จุดพ่วง ระหว่างรถลากจูงและก้านลาก | Hitch (Tractor to Drawbar) |
| **$P_f$** | กึ่งกลางเพลาหน้าของรถพ่วง (จุดหมุนเลี้ยว) | Trailer Front Steerable Axle |
| **$l_f, l_r$** | ระยะจาก CG ถึงเพลาหน้า/หลัง ของรถลากจูง | Tractor Front/Rear lengths |
| **$l_{tf}, l_{tr}$** | ระยะจาก CG ถึงเพลาหน้า/หลัง ของรถพ่วง | Trailer Front/Rear lengths |
| **$L_{bar}$** | ความยาวของก้านลากจูง จาก $H_1$ ถึง $P_f$ | Drawbar Length |
| **$d_h$** | ระยะยื่นจากเพลาหลังรถลากจูงถึงจุดพ่วง $H_1$ | Tractor Rear Overhang |
| **$\theta_0$** | มุมหัวรถลากจูง เทียบกับแกนระดับโลก $X$ | Tractor Yaw Angle |
| **$\theta_t$** | มุมหัวรถพ่วงหลัก เทียบกับแกนระดับโลก $X$ | Trailer Yaw Angle |
| **$\theta_d$** | มุมของก้านลากจูง (กำหนดมุมเลี้ยวเพลาหน้า) | Drawbar Angle |

---

## 1. Origin and Principles of the Method

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบ 2 ชิ้นส่วน
เพื่อให้แบบจำลองถูกต้องตามกายภาพของ Full Trailer ที่มองว่า "รถพ่วงคือรถ 1 คันที่ล้อหน้าหมุนเลี้ยวได้" (ก้านลากเปรียบเสมือนแขนบังคับเลี้ยว) เราจึงใช้ระบบวัตถุเกร็ง 2 ชิ้น

#### การออกแบบพิกัดทั่วไป (Generalized Coordinates Design)
พิกัดทั่วไป (Generalized Coordinates) สำหรับวัตถุ 2 ชิ้นบนระนาบ 2 มิติ จะมี 6 ระดับความอิสระ (Degrees of Freedom):
$$q = [x_0, y_0, \theta_0, x_t, y_t, \theta_t]^T \in \mathbb{R}^6$$

โดยที่มุมของก้านลาก $\theta_d$ ไม่ใช่ตัวแปรอิสระ แต่ถูกกำหนดจากเรขาคณิตของจุดพ่วง:
$$\tan\theta_d = \frac{y_{pf} - y_{h1}}{x_{pf} - x_{h1}}$$
โดยมุมเลี้ยวของล้อหน้ารถพ่วงเทียบกับตัวถังคือ $\delta_t = \theta_d - \theta_t$

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
พลังงานจลน์รวมของระบบเกิดจากการเคลื่อนที่ของมวล 2 ชิ้น (Tractor และ Trailer):
$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_t^2 \right]
$$

**พลังงานศักย์ (Potential Energy: $V$)**
สำหรับการเคลื่อนที่บนระนาบ:
$$V = 0 \implies L = T$$

---

## 2. การอนุพันธ์สมการการเคลื่อนที่ (Derivation of Equations of Motion)

### 2.1 การหาอนุพันธ์ของสมการลากรานจ์ $L$

จากสมการ $L = T$ หาอนุพันธ์เทียบกับพิกัดทั่วไปทั้ง 6 ตัว:

**1. พิกัดของรถลากจูง (Tractor): $q_1 \dots q_3$**
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_0}\right) = m \ddot{x}_0$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_0}\right) = m \ddot{y}_0$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_0}\right) = I_z \ddot{\theta}_0$

**2. พิกัดของรถพ่วงหลัก (Trailer Body): $q_4 \dots q_6$**
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_t}\right) = m_t \ddot{x}_t$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_t}\right) = m_t \ddot{y}_t$
*   $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_t}\right) = I_{zt} \ddot{\theta}_t$

### 2.2 การหาแรงทั่วไป (Generalized Forces $Q_i$)
แรงทั่วไปรวมเอาแรงต้านหน้าสัมผัสยางล้อ (Tire Forces) และแรงจุดพ่วง (Hitch Forces)

กำหนดให้แรงตึงก้านลากที่ถ่ายทอดระหว่าง Tractor กับ Trailer มีแรงปฏิกิริยาในพิกัดตัวรถลากจูง (Tractor Body Frame) เท่ากับ $F_{hx}$ และ $F_{hy}$

**แรงหน้าสัมผัสของ Trailer:**
ล้อหน้าของรถพ่วงหมุนเลี้ยวตามมุม $\delta_t = \theta_d - \theta_t$ แรงด้านข้างที่ล้อหน้าคือ $F_{ytf}$
ดังนั้นแรงล้อหน้ารถพ่วงในแกนของตัวถังรถพ่วง (Trailer Frame) คือ:
* แนวแกน $x_t$: $-F_{ytf}\sin(\theta_d - \theta_t)$
* แนวแกน $y_t$: $F_{ytf}\cos(\theta_d - \theta_t)$

### 2.3 สมการการเคลื่อนที่ในพิกัดตัวรถ (Body-Fixed Equations of Motion)
เมื่อแปลงความเร่งจากพิกัดโลก $\ddot{x}, \ddot{y}$ เข้าสู่พิกัดตัวรถ $\dot{v}_x - v_y r, \dot{v}_y + v_x r$ จะได้สมการ 6 สมการ:

#### 1. รถลากจูง (Tractor)
$$\text{Longitudinal:} \quad m(\dot{v}_x - v_y r) = F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx}$$
$$\text{Lateral:} \quad m(\dot{v}_y + v_x r) = F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy}$$
$$\text{Yaw:} \quad I_z \dot{r} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy}$$

#### 2. รถพ่วงหลัก (Trailer Body)
แรงที่กระทำต่อรถพ่วงเกิดจากแรงลากผ่านก้านลาก (ซึ่งถูกแปลงจากพิกัดรถลากจูงมายังพิกัดรถพ่วง) และแรงล้อ:
กำหนดมุมสัมพัทธ์ $\Delta\theta_t = \theta_0 - \theta_t$
$$\text{Longitudinal:} \quad m_t(\dot{v}_{xt} - v_{yt} r_t) = -F_{ytf}\sin(\theta_d - \theta_t) + F_{hx}\cos\Delta\theta_t + F_{hy}\sin\Delta\theta_t$$
$$\text{Lateral:} \quad m_t(\dot{v}_{yt} + v_{xt} r_t) = F_{ytr} + F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t$$
$$\text{Yaw:} \quad I_{zt} \dot{r}_t = l_{tf} \left[ F_{ytf}\cos(\theta_d - \theta_t) - F_{hx}\sin\Delta\theta_t + F_{hy}\cos\Delta\theta_t \right] - l_{tr} F_{ytr}$$

### 2.6 การจัดรูประบบสมการเมทริกซ์ (Matrix Formulation)
นำสมการการเคลื่อนที่ทั้ง 6 สมการมาจัดรูปรวมเป็นสมการเชิงอนุพันธ์เมทริกซ์บรรทัดเดียว (Single-line Matrix Equation) ในรูปแบบลากรานเจียนมาตรฐาน:

$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q $$

#### 1. เวกเตอร์ความเร่งและความเร็ว
เวกเตอร์ตัวแปรสถานะขนาด $6 \times 1$:
*   $\ddot{q} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t]^T$
*   $\dot{q} = [v_x, v_y, r, v_{xt}, v_{yt}, r_t]^T$

#### 2. เมทริกซ์มวลและความเฉื่อย $M(q)$
เมทริกซ์ขนาด $6 \times 6$:
$$ M(q) = \text{diag}(m, m, I_z, m_t, m_t, I_{zt}) $$

#### 3. เมทริกซ์คอริโอลิส $C(q, \dot{q})$
เมทริกซ์ขนาด $6 \times 6$:
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
เวกเตอร์ขนาด $6 \times 1$:
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

### บทสรุปสมการ
สมการ $M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q$ แบบ 6-DOF นี้นำเสนอระบบ Full Trailer ที่เป็น 2-Body อย่างสมบูรณ์ (Tractor + Trailer) โดยสะท้อนพฤติกรรมที่ก้านลาก (Drawbar) ทำหน้าที่เป็นแขนเลี้ยวให้กับล้อหน้ารถพ่วง (ผ่านเทอม $\theta_d - \theta_t$) ทำให้มีความถูกต้องทางกายภาพสูงและง่ายต่อการทำ State-Space Controller เช่น LQR หรือ MPC ต่อไป
