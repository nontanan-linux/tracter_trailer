# แบบจำลองพลศาสตร์ของยานยนต์ลากจูง (Dynamic Model of a Towing Vehicle)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วงแบบ Full Trailer (Tractor + Drawbar + Trailer Body) โดยเน้นไปที่การประยุกต์ใช้ **วิธีลากรานเจียน (Lagrangian Dynamics)** ในการจัดรูปสมการการเคลื่อนที่

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 3 ชิ้นหลัก เชื่อมต่อกันด้วยจุดพ่วงแบบหมุนได้ (Revolute Joints / Hitch Joints):
1. **รถลากจูง (Tractor)**: มีมวล $m$ มีจุดศูนย์กลางมวล (CG) อยู่ที่พิกัด $(x_0, y_0)$ ในพิกัดโลก และมีทิศทางมุมหัวรถ (Yaw Angle) เท่ากับ $\theta_0$
2. **ชุดก้านลากและล้อหน้า (Drawbar / Dolly)**: มีมวล $m_d$ เปรียบเสมือนล้อหน้าของรถพ่วงที่สามารถหมุนเลี้ยวได้ เชื่อมต่อกับท้ายรถลากจูงที่จุดพ่วง $H_1$ ด้วยความยาวคานก้านลาก $L_{bar}$ และมีมุมหันเหคือ $\theta_1$
3. **ตัวถังรถพ่วงหลัก (Trailer Body)**: มีมวล $m_t$ เชื่อมต่อกับแกนเพลาของก้านลากพอดี (ไม่มีระยะยื่น $l_{rd}$) ทำให้โครงสร้างเป็นเหมือนรถบรรทุกคันเดียวที่มีล้อหน้าคือ Drawbar และมีมุมหัวรถพ่วงเท่ากับ $\theta_2$

#### การอธิบายพารามิเตอร์จากแผนภาพ (Parameters Explanation)
| สัญลักษณ์ (Symbol) | ประเภท (Category) | คำอธิบายภาษาไทย (Thai Description) | คำอธิบายภาษาอังกฤษ (English Description) |
| :---: | :--- | :--- | :--- |
| **$(x_0, y_0)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถลากจูง (จุดอ้างอิงหลัก) | Tractor Rear Axle Center |
| **$(x_d, y_d)$** | พิกัด / ตำแหน่ง | จุดศูนย์กลางเพลาหน้าของ Drawbar | Drawbar Front Axle Center |
| **$(x_t, y_t)$** | พิกัด / ตำแหน่ง | จุดศูนย์กลางมวลของรถพ่วงหลัก | Trailer Center of Gravity |
| **$H_1$** | จุดต่อ / ข้อต่อ | จุดพ่วง ระหว่างรถลากจูงและคานลาก | Hitch 1 (Tractor to Drawbar) |
| **$H_2$** | จุดต่อ / ข้อต่อ | จุดเชื่อมตัวรถพ่วง (อยู่ตรงตำแหน่งเพลาหน้าของ Drawbar พอดี) | Hitch 2 (Drawbar to Trailer Body) |
| **$L_{bar}$** | เรขาคณิต / ขนาด | ความยาวของคานลากจูง จาก $H_1$ ถึงเพลาหน้า | Drawbar Arm Length |
| **$d_h$** | เรขาคณิต / ขนาด | ระยะยื่นจากเพลาหลังรถลากจูงถึงจุดพ่วง $H_1$ | Tractor Rear Overhang |
| **$l_{ft}, l_{rt}$** | เรขาคณิต / ขนาด | ระยะจาก CG รถพ่วง ไปถึงเพลาหน้า และ เพลาหลัง | Trailer Front/Rear Lengths |
| **$\theta_0$** | มุม / ทิศทาง | มุมหัวรถลากจูง เทียบกับแกนระดับโลก $X$ | Tractor Yaw Angle |
| **$\theta_1$** | มุม / ทิศทาง | มุมคานลากจูง เทียบกับแกนระดับโลก $X$ | Drawbar Yaw Angle |
| **$\theta_2$** | มุม / ทิศทาง | มุมหัวรถพ่วงหลัก เทียบกับแกนระดับโลก $X$ | Trailer Yaw Angle |
| **$\delta$** | มุม / ทิศทาง | มุมเลี้ยวของล้อหน้าเทียบกับแกนตามยาวของรถ | Front Wheel Steer Angle |

---

## 1. Origin and Principles of the Method

การวิเคราะห์พลศาสตร์ของระบบหลายชิ้นส่วน (Multi-Body Dynamics) ที่มีการเชื่อมต่อกันด้วยจุดพ่วง สามารถทำได้โดยใช้วิธีพลังงานเพื่อลดความซับซ้อนของการคำนวณแรงปฏิกิริยาภายใน

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบหลายชิ้นส่วน
ระบบนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 3 ชิ้น (รถลากจูง, ก้านลาก Drawbar, รถพ่วงหลัก) โดยพิกัดทั่วไปทั้งหมดมี 9 ตัวแปร:
$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T \in \mathbb{R}^9$$

#### สมการออยเลอร์-ลากรานจ์ (Euler-Lagrange Equation)
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + F_{h,j}$$

### 1.3 ที่มาของความเร็วเชิงเส้นในพิกัดโลก (Derivation of Linear Velocities)
การคิดความเร็วของ Drawbar จะใช้แค่ระยะ $L_{bar}$ เท่านั้น เนื่องจาก Trailer มองเหมือนรถคันเดียวกันโดยจุดพ่วง $H_2$ เกาะอยู่ตรงตำแหน่งแกนเพลาของ Drawbar พอดี ($l_{rd} = 0$)

**1. รถลากจูง (Tractor):**
$$\dot{x}_0 = v_0 \cos\theta_0$$
$$\dot{y}_0 = v_0 \sin\theta_0$$

**2. ก้านลาก (Drawbar / Dolly):**
พิกัด $(x_d, y_d)$ เชื่อมต่อกับท้ายรถลากจูงด้วยระยะ $L_{bar}$:
$$x_d = x_0 - d_h \cos\theta_0 - L_{bar} \cos\theta_1$$
$$y_d = y_0 - d_h \sin\theta_0 - L_{bar} \sin\theta_1$$
เมื่อหาอนุพันธ์เทียบกับเวลา:
$$\dot{x}_d = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + L_{bar} \dot{\theta}_1 \sin\theta_1$$
$$\dot{y}_d = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - L_{bar} \dot{\theta}_1 \cos\theta_1$$

**3. รถพ่วงหลัก (Trailer Body):**
พิกัด $(x_t, y_t)$ เชื่อมต่อที่แกนเพลาของ Drawbar พอดิบพอดี:
$$x_t = x_d - l_{ft} \cos\theta_2 = x_0 - d_h \cos\theta_0 - L_{bar} \cos\theta_1 - l_{ft} \cos\theta_2$$
$$y_t = y_d - l_{ft} \sin\theta_2 = y_0 - d_h \sin\theta_0 - L_{bar} \sin\theta_1 - l_{ft} \sin\theta_2$$
เมื่อหาอนุพันธ์เทียบกับเวลา:
$$\dot{x}_t = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + L_{bar} \dot{\theta}_1 \sin\theta_1 + l_{ft} \dot{\theta}_2 \sin\theta_2$$
$$\dot{y}_t = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - L_{bar} \dot{\theta}_1 \cos\theta_1 - l_{ft} \dot{\theta}_2 \cos\theta_2$$

### 1.5 Kinetic and Potential Energy (พลังงานจลน์และพลังงานศักย์)
$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_d(\dot{x}_d^2 + \dot{y}_d^2) + \frac{1}{2}I_{zd}\dot{\theta}_1^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_2^2 \right]
$$
$$V = 0 \implies L = T$$

---

## 2. การอนุพันธ์สมการการเคลื่อนที่ด้วยพิกัดทั่วไป

### 2.1 การหาอนุพันธ์ของสมการลากรานจ์ $L$
กำหนดให้เวกเตอร์พิกัดทั่วไป (Generalized Coordinates) ของระบบคือ:
$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T$$

จากสมการลากรานจ์ $L = T - V$ (โดยที่ $V=0$ ทำให้ $L=T$) เราทำการหาอนุพันธ์เทียบกับพิกัดทั่วไปและอัตราเร็วของพิกัดทั่วไปแต่ละตัวแปร เพื่อสร้างเทอมฝั่งซ้ายของสมการออยเลอร์-ลากรานจ์ $\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i}$:

**1. พิกัดของรถลากจูง (Tractor): $q_1 \dots q_3$**
*   $q_1 = x_0$: $\quad \frac{\partial L}{\partial \dot{x}_0} = m \dot{x}_0, \quad \frac{\partial L}{\partial x_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_0}\right) - \frac{\partial L}{\partial x_0} = m \ddot{x}_0$
*   $q_2 = y_0$: $\quad \frac{\partial L}{\partial \dot{y}_0} = m \dot{y}_0, \quad \frac{\partial L}{\partial y_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_0}\right) - \frac{\partial L}{\partial y_0} = m \ddot{y}_0$
*   $q_3 = \theta_0$: $\quad \frac{\partial L}{\partial \dot{\theta}_0} = I_z \dot{\theta}_0, \quad \frac{\partial L}{\partial \theta_0} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_0}\right) - \frac{\partial L}{\partial \theta_0} = I_z \ddot{\theta}_0$

**2. พิกัดของก้านลาก (Drawbar / Dolly): $q_4 \dots q_6$**
*   $q_4 = x_d$: $\quad \frac{\partial L}{\partial \dot{x}_d} = m_d \dot{x}_d, \quad \frac{\partial L}{\partial x_d} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_d}\right) - \frac{\partial L}{\partial x_d} = m_d \ddot{x}_d$
*   $q_5 = y_d$: $\quad \frac{\partial L}{\partial \dot{y}_d} = m_d \dot{y}_d, \quad \frac{\partial L}{\partial y_d} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_d}\right) - \frac{\partial L}{\partial y_d} = m_d \ddot{y}_d$
*   $q_6 = \theta_1$: $\quad \frac{\partial L}{\partial \dot{\theta}_1} = I_{zd} \dot{\theta}_1, \quad \frac{\partial L}{\partial \theta_1} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_1}\right) - \frac{\partial L}{\partial \theta_1} = I_{zd} \ddot{\theta}_1$

**3. พิกัดของรถพ่วงหลัก (Trailer Body): $q_7 \dots q_9$**
*   $q_7 = x_t$: $\quad \frac{\partial L}{\partial \dot{x}_t} = m_t \dot{x}_t, \quad \frac{\partial L}{\partial x_t} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{x}_t}\right) - \frac{\partial L}{\partial x_t} = m_t \ddot{x}_t$
*   $q_8 = y_t$: $\quad \frac{\partial L}{\partial \dot{y}_t} = m_t \dot{y}_t, \quad \frac{\partial L}{\partial y_t} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{y}_t}\right) - \frac{\partial L}{\partial y_t} = m_t \ddot{y}_t$
*   $q_9 = \theta_2$: $\quad \frac{\partial L}{\partial \dot{\theta}_2} = I_{zt} \dot{\theta}_2, \quad \frac{\partial L}{\partial \theta_2} = 0 \implies \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}_2}\right) - \frac{\partial L}{\partial \theta_2} = I_{zt} \ddot{\theta}_2$

### 2.2 การหาแรงทั่วไป (Generalized Forces $Q_i$)
แรงทั่วไป $Q_i$ คือผลรวมของแรงภายนอกที่ไม่ใช่อนุรักษ์พลังงาน (ไม่รวมแรงปฏิกิริยาพ่วง $\lambda$) ที่ส่งผลกระทบต่อพิกัดทั่วไป $q_i$ ซึ่งสำหรับยานพาหนะนี้จะประกอบไปด้วย แรงหน้าสัมผัสยางล้อ (Tire Forces) และ แรงขับเคลื่อน (Traction Forces) ที่กระทำต่อวัตถุแต่ละชิ้น โดยสามารถนิยามได้ดังนี้:

**1. รถลากจูง (Tractor): $Q_1 \dots Q_3$**
*   $Q_1 = Q_{x0} = \sum F_{X0}$ (ผลรวมแรงภายนอกในแนวแกน X โลก)
*   $Q_2 = Q_{y0} = \sum F_{Y0}$ (ผลรวมแรงภายนอกในแนวแกน Y โลก)
*   $Q_3 = Q_{\theta0} = \sum M_{z0}$ (ผลรวมโมเมนต์ภายนอกรอบจุดศูนย์กลางมวลรถลากจูง)

**2. ก้านลาก (Drawbar / Dolly): $Q_4 \dots Q_6$**
*   $Q_4 = Q_{xd} = \sum F_{Xd}$ (ผลรวมแรงภายนอกในแนวแกน X โลก)
*   $Q_5 = Q_{yd} = \sum F_{Yd}$ (ผลรวมแรงภายนอกในแนวแกน Y โลก)
*   $Q_6 = Q_{\theta1} = \sum M_{zd}$ (ผลรวมโมเมนต์ภายนอกรอบจุดศูนย์กลางเพลาหน้าของก้านลาก)

**3. รถพ่วงหลัก (Trailer Body): $Q_7 \dots Q_9$**
*   $Q_7 = Q_{xt} = \sum F_{Xt}$ (ผลรวมแรงภายนอกในแนวแกน X โลก)
*   $Q_8 = Q_{yt} = \sum F_{Yt}$ (ผลรวมแรงภายนอกในแนวแกน Y โลก)
*   $Q_9 = Q_{\theta2} = \sum M_{zt}$ (ผลรวมโมเมนต์ภายนอกรอบจุดศูนย์กลางมวลรถพ่วงหลัก)

### 2.3 สมการการเคลื่อนที่ในพิกัดโลก (Inertial Equations of Motion)
ประกอบสมการความเร่งโดยมีแรงปฏิกิริยาพ่วง $\lambda_1, \lambda_2$ (ที่ $H_1$) และ $\lambda_3, \lambda_4$ (ที่ $H_2$ ซึ่งอยู่ตำแหน่งเพลาหน้าของ Drawbar พอดี):

1.  **Tractor ($x_0, y_0, \theta_0$):**
    $$m \ddot{x}_0 = Q_{x0} - \lambda_1$$
    $$m \ddot{y}_0 = Q_{y0} - \lambda_2$$
    $$I_z \ddot{\theta}_0 = Q_{\theta0} - d_h \sin\theta_0 \lambda_1 + d_h \cos\theta_0 \lambda_2$$
2.  **Drawbar ($x_d, y_d, \theta_1$):**
    $$m_d \ddot{x}_d = Q_{xd} + \lambda_1 - \lambda_3$$
    $$m_d \ddot{y}_d = Q_{yd} + \lambda_2 - \lambda_4$$
    $$I_{zd} \ddot{\theta}_1 = Q_{\theta1} - L_{bar} \sin\theta_1 \lambda_1 + L_{bar} \cos\theta_1 \lambda_2$$
    *(หมายเหตุ: แรง $\lambda_3, \lambda_4$ จาก Trailer กระทำตรงที่จุด CG ของ Drawbar พอดี จึงไม่สร้างโมเมนต์รอบ $\theta_1$)*
3.  **Trailer Body ($x_t, y_t, \theta_2$):**
    $$m_t \ddot{x}_t = Q_{xt} + \lambda_3$$
    $$m_t \ddot{y}_t = Q_{yt} + \lambda_4$$
    $$I_{zt} \ddot{\theta}_2 = Q_{\theta2} - l_{ft} \sin\theta_2 \lambda_3 + l_{ft} \cos\theta_2 \lambda_4$$

### 2.4 สมการการเคลื่อนที่ในพิกัดตัวรถ (Body-Fixed Equations of Motion)
แปลงความเร่ง $\ddot{x}, \ddot{y}$ เข้าสู่พิกัดตัวรถ $(\dot{v}_x - v_y r)$ และแตกแรงหน้ายาง:

#### 1. รถลากจูง (Tractor)
$$\text{Longitudinal:} \quad m(\dot{v}_x - v_y r) = F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1}$$
$$\text{Lateral:} \quad m(\dot{v}_y + v_x r) = F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1}$$
$$\text{Yaw:} \quad I_z \dot{r} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy1}$$

#### 2. ก้านลาก (Drawbar)
กำหนดมุมสัมพัทธ์ $\Delta\theta_1 = \theta_0 - \theta_1$ และ $\Delta\theta_2 = \theta_1 - \theta_2$:
$$\text{Longitudinal:} \quad m_d(\dot{v}_{xd} - v_{yd} r_d) = F_{xd} + F_{hx1}\cos\Delta\theta_1 + F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2$$
$$\text{Lateral:} \quad m_d(\dot{v}_{yd} + v_{xd} r_d) = F_{yd} - F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2$$
$$\text{Yaw:} \quad I_{zd} \dot{r}_d = -L_{bar} (F_{hx1}\sin\Delta\theta_1 - F_{hy1}\cos\Delta\theta_1)$$

#### 3. รถพ่วงหลัก (Trailer Body)
$$\text{Longitudinal:} \quad m_t(\dot{v}_{xt} - v_{yt} r_t) = F_{xt} + F_{hx2}$$
$$\text{Lateral:} \quad m_t(\dot{v}_{yt} + v_{xt} r_t) = F_{yt} + F_{hy2}$$
$$\text{Yaw:} \quad I_{zt} \dot{r}_t = -l_{ft} F_{hy2} - l_{rt} F_{yt}$$

### 2.5 สมการเงื่อนไขบังคับเชิงความเร่ง (Hitch Acceleration Constraints)
อนุพันธ์ความเร็วข้อต่อเพื่อให้ได้เงื่อนไขเชิงความเร่ง ($\ddot{g} = 0$) ที่จุด $H_1$ และ $H_2$ 4 สมการดังนี้:

**เงื่อนไขจุดต่อที่ 1 ($H_1$):**
$$\dot{v}_{xd} - \dot{v}_x \cos\Delta\theta_1 - \dot{v}_y \sin\Delta\theta_1 + d_h \dot{r} \sin\Delta\theta_1 = (r - r_d) \left[-v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1\right]$$
$$\dot{v}_{yd} + L_{bar} \dot{r}_d + \dot{v}_x \sin\Delta\theta_1 - \dot{v}_y \cos\Delta\theta_1 + d_h \dot{r} \cos\Delta\theta_1 = (r - r_d) \left[-v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1\right]$$

**เงื่อนไขจุดต่อที่ 2 ($H_2$ เชื่อมที่เพลา Drawbar พอดี ทำให้ $l_{rd}=0$):**
$$\dot{v}_{xt} - \dot{v}_{xd} \cos\Delta\theta_2 - \dot{v}_{yd} \sin\Delta\theta_2 = (r_d - r_t) \left[-v_{xd} \sin\Delta\theta_2 + v_{yd} \cos\Delta\theta_2\right]$$
$$\dot{v}_{yt} + l_{ft} \dot{r}_t + \dot{v}_{xd} \sin\Delta\theta_2 - \dot{v}_{yd} \cos\Delta\theta_2 = (r_d - r_t) \left[-v_{xd} \cos\Delta\theta_2 - v_{yd} \sin\Delta\theta_2\right]$$

### 2.6 การจัดรูประบบสมการเมทริกซ์ (Matrix Formulation)
$$ M(q)\ddot{q} + C(q,\dot{q})\dot{q} = Q $$

#### 1. เวกเตอร์ความเร่งและความเร็ว
*   $\ddot{q} = [\dot{v}_x, \dot{v}_y, \dot{r}, \dot{v}_{xd}, \dot{v}_{yd}, \dot{r}_d, \dot{v}_{xt}, \dot{v}_{yt}, \dot{r}_t]^T$
*   $\dot{q} = [v_x, v_y, r, v_{xd}, v_{yd}, r_d, v_{xt}, v_{yt}, r_t]^T$

#### 2. เมทริกซ์มวลและความเฉื่อย $M(q)$
$$ M(q) = \text{diag}(m, m, I_z, m_d, m_d, I_{zd}, m_t, m_t, I_{zt}) $$

#### 3. เมทริกซ์คอริโอลิสและแรงหนีศูนย์กลาง $C(q, \dot{q})$
$$
C(q, \dot{q}) = \begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_d r_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d r_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
$$

#### 4. เวกเตอร์แรงทั่วไป $Q$
$$
Q = \begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy1} \\
F_{xd} + F_{hx1}\cos\Delta\theta_1 + F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2 \\
F_{yd} - F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2 \\
-L_{bar} (F_{hx1}\sin\Delta\theta_1 - F_{hy1}\cos\Delta\theta_1) \\
F_{xt} + F_{hx2} \\
F_{yt} + F_{hy2} \\
-l_{ft} F_{hy2} - l_{rt} F_{yt}
\end{bmatrix}
$$

#### 5. สมการเต็มรูปแบบ (Full Expanded Equation)
$$
\begin{bmatrix}
m & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & m & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & I_z & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & m_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & I_{zd} & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & m_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & I_{zt}
\end{bmatrix}
\begin{bmatrix}
\dot{v}_x \\ \dot{v}_y \\ \dot{r} \\ \dot{v}_{xd} \\ \dot{v}_{yd} \\ \dot{r}_d \\ \dot{v}_{xt} \\ \dot{v}_{yt} \\ \dot{r}_t
\end{bmatrix}
+
\begin{bmatrix}
0 & -m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
m r & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & -m_d r_d & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & m_d r_d & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & -m_t r_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & m_t r_t & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
v_x \\ v_y \\ r \\ v_{xd} \\ v_{yd} \\ r_d \\ v_{xt} \\ v_{yt} \\ r_t
\end{bmatrix}
=
\begin{bmatrix}
F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1} \\
F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1} \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy1} \\
F_{xd} + F_{hx1}\cos\Delta\theta_1 + F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2 \\
F_{yd} - F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2 \\
-L_{bar} (F_{hx1}\sin\Delta\theta_1 - F_{hy1}\cos\Delta\theta_1) \\
F_{xt} + F_{hx2} \\
F_{yt} + F_{hy2} \\
-l_{ft} F_{hy2} - l_{rt} F_{yt}
\end{bmatrix}
$$
