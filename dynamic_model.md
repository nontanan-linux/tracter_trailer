# แบบจำลองพลศาสตร์ของยานยนต์ลากจูงพร้อมแรงปฏิกิริยาที่จุดพ่วง (Dynamic Model of a Towing Vehicle with Hitch Forces)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วงแบบดอลลี่และตัวพ่วง (Tractor + Drawbar Dolly + Trailer Body) ซึ่งขับเคลื่อนล้อหลัง (Rear-Wheel Drive - RWD) โดยนำเสนอทั้ง **วิธีนิวตัน-ออยเลอร์ (Newton-Euler)** และ **วิธีลากรานเจียน (Lagrangian Mechanics)** พร้อมทั้งการจัดรูปทางคณิตศาสตร์เพื่อนำไปใช้ในการจำลองระบบ (System Simulation)

---

## 0. Schematic and Coordinate Systems (แผนผังและระบบพิกัด)

![แผนภาพพิกัดและขนาดของระบบรถลากจูงพร้อมระบบพ่วง 1 ตัว (Tractor + Drawbar Trailer)](kinematic_diagram_1trailer.png)

แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 3 ชิ้นหลัก เชื่อมต่อกันด้วยจุดพ่วงแบบหมุนได้ (Revolute Joints / Hitch Joints):
1. **รถลากจูง (Tractor)**: มีมวล $m$ มีจุดศูนย์กลางมวล (CG) อยู่ที่พิกัด $(x_0, y_0)$ ในพิกัดโลก และมีทิศทางมุมหัวรถ (Yaw Angle) เท่ากับ $\theta_0$
2. **ดอลลี่ (Drawbar Dolly)**: มีมวล $m_d$ เชื่อมต่อกับท้ายรถลากจูงที่จุดพ่วง $H_1$ และมีมุมหัวดอลลี่เท่ากับ $\theta_1$
3. **ส่วนพ่วงหลัก (Trailer Body)**: มีมวล $m_t$ เชื่อมต่อกับดอลลี่ที่จุดพ่วงตัวที่สอง $H_2$ (Fifth Wheel Joint) และมีมุมหัวรถพ่วงเท่ากับ $\theta_2$

### ระบบพิกัดตามมาตรฐาน ISO 8855-2011 (Vehicle Axis System ISO 8855-2011)
ระบบพิกัดที่ใช้ในแบบจำลองพลศาสตร์นี้ อ้างอิงตามมาตรฐานสากล **ISO 8855-2011** สำหรับพลศาสตร์ยานยนต์ (Vehicle Dynamics) ซึ่งเป็นระบบพิกัดมือขวา (Right-Handed Coordinate System) ดังรูปด้านล่าง:

![Vehicle Axis System ISO 8855-2011](iso_8855_coordinate_system.png)

*   **พิกัดโลก (Global Inertial Frame: $OXY$)**: พิกัดอ้างอิงเฉื่อยบนพื้นระนาบโลกสำหรับการคำนวณทิศทางการเคลื่อนที่และพิกัดตำแหน่งสัมบูรณ์ของยานพาหนะ
*   **พิกัดอ้างอิงตัวรถ (Body-Fixed Frame: $Cxyz$)**: พิกัดที่ยึดติดอยู่กับวัตถุแข็งเกร็งแต่ละชิ้น (Tractor, Dolly, Trailer) โดยมีจุดเริ่มต้น (Origin) อยู่ที่จุดอ้างอิงที่เพลาหรือจุดศูนย์กลางมวล และมีแนวแกนพิกัดดังนี้:
    *   **แกน $x$ (Longitudinal Axis)**: ชี้ไปทางด้านหน้าของตัวรถ (Longitudinal, Forward)
    *   **แกน $y$ (Lateral Axis)**: ชี้ไปทางด้านซ้ายของตัวรถ (Lateral, To Left)
    *   **แกน $z$ (Vertical Axis)**: ชี้ขึ้นด้านบนในแนวตั้งฉากกับพื้นโลก (Vertical, Up)
*   **มุมและการหมุน (Angles & Rotations)**:
    *   **มุมหัวรถ (Yaw Angle: $\theta$ / $\psi$)**: หมุนรอบแกน $z$ (Yaw Rate คือ $r = \dot{\theta}$ หรือ $w_z$) มีทิศทางเป็นบวกเมื่อหมุนทวนเข็มนาฬิกา
    *   **มุมโคลงหน้า-หลัง (Pitch Angle: $\theta_p$)**: หมุนรอบแกน $y$ (Pitch Rate คือ $q$)
    *   **มุมเอียงข้าง (Roll Angle: $\phi$)**: หมุนรอบแกน $x$ (Roll Rate คือ $p$)

### การอธิบายพารามิเตอร์จากแผนภาพ (Parameters Explanation from Diagram)
จากแผนภาพพิกัดและขนาดของระบบรถลากจูงพร้อมระบบพ่วง 1 ตัว (`kinematic_diagram_1trailer.png`) ด้านบน สามารถอธิบายความหมายของพารามิเตอร์แต่ละตัวได้ดังตารางต่อไปนี้:

| สัญลักษณ์ (Symbol) | ประเภท (Category) | คำอธิบายภาษาไทย (Thai Description) | คำอธิบายภาษาอังกฤษ (English Description) |
| :---: | :--- | :--- | :--- |
| **$(x_0, y_0)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถลากจูง (ใช้เป็นจุดอ้างอิงตำแหน่งหลัก) | Tractor Rear Axle Center |
| **$(x_1, y_1)$** | พิกัด / ตำแหน่ง | จุดศูนย์กลางเพลาของดอลลี่ | Dolly Axle Center |
| **$(x_2, y_2)$** | พิกัด / ตำแหน่ง | กึ่งกลางเพลาหลังของรถพ่วงหลัก | Trailer Rear Axle Center |
| **$H_1 (x_{h1}, y_{h1})$** | จุดต่อ / ข้อต่อ | จุดพ่วงตัวแรก เชื่อมระหว่างท้ายรถลากจูงและคานลากของดอลลี่ | Hitch 1 (Tractor to Dolly) |
| **$H_2 (x_{h2}, y_{h2})$** | จุดต่อ / ข้อต่อ | จุดพ่วงตัวที่สอง (Fifth Wheel) เชื่อมระหว่างดอลลี่และรถพ่วงหลัก (ซ้อนทับกับพิกัดเพลาดอลลี่) | Hitch 2 (Dolly to Trailer) |
| **$L_0$** | เรขาคณิต / ขนาด | ระยะฐานล้อของรถลากจูง | Tractor Wheelbase |
| **$L_1$** | เรขาคณิต / ขนาด | ความยาวของคานลากจูงดอลลี่ วัดจากจุดพ่วง $H_1$ ถึงเพลาดอลลี่ $(x_1, y_1)$ | Drawbar Length |
| **$L_2$** | เรขาคณิต / ขนาด | ระยะฐานล้อของตัวพ่วงหลัก วัดจากจุดพ่วง $H_2$ ถึงเพลาหลังรถพ่วง $(x_2, y_2)$ | Trailer Wheelbase |
| **$d_h$** | เรขาคณิต / ขนาด | ระยะยื่นจากเพลาหลังรถลากจูงถึงจุดพ่วง $H_1$ | Tractor Rear Overhang |
| **$\theta_0$** | มุม / ทิศทาง | มุมหัวรถลากจูง เทียบกับแกนระดับโลก $X$ (Global Horizontal Axis) | Tractor Yaw Angle |
| **$\theta_1$** | มุม / ทิศทาง | มุมคานลากจูงดอลลี่ เทียบกับแกนระดับโลก $X$ | Dolly Yaw Angle |
| **$\theta_2$** | มุม / ทิศทาง | มุมหัวรถพ่วงหลัก เทียบกับแกนระดับโลก $X$ | Trailer Yaw Angle |
| **$\delta$** | มุม / ทิศทาง | มุมเลี้ยวของล้อหน้าเทียบกับแกนตามยาวของรถลากจูง | Front Wheel Steer Angle |
| **$F_{hx1}, F_{hy1}$** | แรงปฏิกิริยา | แรงดึงและแรงแนวขวางที่กระทำต่อรถลากจูง ณ จุดพ่วง $H_1$ (ในระบบพิกัดรถลากจูง) | Hitch 1 Forces (Tractor Frame) |
| **$F_{hx2}, F_{hy2}$** | แรงปฏิกิริยา | แรงดึงและแรงแนวขวางที่กระทำต่อดอลลี่ ณ จุดพ่วง $H_2$ (ในระบบพิกัดดอลลี่) | Hitch 2 Forces (Dolly Frame) |

---

## 1. Origin and Principles of the Method (ที่มาและหลักการทางพลศาสตร์)

### 1.1 หลักการของ Newton-Euler สำหรับวัตถุแข็งเกร็ง (Rigid Body Dynamics)
วิธีนิวตัน-ออยเลอร์พิจารณาสมดุลแรงและโมเมนต์ของวัตถุแต่ละชิ้นแยกกัน โดยใช้กฎข้อที่สองของนิวตันสำหรับเชิงเส้น (Translational Motion) และสมการออยเลอร์สำหรับเชิงมุม (Rotational Motion):

$$\sum \mathbf{F}_i = m_i \dot{\mathbf{v}}_i$$

$$\sum \mathbf{M}_i = I_i \dot{\boldsymbol{\omega}}_i + \boldsymbol{\omega}_i \times (I_i \boldsymbol{\omega}_i)$$

สำหรับกรณีระบบเคลื่อนที่บนระนาบ 2 มิติ (Planar Motion) จะลดรูปเหลือ 3 สมการต่อชิ้นงาน (Longitudinal, Lateral, และ Yaw):

$$m (\dot{v}_x - v_y r) = \sum F_x$$

$$m (\dot{v}_y + v_x r) = \sum F_y$$

$$I_z \dot{r} = \sum M_z$$

### 1.2 วิธีลากรานเจียน (Lagrangian Dynamics) สำหรับระบบหลายชิ้นส่วน (Multi-Body Systems)
วิธีลากรานเจียนอาศัยการวิเคราะห์พลังงานรวมของระบบ แทนที่จะพิจารณาสมดุลแรงทีละชิ้นส่วน โดยเริ่มต้นจากการกำหนดพิกัดทั่วไป (Generalized Coordinates) สำหรับระบบเคลื่อนที่บนระนาบ 2 มิติของวัตถุทั้ง 3 ชิ้น:

$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T \in \mathbb{R}^9$$

สมการลากรานจ์ชนิดที่หนึ่งร่วมกับตัวคูณลากรานจ์ (Lagrange's Equations of the First Kind) สามารถเขียนได้ในรูป:

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + \sum_{k=1}^4 \lambda_k \frac{\partial g_k}{\partial q_j}$$

โดยที่:
*   $L = T - V$ คือลากรานเจียน (Lagrangian) ของระบบ
*   $T$ คือพลังงานจลน์รวม (Total Kinetic Energy)
*   $V$ คือพลังงานศักย์รวม (Total Potential Energy) ซึ่งมีค่าเป็น $0$ เนื่องจากระบบเคลื่อนที่บนระนาบระดับเดียวกัน
*   $Q_j$ คือแรงทั่วไปที่ไม่อนุรักษ์ (Generalized Non-conservative Forces) เช่น แรงสัมผัสของยางล้อและแรงขับเคลื่อน
*   $g_k(q) = 0$ คือสมการเงื่อนไขบังคับทางตำแหน่ง (Holonomic Position Constraints) ที่จุดพ่วงข้อต่อ
*   $\lambda_k$ คือตัวคูณลากรานจ์ (Lagrange Multipliers)

### 1.3 ที่มาของความเร็วเชิงเส้นในพิกัดโลก (Derivation of Linear Velocities)
ในการสร้างพลังงานจลน์และการกำหนดสมการเชิงอนุพันธ์ของระบบ ความเร็วของวัตถุแต่ละชิ้นในพิกัดโลก (Global Inertial Frame) จะได้มาจากการทำอนุพันธ์เทียบกับเวลา (Time Derivative) ของตำแหน่งทางเรขาคณิต ดังนี้:

#### 1. รถลากจูง (Tractor)
ตำแหน่งของจุดศูนย์กลางมวล (CG) อ้างอิงอยู่ที่เพลาหลัง $(x_0, y_0)$ ความเร็วของรถลากจูงคำนวณจากแบบจำลองจักรยานจลนศาสตร์ (Kinematic Bicycle Model) ได้ดังนี้:
$$\dot{x}_0 = v_0 \cos\theta_0$$
$$\dot{y}_0 = v_0 \sin\theta_0$$
$$\dot{\theta}_0 = \frac{v_0}{L_0} \tan\delta$$

โดยที่:
*   $\dot{x}_0, \dot{y}_0$ คือ ความเร็วของจุดศูนย์กลางเพลาหลังของรถลากจูงในพิกัดโลก (Velocity of the Tractor's rear axle center in World Frame)
*   $\dot{\theta}_0$ คือ ความเร็วเชิงมุมของรถลากจูง (Angular Velocity of the Tractor)
*   $v_0$ คือ ความเร็วตามแนวแกนยาวของรถลากจูง (Longitudinal velocity of the Tractor)
*   $\delta$ คือ มุมเลี้ยวล้อหน้าของรถลากจูง (Steering angle of the Tractor's front wheels)

#### 2. ดอลลี่ (Drawbar Dolly)
จากตำแหน่งทางเรขาคณิตของจุดศูนย์กลางมวลของดอลลี่ $(x_d, y_d)$ ซึ่งสัมพันธ์กับรถลากจูงและมุมคานลากจูง:
$$x_d = x_0 - d_h \cos\theta_0 - l_{fd} \cos\theta_1$$
$$y_d = y_0 - d_h \sin\theta_0 - l_{fd} \sin\theta_1$$
เมื่อหาอนุพันธ์เทียบกับเวลา (โดยใช้กฎลูกโซ่สำหรับฟังก์ชันไซน์และโคไซน์) จะได้ความเร็วเชิงเส้นของดอลลี่ดังนี้:
$$\dot{x}_d = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + l_{fd} \dot{\theta}_1 \sin\theta_1$$
$$\dot{y}_d = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - l_{fd} \dot{\theta}_1 \cos\theta_1$$

#### 3. รถพ่วงหลัก (Trailer Body)
จากพิกัดตำแหน่งของจุดศูนย์กลางมวลรถพ่วงหลัก $(x_t, y_t)$ ที่เชื่อมต่อผ่านจุดพ่วงตัวที่สอง $H_2$ (อยู่กึ่งกลางเพลาดอลลี่):
$$x_t = x_d - l_{rd} \cos\theta_1 - l_{ft} \cos\theta_2 = x_0 - d_h \cos\theta_0 - (l_{fd} + l_{rd}) \cos\theta_1 - l_{ft} \cos\theta_2$$
$$y_t = y_d - l_{rd} \sin\theta_1 - l_{ft} \sin\theta_2 = y_0 - d_h \sin\theta_0 - (l_{fd} + l_{rd}) \sin\theta_1 - l_{ft} \sin\theta_2$$
เมื่อหาอนุพันธ์เทียบกับเวลา จะได้ความเร็วเชิงเส้นของตัวรถพ่วงหลักในพิกัดโลกดังนี้:
$$\dot{x}_t = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + (l_{fd} + l_{rd}) \dot{\theta}_1 \sin\theta_1 + l_{ft} \dot{\theta}_2 \sin\theta_2$$
$$\dot{y}_t = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - (l_{fd} + l_{rd}) \dot{\theta}_1 \cos\theta_1 - l_{ft} \dot{\theta}_2 \cos\theta_2$$

> **หมายเหตุสำหรับการอนุพันธ์และเครื่องหมาย (Derivation and Sign Convention Note):**
> การคำนวณความเร็วข้างต้นมีลักษณะการเปลี่ยนแปลงเครื่องหมายที่สอดคล้องกับหัวข้อ 4.2 ใน [README.md](README.md) โดยสืบเนื่องมาจากการหาอนุพันธ์เทียบกับเวลา (Time Derivative) ของฟังก์ชันตรีโกณมิติสำหรับพิกัดทางเรขาคณิต:
> *   **แกน $X$**: พิกัดอ้างอิงของวัตถุตามหลังคำนวณจาก $x_{\text{rear}} = x_{\text{front}} - d \cos\theta$ เมื่อทำอนุพันธ์จะได้ $-\frac{d}{dt}(\cos\theta) = \dot{\theta}\sin\theta$ เครื่องหมายลบจึงหักล้างกันเปลี่ยนเป็น **บวก ($+$)**
> *   **แกน $Y$**: พิกัดอ้างอิงของวัตถุตามหลังคำนวณจาก $y_{\text{rear}} = y_{\text{front}} - d \sin\theta$ เมื่อทำอนุพันธ์จะได้ $-\frac{d}{dt}(\sin\theta) = -\dot{\theta}\cos\theta$ เครื่องหมายจึงยังคงเป็น **ลบ ($-$)**

### 1.3.1 ที่มาของความเร็วเชิงมุมสำหรับส่วนพ่วง (Source of Trailer/Dolly Angular Velocities)
ในการคำนวณสมการทั้งหมด ความเร็วเชิงมุมของดอลลี่ ($\dot{\theta}_1$) และรถพ่วงหลัก ($\dot{\theta}_2$) จะมีแนวคิดที่แตกต่างกันระหว่างแบบจำลองจลนศาสตร์ (Kinematic Model) และแบบจำลองพลศาสตร์ (Dynamic Model) ดังนี้:

#### 1. ในแบบจำลองทางจลนศาสตร์ (Kinematic Model)
ความเร็วเชิงมุมของส่วนพ่วงจะถูกกำหนดไว้อย่างสมบูรณ์ด้วย **เงื่อนไขบังคับการไม่ลื่นไถลด้านข้างของล้อ (No-Side-Slip Constraint)** ที่แต่ละเพลาล้อ ซึ่งทำให้ความเร็วเชิงมุมขึ้นกับความเร็วเชิงเส้นของตัวรถและมุมพ่วงโดยตรง (อ้างอิงจากบทที่ 4.2 ใน [README.md](README.md)):
*   **ความเร็วเชิงมุมของดอลลี่ ($\dot{\theta}_1$)**:
    $$\dot{\theta}_1 = \frac{1}{L_1} \left( v_0 \sin(\theta_0 - \theta_1) - d_h \dot{\theta}_0 \cos(\theta_0 - \theta_1) \right)$$
*   **ความเร็วเชิงมุมของรถพ่วงหลัก ($\dot{\theta}_2$)**:
    $$\dot{\theta}_2 = \frac{1}{L_2} \left( v_1 \sin(\theta_1 - \theta_2) - d_{h1} \dot{\theta}_1 \cos(\theta_1 - \theta_2) \right)$$
    *(โดยที่ $v_1$ คือความเร็วเชิงเส้นในแนวแกนของดอลลี่)*

#### 2. ในแบบจำลองทางพลศาสตร์ (Dynamic Model)
เนื่องจากแบบจำลองพลศาสตร์ยอมให้ล้อของยานพาหนะเกิดการลื่นไถลได้ (Tire Slip Angle $\alpha \neq 0$) ความเร็วเชิงมุมของดอลลี่ ($\dot{\theta}_1$) และรถพ่วงหลัก ($\dot{\theta}_2$) จึงมีสถานะเป็น **ตัวแปรสถานะอิสระ (Independent State Variables)** ของระบบ (ไม่ถูกผูกมัดด้วยสมการพีชคณิตทางจลนศาสตร์โดยตรง) โดยอนุพันธ์เทียบเวลาลำดับที่สอง ($\ddot{\theta}_1, \ddot{\theta}_2$) จะถูกคำนวณผ่านสมการการเคลื่อนที่ทางพลศาสตร์ (Equations of Motion) จากผลรวมของโมเมนต์รอบจุดศูนย์กลางมวล (CG):
$$\ddot{\theta}_1 = \frac{\sum M_{\text{CG,dolly}}}{I_{zd}}$$
$$\ddot{\theta}_2 = \frac{\sum M_{\text{CG,trailer}}}{I_{zt}}$$

---

### 1.4 พลังงานจลน์ของระบบ (Kinetic Energy Formulation)
พลังงานจลน์รวมของระบบเกิดจากผลรวมของพลังงานจลน์จากการเคลื่อนที่เชิงเส้นและการหมุนของวัตถุทั้ง 3 ชิ้น:

$$T = T_{\text{tractor}} + T_{\text{dolly}} + T_{\text{trailer}}$$

$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_d(\dot{x}_d^2 + \dot{y}_d^2) + \frac{1}{2}I_{zd}\dot{\theta}_1^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_2^2 \right]
$$

### 1.5 สมการเงื่อนไขบังคับและการแปลง Lagrange Multipliers เป็นแรงดึงพ่วง
ตำแหน่งของจุดศูนย์กลางมวลของดอลลี่ $(x_d, y_d)$ และตัวพ่วง $(x_t, y_t)$ ถูกจำกัดไว้ด้วยข้อต่อจุดพ่วง $H_1$ และ $H_2$ เสมือนมีสมการเงื่อนไขบังคับเชิงตำแหน่ง 4 สมการดังนี้:

$$g_1(q) = x_d - x_0 + d_h \cos\theta_0 + l_{fd} \cos\theta_1 = 0$$

$$g_2(q) = y_d - y_0 + d_h \sin\theta_0 + l_{fd} \sin\theta_1 = 0$$

$$g_3(q) = x_t - x_d + l_{rd} \cos\theta_1 + l_{ft} \cos\theta_2 = 0$$

$$g_4(q) = y_t - y_d + l_{rd} \sin\theta_1 + l_{ft} \sin\theta_2 = 0$$

เมื่อเราหาอนุพันธ์ของสมการเงื่อนไขบังคับเทียบกับตำแหน่งทั่วไปเพื่อหาจาโคเบียนของเงื่อนไขบังคับ (Constraint Jacobian Matrix: $J_c = \frac{\partial g}{\partial q}$):

$$J_c = \begin{bmatrix} 
-1 & 0 & -d_h\sin\theta_0 & 1 & 0 & -l_{fd}\sin\theta_1 & 0 & 0 & 0 \\
0 & -1 & d_h\cos\theta_0 & 0 & 1 & l_{fd}\cos\theta_1 & 0 & 0 & 0 \\
0 & 0 & 0 & -1 & 0 & -l_{rd}\sin\theta_1 & 1 & 0 & -l_{ft}\sin\theta_2 \\
0 & 0 & 0 & 0 & -1 & l_{rd}\cos\theta_1 & 0 & 1 & l_{ft}\cos\theta_2
\end{bmatrix}$$

แรงปฏิกิริยาพ่วงที่กระทำต่อระบบในพิกัดทั่วไปสามารถคำนวณได้จาก $F_c = J_c^T \lambda$ โดยที่ $\lambda = [\lambda_1, \lambda_2, \lambda_3, \lambda_4]^T$ คือเวกเตอร์ตัวคูณลากรานจ์ ซึ่งมีทิศทางตามแนวพิกัดโลก:
*   $\lambda_1, \lambda_2$: แรงปฏิกิริยาพ่วงที่จุดพ่วง $H_1$ ในทิศทาง $X$ และ $Y$ ของพิกัดโลก
*   $\lambda_3, \lambda_4$: แรงปฏิกิริยาพ่วงที่จุดพ่วง $H_2$ ในทิศทาง $X$ และ $Y$ ของพิกัดโลก

เราสามารถนำ $\lambda_k$ เหล่านี้มาหมุนแปลงทิศทาง (Rotate) เพื่อหาแรงปฏิกิริยาในพิกัดตัวรถ (Body-Fixed Hitch Forces) ได้ดังนี้:

$$\begin{bmatrix} F_{hx1} \\ F_{hy1} \end{bmatrix} = \begin{bmatrix} \cos\theta_0 & \sin\theta_0 \\ -\sin\theta_0 & \cos\theta_0 \end{bmatrix} \begin{bmatrix} -\lambda_1 \\ -\lambda_2 \end{bmatrix}$$

$$\begin{bmatrix} F_{hx2} \\ F_{hy2} \end{bmatrix} = \begin{bmatrix} \cos\theta_2 & \sin\theta_2 \\ -\sin\theta_2 & \cos\theta_2 \end{bmatrix} \begin{bmatrix} \lambda_3 \\ \lambda_4 \end{bmatrix}$$

### 1.5.1 ขั้นตอนการหาอนุพันธ์สมการการเคลื่อนที่ด้วยสมการลากรานจ์ (Lagrangian Derivation of Equations of Motion)
เพื่อให้เห็นขั้นตอนการแปลงจากพลังงานจลน์ในหัวข้อ 1.4 และเงื่อนไขบังคับในหัวข้อ 1.5 ไปเป็นสมการพลศาสตร์ของระบบ เราสามารถคำนวณผ่านสมการลากรานจ์ที่มีตัวคูณลากรานจ์ (Lagrange's Equations with Multipliers) ดังนี้:

#### 1. พิกัดทั่วไป (Generalized Coordinates)
พิกัดทั่วไปของระบบที่มี 9 ระดับความอิสระ (ก่อนคิดเงื่อนไขบังคับข้อต่อ) คือ:
$$q = [x_0, y_0, \theta_0, x_d, y_d, \theta_1, x_t, y_t, \theta_2]^T$$

#### 2. สมการตั้งต้นของลากรานจ์
$$\frac{d}{dt}\left( \frac{\partial T}{\partial \dot{q}_i} \right) - \frac{\partial T}{\partial q_i} = Q_i + \sum_{k=1}^4 \lambda_k \frac{\partial g_k}{\partial q_i}$$

โดยที่ $Q_i$ คือแรงภายนอกทั่วไป (Generalized Forces) ที่เกิดจากแรงสัมผัสยางและการขับเคลื่อน

#### 3. การหาอนุพันธ์ของพลังงานจลน์ $T$
เนื่องจากพลังงานจลน์ $T$ ในหัวข้อ 1.4 เขียนอยู่ในรูปของความเร็วในพิกัดโลกโดยตรง ดังนั้นการหาอนุพันธ์ย่อยจะได้ดังนี้:
*   สำหรับแนวพิกัดโลกเชิงเส้น ($x_j, y_j$):
    $$\frac{\partial T}{\partial \dot{x}_j} = m_j \dot{x}_j \implies \frac{d}{dt}\left(\frac{\partial T}{\partial \dot{x}_j}\right) = m_j \ddot{x}_j, \quad \frac{\partial T}{\partial x_j} = 0$$
*   สำหรับแนวพิกัดการหมุน ($\theta_j$):
    $$\frac{\partial T}{\partial \dot{\theta}_j} = I_{zj} \dot{\theta}_j \implies \frac{d}{dt}\left(\frac{\partial T}{\partial \dot{\theta}_j}\right) = I_{zj} \ddot{\theta}_j, \quad \frac{\partial T}{\partial \theta_j} = 0$$

#### 4. สมการการเคลื่อนที่ในพิกัดเฉื่อยโลก (Inertial Frame Equations)
เมื่อแทนค่าอนุพันธ์และแรงเงื่อนไขบังคับจากจาโคเบียน $J_c^T \lambda$ จะได้สมการเคลื่อนที่ 9 สมการดังนี้:
1.  **Tractor Longitudinal/Lateral ($x_0, y_0$):**
    $$m \ddot{x}_0 = Q_{x0} - \lambda_1$$
    $$m \ddot{y}_0 = Q_{y0} - \lambda_2$$
2.  **Tractor Yaw ($\theta_0$):**
    $$I_z \ddot{\theta}_0 = Q_{\theta0} - d_h \sin\theta_0 \lambda_1 + d_h \cos\theta_0 \lambda_2$$
3.  **Dolly Longitudinal/Lateral ($x_d, y_d$):**
    $$m_d \ddot{x}_d = Q_{xd} + \lambda_1 - \lambda_3$$
    $$m_d \ddot{y}_d = Q_{yd} + \lambda_2 - \lambda_4$$
4.  **Dolly Yaw ($\theta_1$):**
    $$I_{zd} \ddot{\theta}_1 = Q_{\theta1} - l_{fd} \sin\theta_1 \lambda_1 + l_{fd} \cos\theta_1 \lambda_2 - l_{rd} \sin\theta_1 \lambda_3 + l_{rd} \cos\theta_1 \lambda_4$$
5.  **Trailer Longitudinal/Lateral ($x_t, y_t$):**
    $$m_t \ddot{x}_t = Q_{xt} + \lambda_3$$
    $$m_t \ddot{y}_t = Q_{yt} + \lambda_4$$
6.  **Trailer Yaw ($\theta_2$):**
    $$I_{zt} \ddot{\theta}_2 = Q_{\theta2} - l_{ft} \sin\theta_2 \lambda_3 + l_{ft} \cos\theta_2 \lambda_4$$

#### 5. การแปลงเข้าสู่พิกัดตัวรถ (Body-Fixed Transformation)
เพื่อให้สอดคล้องกับพิกัดภายในของตัวรถ ($v_x, v_y, r$) ที่เราใช้วัดและควบคุม เราจะหมุนพิกัดความเร่งโลกเข้าสู่แกนตัวรถโดยใช้เมทริกซ์การหมุน $R(\theta)$:
$$\begin{bmatrix} \dot{v}_x - v_y r \\ \dot{v}_y + v_x r \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} \ddot{x} \\ \ddot{y} \end{bmatrix}$$

เมื่อแปลงสมการของ Tractor ในข้อ 1 ข้างต้น จะได้:
$$m (\dot{v}_x - v_y r) = (Q_{x0}\cos\theta_0 + Q_{y0}\sin\theta_0) - (\lambda_1\cos\theta_0 + \lambda_2\sin\theta_0)$$
$$m (\dot{v}_y + v_x r) = (-Q_{x0}\sin\theta_0 + Q_{y0}\cos\theta_0) - (-\lambda_1\sin\theta_0 + \lambda_2\cos\theta_0)$$

เมื่อแทนค่าความสัมพันธ์ของแรงปฏิกิริยาพ่วงในพิกัดตัวรถ $F_{hx1}, F_{hy1}$ และแรงทั่วไป $F_{xf0}, F_{yf0}, F_{xr0}, F_{yr0}$ จะลดรูปเหลือ:
$$m (\dot{v}_x - v_y r) = F_{xr0} + F_{xf0} - F_{hx1}$$
$$m (\dot{v}_y + v_x r) = F_{yr0} + F_{yf0} - F_{hy1}$$
ซึ่งตรงกับสมการพลศาสตร์ของนิวตัน-ออยเลอร์ในบทที่ 4 ทุกประการ

### 1.6 ความแตกต่างเชิงเปรียบเทียบในระบบลากจูง
1.  **Newton-Euler**: มองวัตถุแต่ละชิ้นแยกกันอย่างเด็ดขาด แล้วใส่แรงดึงพ่วง $F_{hx}, F_{hy}$ เป็นแรงภายนอกกระทำกับปลายโครงสร้างทางเรขาคณิต มีข้อดีคือคำนวณง่ายตรงไปตรงมา และมีประสิทธิภาพสูงในเชิงคอมพิวเตอร์เมื่อรวมแรงดึงพ่วงเป็นตัวแปรในระบบสมการ
2.  **Lagrangian**: รวบรวมพลังงานของรถทั้งขบวนเข้าไว้ด้วยกัน ข้อดีคือสมการไม่มีแรงภายในติดอยู่ (หากใช้สมการพิกัดย่อผ่านตัวแปรมุมสัมพัทธ์) แต่หากต้องการทราบค่าของแรงปฏิกิริยาที่จุดต่อพ่วง $\lambda_k$ ก็จะต้องใช้ตัวคูณลากรานจ์ร่วมกับพิกัดแบบแยกตัวอิสระ ซึ่งให้ผลสัมพัทธ์ทางคณิตศาสตร์ที่เทียบเท่ากับวิธีนิวตัน-ออยเลอร์ทุกประการโดยสมบูรณ์

---

## 2. Definition of Variables and Parameters (การกำหนดตัวแปรและพารามิเตอร์)

### 2.1 State Variables (ตัวแปรสถานะเชิงพลศาสตร์)
| สัญลักษณ์ | คำอธิบาย | หน่วย |
| :---: | --- | :---: |
| $x_0, y_0$ | พิกัดจุด CG ของ Tractor ในพิกัดโลก | $\text{m}$ |
| $\theta_0$ | มุมทิศทาง (Yaw Angle) ของ Tractor | $\text{rad}$ |
| $\theta_1$ | มุมทิศทาง (Yaw Angle) ของ Dolly | $\text{rad}$ |
| $\theta_2$ | มุมทิศทาง (Yaw Angle) ของ Trailer Body | $\text{rad}$ |
| $v_x$ | ความเร็วแนวยาวของ Tractor ในพิกัดตัวรถ | $\text{m/s}$ |
| $v_y$ | ความเร็วแนวข้างของ Tractor ในพิกัดตัวรถ | $\text{m/s}$ |
| $r$ | อัตราการหมุน (Yaw Rate) ของ Tractor | $\text{rad/s}$ |
| $r_d$ | อัตราการหมุน (Yaw Rate) ของ Dolly ($\dot{\theta}_1$) | $\text{rad/s}$ |
| $r_t$ | อัตราการหมุน (Yaw Rate) ของ Trailer Body ($\dot{\theta}_2$) | $\text{rad/s}$ |

### 2.2 Vehicle Parameters (พารามิเตอร์รถลากและส่วนพ่วง)
| สัญลักษณ์ | คำอธิบาย | หน่วย |
| :---: | --- | :---: |
| $m, m_d, m_t$ | มวลของ Tractor, Dolly, และ Trailer ตามลำดับ | $\text{kg}$ |
| $I_z, I_{zd}, I_{zt}$ | โมเมนต์ความเฉื่อยรอบแกนดิ่งของ Tractor, Dolly, และ Trailer | $\text{kg}\cdot\text{m}^2$ |
| $l_f, l_r$ | ระยะจาก CG ของ Tractor ไปยังเพลาหน้า และเพลาหลัง | $\text{m}$ |
| $d_h$ | ระยะจาก CG ของ Tractor ยื่นไปด้านหลังถึงจุดพ่วง $H_1$ | $\text{m}$ |
| $l_{fd}$ | ระยะจากจุดพ่วง $H_1$ ไปยัง CG ของ Dolly | $\text{m}$ |
| $l_{rd}$ | ระยะจาก CG ของ Dolly ไปยังจุดพ่วงตัวที่สอง $H_2$ | $\text{m}$ |
| $l_{ft}$ | ระยะจากจุดพ่วง $H_2$ ไปยัง CG ของ Trailer | $\text{m}$ |
| $l_{rt}$ | ระยะจาก CG ของ Trailer ไปยังเพลาล้อของ Trailer | $\text{m}$ |
| $C_f, C_r$ | ค่าความแข็งเกร็งในการเลี้ยวโค้งของยางล้อหน้าและหลังของ Tractor | $\text{N/rad}$ |
| $C_d$ | ค่าความแข็งเกร็งในการเลี้ยวโค้งของยางล้อของ Dolly | $\text{N/rad}$ |
| $C_t$ | ค่าความแข็งเกร็งในการเลี้ยวโค้งของยางล้อของ Trailer | $\text{N/rad}$ |

### 2.3 Hitch & Control Inputs
*   **Hitch Forces**: แรงเชิงเส้นที่กระทำผ่านข้อต่อจุดพ่วงในทิศพิกัดของตัวรถตามแกนตัวถัง
    *   $F_{hx1}, F_{hy1}$: แรงปฏิกิริยาที่จุดพ่วง $H_1$ (ระหว่าง Tractor และ Dolly)
    *   $F_{hx2}, F_{hy2}$: แรงปฏิกิริยาที่จุดพ่วง $H_2$ (ระหว่าง Dolly และ Trailer)
*   **Control Inputs**:
    *   $\delta$: มุมเลี้ยวของล้อหน้า (Front Steering Angle) ของ Tractor
    *   $F_{xr}$: แรงขับเคลื่อนล้อหลัง (Rear Wheel Drive Thrust Force)

---

## 3. Free Body Diagram (FBD) and Force Analysis (แผนภาพวัตถุอิสระและการวิเคราะห์แรง)

### 3.1 การแตกแรงสัมผัสยาง (Tire Forces) เข้าสู่พิกัดตัวรถ
ล้อหน้ามีมุมเลี้ยว $\delta$ ดังนั้นแรงที่ล้อหน้า $F_{xf}$ (Longitudinal) และ $F_{yf}$ (Lateral) ต้องแตกเข้าแกนรถลากจูงดังนี้:

$$F_{xf0} = F_{xf}\cos\delta - F_{yf}\sin\delta$$

$$F_{yf0} = F_{xf}\sin\delta + F_{yf}\cos\delta$$

สำหรับล้อหลังที่ไม่มีการเลี้ยว มุมเลี้ยวมีค่าเป็นศูนย์ แรงสัมผัสยางจึงกระทำในแนวพิกัดตัวรถโดยตรง:

$$F_{xr0} = F_{xr}, \quad F_{yr0} = F_{yr}$$

### 3.2 ค่ามุมลื่นไถลของยาง (Tire Slip Angles: $\alpha$)
คำนวณมุมลื่นไถลจากทิศทางการเคลื่อนที่ของแกนล้อเทียบกับมุมเลี้ยวของล้อ:
*   **Tractor Front**: $\alpha_f = \arctan2(v_y + l_f r, v_x) - \delta$
*   **Tractor Rear**: $\alpha_r = \arctan2(v_y - l_r r, v_x)$
*   **Dolly Axle**: $\alpha_d = \arctan2(v_{yd}, v_{xd})$ (พิจารณาล้ออยู่ใต้ CG ของดอลลี่)
*   **Trailer Axle**: $\alpha_t = \arctan2(v_{yt} - l_{rt} r_t, v_{xt})$

แรงต้านทางข้างคูณด้วยค่าความแข็งเกร็งของหน้าสัมผัสยางตามแนวขวาง (Linear Tire Model):

$$F_{yf} = -C_f \alpha_f, \quad F_{yr} = -C_r \alpha_r, \quad F_{yd} = -C_d \alpha_d, \quad F_{yt} = -C_t \alpha_t$$

---

## 4. Step-by-Step Derivation of Equations of Motion (ขั้นตอนการสร้างสมการการเคลื่อนที่)

### 4.1 สมการการเคลื่อนที่ของ Tractor
พิจารณาแรงสัมผัสยาง และแรงดึงพ่วง $F_{hx1}, F_{hy1}$ ที่กระทำที่ท้ายรถระยะ $d_h$:

$$\text{Longitudinal:} \quad m(\dot{v}_x - v_y r) = F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta - F_{hx1}$$

$$\text{Lateral:} \quad m(\dot{v}_y + v_x r) = F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta - F_{hy1}$$

$$\text{Yaw:} \quad I_z \dot{r} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} - d_h F_{hy1}$$

---

### 4.2 สมการการเคลื่อนที่ของ Dolly
พิจารณาแรงปฏิกิริยาพ่วงสองจุด โดย $F_{h1}^d$ และ $F_{h2}^d$ แตกแรงเข้าสู่พิกัดของดอลลี่ โดยให้มุมความแตกต่างคือ $\Delta\theta_1 = \theta_0 - \theta_1$ และ $\Delta\theta_2 = \theta_1 - \theta_2$:

$$\text{Longitudinal:} \quad m_d(\dot{v}_{xd} - v_{yd} r_d) = F_{xd} + F_{hx1}\cos\Delta\theta_1 + F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 - F_{hy2}\sin\Delta\theta_2$$

$$\text{Lateral:} \quad m_d(\dot{v}_{yd} + v_{xd} r_d) = F_{yd} - F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 + F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2$$

$$\text{Yaw:} \quad I_{zd} \dot{r}_d = -l_{fd} (F_{hx1}\sin\Delta\theta_1 - F_{hy1}\cos\Delta\theta_1) + l_{rd} (F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2)$$

---

### 4.3 สมการการเคลื่อนที่ของ Trailer Body
พิจารณาแรงกระทำที่จุดเชื่อมต่อที่สอง $H_2$ และแรงสัมผัสยางล้อหลังส่วนพ่วง:

$$\text{Longitudinal:} \quad m_t(\dot{v}_{xt} - v_{yt} r_t) = F_{xt} + F_{hx2}$$

$$\text{Lateral:} \quad m_t(\dot{v}_{yt} + v_{xt} r_t) = F_{yt} + F_{hy2}$$

$$\text{Yaw:} \quad I_{zt} \dot{r}_t = -l_{ft} F_{hy2} - l_{rt} F_{yt}$$

---

### 4.4 สมการเงื่อนไขบังคับเชิงความเร่ง (Constraint Equations)
เพื่อแก้ระบบสมการพลศาสตร์ เราต้องการสมการเงื่อนไขระหว่างความเร่งของวัตถุแต่ละชิ้น ซึ่งได้มาจากการหาอนุพันธ์เทียบกับเวลา (Time Derivative) ของสมการข้อต่อความเร็ว:

#### เงื่อนไขจุดต่อที่ 1 ($H_1$):
$$\dot{v}_{xd} - \dot{v}_x \cos\Delta\theta_1 - \dot{v}_y \sin\Delta\theta_1 + d_h \dot{r} \sin\Delta\theta_1 = (r - r_d) \left[-v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1\right]$$

$$\dot{v}_{yd} + l_{fd} \dot{r}_d + \dot{v}_x \sin\Delta\theta_1 - \dot{v}_y \cos\Delta\theta_1 + d_h \dot{r} \cos\Delta\theta_1 = (r - r_d) \left[-v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1\right]$$

#### เงื่อนไขจุดต่อที่ 2 ($H_2$):
$$\dot{v}_{xt} - \dot{v}_{xd} \cos\Delta\theta_2 - \dot{v}_{yd} \sin\Delta\theta_2 + l_{rd} \dot{r}_d \sin\Delta\theta_2 = (r_d - r_t) \left[-v_{xd} \sin\Delta\theta_2 + (v_{yd} - l_{rd} r_d) \cos\Delta\theta_2\right]$$

$$\dot{v}_{yt} + l_{ft} \dot{r}_t + \dot{v}_{xd} \sin\Delta\theta_2 - \dot{v}_{yd} \cos\Delta\theta_2 + l_{rd} \dot{r}_d \cos\Delta\theta_2 = (r_d - r_t) \left[-v_{xd} \cos\Delta\theta_2 - (v_{yd} - l_{rd} r_d) \sin\Delta\theta_2\right]$$

---

## 5. System Simulation Formulation (การจัดรูปสมการสำหรับการจำลองระบบ)

สมการเชิงอนุพันธ์ทั้งหมดสามารถนำมารวบรวมเขียนในรูปแบบระบบสมการเชิงเส้นระนาบ (Linear System of Equations) ขนาด $13 \times 13$ เพื่อแก้หาค่าความเร่งและแรงดึงพ่วงแบบเรียลไทม์ (Real-Time) ที่ทุกก้าวเวลาของการคำนวณ:

$$A(q, u) \cdot X = b(q, u, F_{\text{tire}}, \delta)$$

โดยเวกเตอร์ของตัวแปรสถานะอนุพันธ์และแรงภายในที่เราต้องการหาคำตอบ ($13$ ตัวแปร) คือ:

$$X = \begin{bmatrix} \dot{v}_x & \dot{v}_y & \dot{r} & \dot{r}_d & \dot{r}_t & F_{hx1} & F_{hy1} & F_{hx2} & F_{hy2} & \dot{v}_{xd} & \dot{v}_{yd} & \dot{v}_{xt} & \dot{v}_{yt} \end{bmatrix}^T$$

### โครงสร้างเมทริกซ์ $A$ แบบเต็ม (Full Matrix $A$)
เมทริกซ์สัมประสิทธิ์ $A$ ขนาด $13 \times 13$ มีโครงสร้างสัมประสิทธิ์ดังนี้:

$$A = \begin{bmatrix}
m & 0 & 0 & 0 & 0 & -1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & m & 0 & 0 & 0 & 0 & -1 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & I_z & 0 & 0 & 0 & d_h & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & \cos\Delta\theta_1 & \sin\Delta\theta_1 & -\cos\Delta\theta_2 & \sin\Delta\theta_2 & m_d & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & -\sin\Delta\theta_1 & \cos\Delta\theta_1 & -\sin\Delta\theta_2 & -\cos\Delta\theta_2 & 0 & m_d & 0 & 0 \\
0 & 0 & 0 & I_{zd} & 0 & -l_{fd}\sin\Delta\theta_1 & l_{fd}\cos\Delta\theta_1 & l_{rd}\sin\Delta\theta_2 & l_{rd}\cos\Delta\theta_2 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & m_t & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & m_t \\
0 & 0 & 0 & 0 & I_{zt} & 0 & 0 & 0 & -l_{ft} & 0 & 0 & 0 & 0 \\
-\cos\Delta\theta_1 & -\sin\Delta\theta_1 & d_h\sin\Delta\theta_1 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
\sin\Delta\theta_1 & -\cos\Delta\theta_1 & d_h\cos\Delta\theta_1 & l_{fd} & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & l_{rd}\sin\Delta\theta_2 & 0 & 0 & 0 & 0 & 0 & -\cos\Delta\theta_2 & -\sin\Delta\theta_2 & 1 & 0 \\
0 & 0 & 0 & l_{rd}\cos\Delta\theta_2 & l_{ft} & 0 & 0 & 0 & 0 & \sin\Delta\theta_2 & -\cos\Delta\theta_2 & 0 & 1
\end{bmatrix}$$

### โครงสร้างเวกเตอร์ผลลัพธ์ $b$ แบบเต็ม (Full Vector $b$)
เวกเตอร์ด้านขวา $b$ ขนาด $13 \times 1$ มีรายละเอียดพจน์ต่าง ๆ ดังนี้:

$$b = \begin{bmatrix}
m v_y r + F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta \\
-m v_x r + F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta \\
l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr} \\
m_d v_{yd} r_d + F_{xd} \\
-m_d v_{xd} r_d + F_{yd} \\
0 \\
m_t v_{yt} r_t + F_{xt} \\
-m_t v_{xt} r_t + F_{yt} \\
-l_{rt} F_{yt} \\
(r - r_d) \left[-v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1\right] \\
(r - r_d) \left[-v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1\right] \\
(r_d - r_t) \left[-v_{xd} \sin\Delta\theta_2 + (v_{yd} - l_{rd} r_d) \cos\Delta\theta_2\right] \\
(r_d - r_t) \left[-v_{xd} \cos\Delta\theta_2 - (v_{yd} - l_{rd} r_d) \sin\Delta\theta_2\right]
\end{bmatrix}$$

### รายละเอียดรายแถวของสมการ (Row-by-Row Equations)
เพื่อความชัดเจนในการนำไปเขียนโค้ดและวิเคราะห์ทางวิชาการ สมการทั้ง 13 แถวเขียนแยกออกมาได้ดังนี้:

#### 1. พลศาสตร์ของรถลากจูง (Tractor Dynamics)
*   **แถวที่ 1 (Longitudinal):**
    $$m \dot{v}_x - F_{hx1} = m v_y r + F_{xr} + F_{xf}\cos\delta - F_{yf}\sin\delta$$
*   **แถวที่ 2 (Lateral):**
    $$m \dot{v}_y - F_{hy1} = -m v_x r + F_{yr} + F_{xf}\sin\delta + F_{yf}\cos\delta$$
*   **แถวที่ 3 (Yaw):**
    $$I_z \dot{r} + d_h F_{hy1} = l_f (F_{yf}\cos\delta + F_{xf}\sin\delta) - l_r F_{yr}$$

#### 2. พลศาสตร์ของดอลลี่ (Dolly Dynamics)
*   **แถวที่ 4 (Longitudinal):**
    $$m_d \dot{v}_{xd} + F_{hx1}\cos\Delta\theta_1 + F_{hy1}\sin\Delta\theta_1 - F_{hx2}\cos\Delta\theta_2 + F_{hy2}\sin\Delta\theta_2 = m_d v_{yd} r_d + F_{xd}$$
*   **แถวที่ 5 (Lateral):**
    $$m_d \dot{v}_{yd} - F_{hx1}\sin\Delta\theta_1 + F_{hy1}\cos\Delta\theta_1 - F_{hx2}\sin\Delta\theta_2 - F_{hy2}\cos\Delta\theta_2 = -m_d v_{xd} r_d + F_{yd}$$
*   **แถวที่ 6 (Yaw):**
    $$I_{zd} \dot{r}_d - l_{fd} F_{hx1}\sin\Delta\theta_1 + l_{fd} F_{hy1}\cos\Delta\theta_1 + l_{rd} F_{hx2}\sin\Delta\theta_2 + l_{rd} F_{hy2}\cos\Delta\theta_2 = 0$$

#### 3. พลศาสตร์ของตัวพ่วงหลัก (Trailer Body Dynamics)
*   **แถวที่ 7 (Longitudinal):**
    $$m_t \dot{v}_{xt} + F_{hx2} = m_t v_{yt} r_t + F_{xt}$$
*   **แถวที่ 8 (Lateral):**
    $$m_t \dot{v}_{yt} + F_{hy2} = -m_t v_{xt} r_t + F_{yt}$$
*   **แถวที่ 9 (Yaw):**
    $$I_{zt} \dot{r}_t - l_{ft} F_{hy2} = -l_{rt} F_{yt}$$

#### 4. สมการความเร่งเชิงบังคับ (Hitch Acceleration Constraints)
*   **แถวที่ 10 (Longitudinal Hitch 1):**
    $$-\dot{v}_x \cos\Delta\theta_1 - \dot{v}_y \sin\Delta\theta_1 + d_h \dot{r} \sin\Delta\theta_1 + \dot{v}_{xd} = (r - r_d) \left[-v_x \sin\Delta\theta_1 + (v_y - d_h r) \cos\Delta\theta_1\right]$$
*   **แถวที่ 11 (Lateral Hitch 1):**
    $$\dot{v}_x \sin\Delta\theta_1 - \dot{v}_y \cos\Delta\theta_1 + d_h \dot{r} \cos\Delta\theta_1 + l_{fd} \dot{r}_d + \dot{v}_{yd} = (r - r_d) \left[-v_x \cos\Delta\theta_1 - (v_y - d_h r) \sin\Delta\theta_1\right]$$
*   **แถวที่ 12 (Longitudinal Hitch 2):**
    $$l_{rd} \dot{r}_d \sin\Delta\theta_2 - \dot{v}_{xd} \cos\Delta\theta_2 - \dot{v}_{yd} \sin\Delta\theta_2 + \dot{v}_{xt} = (r_d - r_t) \left[-v_{xd} \sin\Delta\theta_2 + (v_{yd} - l_{rd} r_d) \cos\Delta\theta_2\right]$$
*   **แถวที่ 13 (Lateral Hitch 2):**
    $$l_{rd} \dot{r}_d \cos\Delta\theta_2 + l_{ft} \dot{r}_t + \dot{v}_{xd} \sin\Delta\theta_2 - \dot{v}_{yd} \cos\Delta\theta_2 + \dot{v}_{yt} = (r_d - r_t) \left[-v_{xd} \cos\Delta\theta_2 - (v_{yd} - l_{rd} r_d) \sin\Delta\theta_2\right]$$

---

การใช้อัลกอริทึมแก้ระบบสมการเชิงเส้นตัวเลขอย่างเช่น `np.linalg.solve(A, b)` ในแต่ละก้าวเวลาการอินทิเกรต (เช่น Euler หรือ Runge-Kutta) จะให้ผลลัพธ์ของความเร่งของตัวแปรสถานะ ($\dot{v}_x, \dot{v}_y, \dot{r}, \dot{r}_d, \dot{r}_t$) และแรงปฏิกิริยาพ่วงอย่างทันที ทำให้ระบบมีความเสถียรเชิงตัวเลขสูงมาก และเหมาะสมสำหรับการใช้งานในการจำลองแบบเรียลไทม์ (Real-Time Simulation) หรือระบบควบคุมคาดการณ์เชิงแบบจำลอง (Model Predictive Control - MPC)
