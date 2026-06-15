# แบบจำลองพลศาสตร์ของยานยนต์ลากจูง (Dynamic Model of a Towing Vehicle)

เอกสารฉบับนี้จัดทำขึ้นเพื่อแสดงการอนุพันธ์ทางคณิตศาสตร์ (Mathematical Derivation) ของแบบจำลองพลศาสตร์ระนาบ (Planar Dynamic Model) สำหรับระบบรถลากจูงและส่วนพ่วงแบบดอลลี่และตัวพ่วง (Tractor + Drawbar Dolly + Trailer Body) โดยเน้นไปที่การประยุกต์ใช้ **วิธีลากรานเจียน (Lagrangian Dynamics)** ในการจัดรูปสมการการเคลื่อนที่

---

## 0. Schematic and Coordinate Systems

### 0.1 Tracter with Drawbar Trailler Diagram
![แผนภาพพิกัดและขนาดของระบบรถลากจูงพร้อมระบบพ่วง 1 ตัว (Tractor + Drawbar Trailer)](kinematic_diagram_1trailer.png)

แบบจำลองนี้ประกอบด้วยวัตถุเกร็ง (Rigid Bodies) 3 ชิ้นหลัก เชื่อมต่อกันด้วยจุดพ่วงแบบหมุนได้ (Revolute Joints / Hitch Joints):
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

สำหรับระบบของยานพาหนะที่มีแรงสัมผัสยางภายนอกที่ไม่ใช่อนุรักษ์พลังงาน (Non-conservative Forces) $Q_j$ และมีสมการเงื่อนไขบังคับเชิงตำแหน่งที่จุดพ่วง (Holonomic Constraints) $g_k(q) = 0$ สมการจะขยายเป็นสมการลากรานจ์ชนิดที่หนึ่ง (Lagrange's Equations of the First Kind):
$$\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_j}\right) - \frac{\partial L}{\partial q_j} = Q_j + \sum_{k=1}^4 \lambda_k \frac{\partial g_k}{\partial q_j}$$
โดยที่ $\lambda_k$ คือตัวคูณลากรานจ์ (Lagrange Multipliers) ซึ่งเป็นตัวแทนเชิงคณิตศาสตร์ของแรงดึงที่จุดพ่วง (Hitch Forces)

### 1.3 ที่มาของความเร็วเชิงเส้นในพิกัดโลก (Derivation of Linear Velocities)
เพื่อให้สามารถเขียนสมการพลังงานจลน์ได้ เราต้องหาความเร็วของจุดศูนย์กลางมวลของวัตถุแต่ละชิ้นในพิกัดโลก:

**1. รถลากจูง (Tractor):**
ตำแหน่งของจุดศูนย์กลางมวลอยู่ที่เพลาหลัง $(x_0, y_0)$ ความเร็วคำนวณจากแบบจำลองจักรยาน:
$$\dot{x}_0 = v_0 \cos\theta_0$$
$$\dot{y}_0 = v_0 \sin\theta_0$$
$$\dot{\theta}_0 = \frac{v_0}{L_0} \tan\delta$$

**2. ดอลลี่ (Drawbar Dolly):**
พิกัดศูนย์กลางมวลของดอลลี่ $(x_d, y_d)$ เชื่อมต่อกับท้ายรถลากจูง:
$$x_d = x_0 - d_h \cos\theta_0 - l_{fd} \cos\theta_1$$
$$y_d = y_0 - d_h \sin\theta_0 - l_{fd} \sin\theta_1$$
เมื่อหาอนุพันธ์เทียบกับเวลา (Time Derivative):
$$\dot{x}_d = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + l_{fd} \dot{\theta}_1 \sin\theta_1$$
$$\dot{y}_d = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - l_{fd} \dot{\theta}_1 \cos\theta_1$$

**3. รถพ่วงหลัก (Trailer Body):**
พิกัดศูนย์กลางมวลรถพ่วงหลัก $(x_t, y_t)$ เชื่อมต่อผ่านจุดพ่วงตัวที่สอง $H_2$:
$$x_t = x_d - l_{rd} \cos\theta_1 - l_{ft} \cos\theta_2 = x_0 - d_h \cos\theta_0 - (l_{fd} + l_{rd}) \cos\theta_1 - l_{ft} \cos\theta_2$$
$$y_t = y_d - l_{rd} \sin\theta_1 - l_{ft} \sin\theta_2 = y_0 - d_h \sin\theta_0 - (l_{fd} + l_{rd}) \sin\theta_1 - l_{ft} \sin\theta_2$$
เมื่อหาอนุพันธ์เทียบกับเวลา:
$$\dot{x}_t = \dot{x}_0 + d_h \dot{\theta}_0 \sin\theta_0 + (l_{fd} + l_{rd}) \dot{\theta}_1 \sin\theta_1 + l_{ft} \dot{\theta}_2 \sin\theta_2$$
$$\dot{y}_t = \dot{y}_0 - d_h \dot{\theta}_0 \cos\theta_0 - (l_{fd} + l_{rd}) \dot{\theta}_1 \cos\theta_1 - l_{ft} \dot{\theta}_2 \cos\theta_2$$

> **หมายเหตุการเปลี่ยนแปลงเครื่องหมาย:** เมื่ออนุพันธ์ฟังก์ชันทางเรขาคณิต $x_{\text{rear}} = x_{\text{front}} - d \cos\theta$ จะได้ $-\frac{d}{dt}(\cos\theta) = \dot{\theta}\sin\theta$ เครื่องหมายลบจึงหักล้างกันเปลี่ยนเป็น **บวก ($+$)** ส่วนแกน Y เครื่องหมายยังคงเป็นลบ

### 1.4 ที่มาของความเร็วเชิงมุมสำหรับส่วนพ่วง (Source of Trailer/Dolly Angular Velocities)
ในการกำหนดพฤติกรรมการหมุน ($\dot{\theta}_1, \dot{\theta}_2$) มีความแตกต่างระหว่างการวิเคราะห์ทางจลนศาสตร์และพลศาสตร์:

1. **ในแบบจำลองทางจลนศาสตร์ (Kinematic Model)**
   ความเร็วเชิงมุมจะถูกกำหนดไว้อย่างสมบูรณ์ด้วย **เงื่อนไขบังคับการไม่ลื่นไถลด้านข้างของล้อ (No-Side-Slip Constraint)** ที่เพลาล้อ โดยคำนวณจากความเร็วที่จุดพ่วง (Hitch Velocity) ซึ่งถูกฉาย (Project) เข้าสู่แนวตั้งฉากกับคานลากจูง:
   
   *   **ความเร็วเชิงมุมของดอลลี่ ($\dot{\theta}_1$)**: 
       ความเร็วที่จุดพ่วง $H_1$ ขับเคลื่อนการหมุนรอบแกนเพลาดอลลี่
       $$ v_{hx1} = v_0 \cos\theta_0 + d_h \dot{\theta}_0 \sin\theta_0 $$
       $$ v_{hy1} = v_0 \sin\theta_0 - d_h \dot{\theta}_0 \cos\theta_0 $$
       เมื่อพิจารณาความเร็วสัมพัทธ์ในแนวตั้งฉากกับคานลากจูง จะได้สมการความเร็วเชิงมุม:
       $$\dot{\theta}_1 = \frac{1}{L_1} \left( v_0 \sin(\theta_0 - \theta_1) - d_h \dot{\theta}_0 \cos(\theta_0 - \theta_1) \right)$$
       
   *   **ความเร็วเชิงมุมของรถพ่วงหลัก ($\dot{\theta}_2$)**:
       ความเร็วเชิงเส้นเดินหน้าของเพลาดอลลี่ ($v_1$) ทำหน้าที่ลากตัวพ่วงหลักที่จุดพ่วง $H_2$
       $$ v_1 = v_0 \cos(\theta_0 - \theta_1) + d_h \dot{\theta}_0 \sin(\theta_0 - \theta_1) $$
       เมื่อนำ $v_1$ มาฉายตั้งฉากกับคานของตัวพ่วงหลัก จะได้:
       $$\dot{\theta}_2 = \frac{v_1}{L_2} \sin(\theta_1 - \theta_2)$$

2. **ในแบบจำลองทางพลศาสตร์ (Dynamic Model)**
   เนื่องจากแบบจำลองพลศาสตร์ยอมให้หน้ายางเกิดการลื่นไถล (Tire Slip Angle $\alpha \neq 0$) ความเร็วเชิงมุมของดอลลี่ ($\dot{\theta}_1$) และรถพ่วงหลัก ($\dot{\theta}_2$) จึงมีสถานะเป็น **ตัวแปรสถานะอิสระ (Independent State Variables)** ของระบบ โดยค่าอนุพันธ์อันดับสอง ($\ddot{\theta}_1, \ddot{\theta}_2$) จะถูกคำนวณผ่านสมการการเคลื่อนที่ทางพลศาสตร์ผ่านผลรวมของโมเมนต์รอบจุดศูนย์กลางมวล

### 1.5 Kinetic Energy (พลังงานจลน์)
พลังงานจลน์รวมของระบบเกิดจากผลรวมของพลังงานจลน์จากการเคลื่อนที่เชิงเส้นและการหมุนของวัตถุทั้ง 3 ชิ้น:
$$T = T_{\text{tractor}} + T_{\text{dolly}} + T_{\text{trailer}}$$
$$
T = \left[ \frac{1}{2}m(\dot{x}_0^2 + \dot{y}_0^2) + \frac{1}{2}I_z\dot{\theta}_0^2 \right] + \left[ \frac{1}{2}m_d(\dot{x}_d^2 + \dot{y}_d^2) + \frac{1}{2}I_{zd}\dot{\theta}_1^2 \right] + \left[ \frac{1}{2}m_t(\dot{x}_t^2 + \dot{y}_t^2) + \frac{1}{2}I_{zt}\dot{\theta}_2^2 \right]
$$

### 1.6 ประกอบสมการ Lagrangian
ยานพาหนะเคลื่อนที่บนพื้นราบแนวระดับ ทำให้ไม่มีพลังงานศักย์โน้มถ่วงเข้ามาเกี่ยวข้อง ($V = 0$) ส่งผลให้ $L = T - V = T$ 

การสร้างสมการการเคลื่อนที่จะเริ่มจากสมการเงื่อนไขบังคับของข้อต่อจุดพ่วง $H_1$ และ $H_2$ 4 สมการดังนี้:
$$g_1(q) = x_d - x_0 + d_h \cos\theta_0 + l_{fd} \cos\theta_1 = 0$$
$$g_2(q) = y_d - y_0 + d_h \sin\theta_0 + l_{fd} \sin\theta_1 = 0$$
$$g_3(q) = x_t - x_d + l_{rd} \cos\theta_1 + l_{ft} \cos\theta_2 = 0$$
$$g_4(q) = y_t - y_d + l_{rd} \sin\theta_1 + l_{ft} \sin\theta_2 = 0$$

เมื่อนำสมการเงื่อนไขบังคับมาหาจาโคเบียน $J_c = \frac{\partial g}{\partial q}$ และกระจายลงในสมการออยเลอร์-ลากรานจ์:
$$\frac{d}{dt}\left( \frac{\partial T}{\partial \dot{q}_i} \right) - \frac{\partial T}{\partial q_i} = Q_i + \sum_{k=1}^4 \lambda_k \frac{\partial g_k}{\partial q_i}$$

เนื่องจาก $T$ ไม่ขึ้นกับตำแหน่ง $q_i$ โดยตรง ($\frac{\partial T}{\partial q_i} = 0$) สมการจึงอยู่ในรูปมวลและความเร่งโดยตรง:
1.  **Tractor ($x_0, y_0, \theta_0$):**
    $$m \ddot{x}_0 = Q_{x0} - \lambda_1$$
    $$m \ddot{y}_0 = Q_{y0} - \lambda_2$$
    $$I_z \ddot{\theta}_0 = Q_{\theta0} - d_h \sin\theta_0 \lambda_1 + d_h \cos\theta_0 \lambda_2$$
2.  **Dolly ($x_d, y_d, \theta_1$):**
    $$m_d \ddot{x}_d = Q_{xd} + \lambda_1 - \lambda_3$$
    $$m_d \ddot{y}_d = Q_{yd} + \lambda_2 - \lambda_4$$
    $$I_{zd} \ddot{\theta}_1 = Q_{\theta1} - l_{fd} \sin\theta_1 \lambda_1 + l_{fd} \cos\theta_1 \lambda_2 - l_{rd} \sin\theta_1 \lambda_3 + l_{rd} \cos\theta_1 \lambda_4$$
3.  **Trailer Body ($x_t, y_t, \theta_2$):**
    $$m_t \ddot{x}_t = Q_{xt} + \lambda_3$$
    $$m_t \ddot{y}_t = Q_{yt} + \lambda_4$$
    $$I_{zt} \ddot{\theta}_2 = Q_{\theta2} - l_{ft} \sin\theta_2 \lambda_3 + l_{ft} \cos\theta_2 \lambda_4$$

โดยสามารถนำเวกเตอร์ความเร่งโลกหมุนแปลงสู่พิกัดตัวรถ (Body-Fixed Transformation) ด้วยเมทริกซ์การหมุน เพื่อนำไปสร้างแบบจำลองการจำลองระบบในพิกัดความเร็วของรถ $v_x, v_y, r$ อย่างมีประสิทธิภาพ
