import numpy as np

def build_A(d_theta1, d_theta2):
    m = 3000.0; I_z = 2600.0; l_f = 0.64; l_r = 0.64; d_h = 0.62
    m_d = 6000.0; I_zd = 1000.0; L_bar = 1.5
    m_t = 6000.0; I_zt = 5200.0; l_ft = 0.64; l_rt = 0.64
    
    A = np.zeros((13, 13))
    
    A[0, 0] = m; A[0, 9] = 1.0
    A[1, 1] = m; A[1, 10] = 1.0
    A[2, 2] = I_z; A[2, 10] = d_h
    
    A[3, 3] = m_d; A[3, 9] = -np.cos(d_theta1); A[3, 10] = -np.sin(d_theta1); A[3, 11] = np.cos(d_theta2); A[3, 12] = np.sin(d_theta2)
    A[4, 4] = m_d; A[4, 9] = np.sin(d_theta1); A[4, 10] = -np.cos(d_theta1); A[4, 11] = -np.sin(d_theta2); A[4, 12] = np.cos(d_theta2)
    A[5, 5] = I_zd; A[5, 9] = L_bar * np.sin(d_theta1); A[5, 10] = -L_bar * np.cos(d_theta1)
    
    A[6, 6] = m_t; A[6, 11] = -1.0
    A[7, 7] = m_t; A[7, 12] = -1.0
    A[8, 8] = I_zt; A[8, 12] = l_ft
    
    A[9, 3] = 1.0; A[9, 0] = -np.cos(d_theta1); A[9, 1] = -np.sin(d_theta1); A[9, 2] = d_h * np.sin(d_theta1)
    A[10, 4] = 1.0; A[10, 5] = L_bar; A[10, 0] = np.sin(d_theta1); A[10, 1] = -np.cos(d_theta1); A[10, 2] = d_h * np.cos(d_theta1)
    
    A[11, 6] = 1.0; A[11, 3] = -np.cos(d_theta2); A[11, 4] = -np.sin(d_theta2)
    A[12, 7] = 1.0; A[12, 8] = l_ft; A[12, 3] = np.sin(d_theta2); A[12, 4] = -np.cos(d_theta2)
    
    return A

A = build_A(0.0, 0.0)
print("Cond at 0, 0:", np.linalg.cond(A))

angles = np.linspace(0, 2*np.pi, 100)
for a in angles:
    A = build_A(a, 0.0)
    if np.linalg.cond(A) > 1e10:
        print("Ill conditioned at d_theta1 =", a)

print("Check finished.")
