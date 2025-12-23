
import numpy as np
import control as ct

def get_u_to_y_tf(A, B, C, D, u_ind, y_ind):

    # B = [B_u0, B_u1]
    # shape needs to be (4,1), not (4,)
    B_u = np.array([[B[i,u_ind]] for i in range(4)])

    # y = C * x
    # yi = Ci * x
    C_y = np.zeros([1,4])
    C_y[0,y_ind] = 1

    D_u_y = np.array([[0.0]]) # shape needs to be (1,1)

    tf_u_to_y = ct.ss2tf(ct.ss(A, B_u, C_y, D_u_y))

    return tf_u_to_y
