import numpy as np
from numpy.linalg import inv, pinv

class Kalman():

    def __init__(self, A, B, H, Q, R):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.I = np.eye(6)

    def step_unknown(self, x, P, q, W, N, A=None, B=None, H=None, Q=None, R=None, pr=False):
        if A is None: A = self.A
        if B is None: B = self.B
        if H is None: H = self.H
        if Q is None: Q = self.Q
        if R is None: R = self.R

        noise = 0.001

        # Prediction
        #pred_P = np.dot(N, np.dot(A, np.dot(P, A.T)) + Q)
        pred_P = np.dot(A, np.dot(P, A.T)) + np.dot(N, Q)
        pred_x = np.dot(A, x)

        #if pr:
        #    print("Pred_P")
        #    print(pred_P)

        # Intermediate posteriori state error covariance
        pred_P_inter = pinv(pinv(pred_P) + W)

        #if pr:
        #    print("Pred_P_inter", pred_P_inter)

        # Input estimate
        theta_lit = np.dot(self.I - np.dot(W, pred_P_inter), q)
        theta_big = W - np.dot(W, np.dot(pred_P_inter, W))

        #gram = np.dot(B.T, np.dot(theta_big, B)) + (np.random.random((3, 3)) * noise)
        #U = np.dot(inv(gram),
        #        np.dot(B.T, theta_lit) - np.dot(B.T, np.dot(theta_big, pred_x)))
        
        U = np.dot(pinv(np.dot(B.T, np.dot(theta_big, B))),
                np.dot(B.T, theta_lit) - np.dot(B.T, np.dot(theta_big, pred_x)))

        # Begin TRO Divergence
        #U = np.dot(pinv(np.dot(B.T, np.dot(theta_big, B))),
        #        np.dot(B.T, theta_lit) - np.dot(B.T, np.dot(W, pred_x)))
        #Y = np.dot(B, np.dot(pinv(np.dot(B.T, np.dot(W, B))), B.T))
        #pred_P_star = pred_P - np.dot(Y, np.dot(W, pred_P)) - np.dot(pred_P, np.dot(W, Y))
        #Pc = np.dot(self.I - np.dot(pred_P, W), pred_P_star - np.dot(pred_P, np.dot(W, Y)))
        # End TRO Divergence

        # Intermediate update
        #Y = np.dot(B, np.dot(inv(gram), B.T))

        Y = np.dot(B, np.dot(pinv(np.dot(B.T, np.dot(theta_big, B))), B.T))
        pred_x_star = pred_x + np.dot(B, U)
        pred_P_star = pred_P - np.dot(Y, np.dot(theta_big, pred_P)) \
                        - np.dot(pred_P, np.dot(theta_big, Y))

        if pr:
            print("pred_P")
            print(pred_P)
        #    print("Y x TB x pred_P")
        #    print(np.dot(Y, np.dot(theta_big, pred_P)))
        #    print("pred_P x TB x Y")
        #    print(np.dot(pred_P, np.dot(theta_big, Y)))
        #    print("Y x TB")
        #    print(np.dot(Y, theta_big))
            print("pred_P_star") 
            print(pred_P_star)
        #    print("Pred_P")
        #    print("P x t x T")
        #    print(np.dot(pred_P, theta_lit - np.dot(theta_big, pred_x_star)))

        # Correction
        xc = pred_x_star + np.dot(pred_P,
                theta_lit - np.dot(theta_big, pred_x_star))
        Pc = np.dot(self.I - np.dot(pred_P, theta_big),
                pred_P_star - np.dot(pred_P, np.dot(theta_big, Y)))

        #PC = Pc / N

        return (xc, Pc)

    def step_known(self, x, P, u, z, A=None, B=None, H=None, Q=None, R=None, pr=False):
        if A is None: A = self.A
        if B is None: B = self.B
        if H is None: H = self.H
        if Q is None: Q = self.Q
        if R is None: R = self.R

        pred_x = np.dot(A, x) + np.dot(B, u)
        pred_P = np.dot(A, np.dot(P, A.T)) + Q

        K = np.dot(pred_P, np.dot(H.T, pinv(np.dot(H, np.dot(pred_P, H.T)) + R)))

        xc = pred_x + np.dot(K, z - np.dot(H, pred_x))
        Pc = pred_P - np.dot(K, np.dot(H, pred_P))

        
        # Clean
        counter = 0
        for i in range(len(Pc)):
            for j in range(len(Pc[i])):
                counter += 1
                if Pc[i][j] < 0.:
                    Pc[i][j] = 0.
                else:
                    pass

        return xc, Pc

    def update_R(self, R):
        self.R = R
