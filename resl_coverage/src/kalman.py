import numpy as np
from numpy.linalg import pinv


class Kalman():

    def __init__(self, A, B, H, Q, R):
        self.A = A
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.I = np.eye(4)

    def step_unknown(self, x, P, q, W, N, A=None, B=None, Q=None, pr=False):
        """Predicting targets' state without knowing the external input of the target.
        
            This kalman filter takes result from the previous consensus update step to 
        perform individual update step. And this filter is designed to handle the 
        situation which the *exogenous input of the target is unknown*.
            There is no central entity where all measurements are collected and 
        processed. The estimation is obtained with a combination of local computations
        at each node and of exchanges of messages among neighbors.


        ---------------------------------
        This algo is based on papers below:
        [1] A. Esna Ashari, A. Y. Kibangou, and F. Garin, "Distributed input and
            state estimation for linear discrete-time systems," in 2012 IEEE 51st
            IEEE Conference on Decision and Control (CDC), 2012, pp. 782-787.

        [2] S. Gillijns and B. De Moor, "Unbiased minimum-variance input and
            state estimation for linear discrete-time systems," Automatica, vol. 43,
            no. 1, pp. 111-116, 2007.
        ---------------------------------

        Args:
            x (numpy.ndarray): State vector of the target.
            P (numpy.ndarray): State estimation posteriori error covariance matrix.
            q (numpy.ndarray): Sensor measurements with noise. "z" in equation 41 of [1].
            W (numpy.ndarray): Output noise, which is "S" in [1] in equation 40.
            N (int): Entries number. The number of sensors which have made a consensus.
            A (numpy.ndarray, optional): State transtion matrix of the target.
            B (numpy.ndarray, optional): Input matrix.
            Q (numpy.ndarray, optional): Covariance matrix of the state containing process noise.
            pr (bool, optional): Printing the intermediate variables. Defaults to False.

        Returns:
            xc [type]: The estimated state of target.
            Pc [type]: The new covariance matrix of the state.
        """
        if A is None: A = self.A
        if B is None: B = self.B
        if Q is None: Q = self.Q

        # Prediction
        # Making prediction of the current state based on the information from the previous state.
        # Similar to a general kalman filter.
        pred_x = np.dot(A, x)       # No input correction term since we do not know the exogenous input for now
        pred_P = np.dot(A, np.dot(P, A.T)) + np.dot(N, Q)       # Covariance of the predicted state
        
        if pr:
            print("pred_x", pred_x)


        # Intermediate posteriori state error covariance
        pred_P_inter = pinv(pinv(pred_P) + W)

        # Input estimate
        theta_lit = np.dot(self.I - np.dot(W, pred_P_inter), q)
        theta_big = W - np.dot(W, np.dot(pred_P_inter, W))      
        U = np.dot(pinv(np.dot(B.T, np.dot(theta_big, B))),
                   np.dot(B.T, theta_lit) - np.dot(B.T, np.dot(theta_big, pred_x)))     # Estimated input

        # Intermediate update
        pred_x_star = pred_x + np.dot(B, U)        # Plus the input correction term by using the estimated input

        Y = np.dot(B, np.dot(pinv(np.dot(B.T, np.dot(theta_big, B))), B.T))

        pred_P_star = pred_P - np.dot(Y, np.dot(theta_big, pred_P)) \
            - np.dot(pred_P, np.dot(theta_big, Y))

        if pr:
            print("pred_x_star", pred_x_star)

        # Correction
        # Fusion the results
        xc = pred_x_star + np.dot(pred_P, 
                                  theta_lit - np.dot(theta_big, pred_x_star))
        Pc = np.dot(self.I - np.dot(pred_P, theta_big),
                    pred_P_star - np.dot(pred_P, np.dot(theta_big, Y)))

        if pr:
            print("XC", xc)

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
