import numpy as np
from fxpmath import Fxp
from FPGAJacobi.Cordic import Cordic
from tqdm import tqdm
import os

class Jacobi:
    def __init__(self, 
                 n_cordic_iterations : int = 12, 
                 n_word : int = 18, 
                 n_frac : int = 15, 
                 is_signed : bool = True,
                 matrix_dim : int = 12,
                 n_sweeps : int = 1):
                
                 
    
        self._n_word = n_word
        self._n_frac = n_frac
        self._is_signed = is_signed
        self._rotation_cordic = Cordic(mode = "rotation", 
                              n_iterations = n_cordic_iterations, 
                              n_word = self._n_word, 
                              n_frac = self._n_frac, 
                              is_signed = True)
        self._vectoring_cordic = Cordic(mode = "vectoring", 
                              n_iterations = n_cordic_iterations, 
                              n_word = self._n_word, 
                              n_frac = self._n_frac, 
                              is_signed = True)
        self._N = matrix_dim
        self._A = Fxp(np.zeros((self._N, self._N)), n_word = self._n_word, n_frac = self._n_frac, signed = self._is_signed,
                      rounding = "around")
        self._V = Fxp(np.zeros((self._N, self._N)), n_word = self._n_word, n_frac = self._n_frac, signed = self._is_signed,
                      rounding = "around")
        
        self._round_state = np.arange(self._N)
        
        self._n_sweeps = n_sweeps
        
    
    def get_A(self, A : Fxp):
        self._A.equal(A)
    
    def init_V(self):
        self._V.set_val(np.eye(self._N))
        
    def init_round_state(self):
        self._round_state = np.arange(self._N)
        
    def update_matrices(self, i,j):
        
        zero = Fxp(0, n_word = self._n_word, n_frac = self._n_frac, signed = self._is_signed, rounding = "around")
        
        Aij2 = self._A[i,j] << 1
        pi_to_add = 0*zero
        sign = 0*zero
        
        #hadle |2theta| > 90 degrees 
        if (self._A[j,j] > self._A[i,i]):
            Ajj_ii= self._A[j,j] - self._A[i,i]     
        else:
            Ajj_ii= -self._A[j,j] + self._A[i,i]
            Aij2 = - Aij2
            pi_to_add.set_val(np.pi)
        
        if (Aij2 > 0):
            sign.set_val(-1)
        else:
            sign.set_val(1)
        
        x, y, theta = self._vectoring_cordic.run(Ajj_ii, Aij2, zero)
        theta = (theta + sign*pi_to_add) >> 1
        
        #diagonal
        temp_ii_i, temp_ii_j, _ = self._rotation_cordic.run(self._A[i,i], self._A[i,j], theta)
        temp_jj_i, temp_jj_j, _ = self._rotation_cordic.run(self._A[i,j], self._A[j,j], theta)
        self._A[i,i], _, _ = self._rotation_cordic.run(temp_ii_i, temp_jj_i, theta)
        _, self._A[j,j], _ = self._rotation_cordic.run(temp_ii_j, temp_jj_j, theta)
          
        #not diagonal i,j
        self._A[i,j] = self._A[j,i] = 0
        
        #not diagonal != i,j
        for k in range(self._N):
            if (k != i and k != j):
                self._A[k,i], self._A[k,j], _ = self._rotation_cordic.run(self._A[k,i], self._A[k,j], theta)
                self._A[i,k] = self._A[k,i]
                self._A[j,k] = self._A[k,j]
    
        #Update V matrix
        for k in range(self._N):
            self._V[k,i], self._V[k,j], _ = self._rotation_cordic.run(self._V[k,i], self._V[k,j], theta)
    
    def do_one_round(self):
        for n in range(0, int(self._N/2)):
            a = self._round_state[n]
            b = self._round_state[self._N - n -1]
            p = np.min([a,b])
            q = np.max([a,b])
            #print("p = {p}, q = {q}".format(p=p, q=q))
            self.update_matrices(p,q)
        
        
        state_1 = self._round_state[1]
        
        for n in range(1,self._N-1):
            self._round_state[n] = self._round_state[n+1]
        self._round_state[self._N-1] = state_1
        
        #print(self._round_state)
    
    def do_one_sweep(self):
        self.init_round_state()
        for i in range(self._N-1):
            self.do_one_round()
    
    def run(self, A):
        self.get_A(A)
        self.init_V()
        for i in range(self._n_sweeps):
            print("Sweep {n}/{N}".format(n = i+1, N = self._n_sweeps), end = '\r')
            self.do_one_sweep()
        return self._A, self._V
    
    
class Jacobi_TV_generator:
    
    def __init__(self,
                 name : str,
                 n_matrices : int, 
                 n_iter : int, 
                 n_word : int,
                 n_frac : int, 
                 is_signed : bool, 
                 N : int, 
                 n_sweeps : int):
        
        self.n_matrices = n_matrices
        self.name = name
        self.jacobi = Jacobi(n_iter, n_word, n_frac, is_signed, N, n_sweeps)
        self.n_word = n_word
        self.n_frac = n_frac
        self.is_signed = is_signed
        self.N = N
        
    @staticmethod
    def symmetric_matrix_to_txt(A : Fxp) -> str:
        N = A.shape[0]
        matrix_string = ""

        #generate upper triangle
        for row in range(N):
            for column in range(row,N):
                matrix_string += "{val}\n".format(val=A[row,column].hex())

        return matrix_string
    
    @staticmethod
    def nonsymmetric_matrix_to_txt(A : Fxp) -> str:
        N = A.shape[0]
        matrix_string = ""

        #generate upper triangle
        for row in range(N):
            for column in range(N):
                matrix_string += "{val}\n".format(val=A[row,column].hex())

        return matrix_string

    def generate_files(self, input_matrices : list):
        
        try:
            os.mkdir("..//TV//{name}".format(name=self.name))
        except OSError as error:
            print(error)
            
        with open("..//TV//{name}//input_matrix.txt".format(name=self.name), 'w') as f_in:
            with open("..//TV//{name}//eigenvectors_matrix.txt".format(name=self.name), 'w') as f_v:
                with open("..//TV//{name}//diagonal_matrix.txt".format(name=self.name), 'w') as f_w:
                    for i in tqdm(range(self.n_matrices)):
                        A = input_matrices[i]
                        w,v = self.jacobi.run(A)

                        in_matrix_string = self.symmetric_matrix_to_txt(A)
                        f_in.write(in_matrix_string)
                        f_in.write("\n")

                        v_matrix_string = self.nonsymmetric_matrix_to_txt(v)
                        f_v.write(v_matrix_string)
                        f_v.write("\n")

                        w_matrix_string = self.symmetric_matrix_to_txt(w)
                        f_w.write(w_matrix_string)
                        f_w.write("\n")
    
    def run(self):
        matrices = []
        for i in range(self.n_matrices):        
            A = (np.random.rand(self.N,self.N)*2)-1
            A = (A + A.T)/2
            matrices.append(Fxp(A, n_word = self.n_frac+1, n_frac = self.n_frac, signed = self.is_signed))
        self.generate_files(matrices)
            