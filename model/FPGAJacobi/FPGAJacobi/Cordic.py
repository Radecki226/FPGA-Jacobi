import numpy as np
from fxpmath import Fxp
import os
from tqdm import tqdm


class Cordic:
    
    def __init__(self, 
                 mode : str = "rotation", 
                 n_iterations : int = 12, 
                 n_word : int = 17, 
                 n_frac : int = 15, 
                 is_signed : bool = True):
        
        self.mode = mode
        self._n_iterations = n_iterations
        self._n_word = n_word
        self._n_frac = n_frac
        self._is_signed = is_signed
    
    def set_mode(self, mode : str):
        self.mode = mode
    
    def run(self, x_in : Fxp, y_in : Fxp, z_in : Fxp) -> tuple:
        #copy
        x = x_in.copy()
        y = y_in.copy()
        z = z_in.copy()
        
        x.rounding = "around"
        y.rounding = "around"
        z.rounding = "around"

        #d will be used as direction of rotation
        d = 0

        #Choose mode
        MODE_ROTATION = 0
        MODE_VECTORING = 1

        cordic_mode = MODE_ROTATION
        criterion = z
        if (self.mode == "vectoring"):
            cordic_mode = MODE_VECTORING
            criterion = y

        #arctan array
        arctan = Fxp(np.array([np.arctan(1/(2**i)) for i in range(self._n_iterations)]),
                     n_word=self._n_word, n_frac=self._n_frac, signed=self._is_signed, rounding = "around")

        for i in range(self._n_iterations):
            #Choose d based on 
            if (cordic_mode == MODE_ROTATION):   
                if (criterion >= 0):
                    d = 1
                else:
                    d = -1
            else:
                if (criterion <= 0):
                    d = 1
                else:
                    d = -1

            new_x = Fxp(signed=self._is_signed, n_word=self._n_word, n_frac=self._n_frac, rounding = "around")
            y_shift = y.copy()
            y_shift.equal(y >> i)

            new_y = Fxp(signed=self._is_signed, n_word=self._n_word, n_frac=self._n_frac, rounding = "around")
            x_shift = x.copy()
            x_shift.equal(x >> i)

            new_z = Fxp(signed=self._is_signed, n_word=self._n_word, n_frac=self._n_frac, rounding = "around")

            if (d > 0):
              new_x.equal(x - y_shift)
              new_y.equal(y + x_shift)
              new_z.equal(z - arctan[i])
            else:
              new_x.equal(x + y_shift)
              new_y.equal(y - x_shift)
              new_z.equal(z + arctan[i])

            x.equal(new_x)
            y.equal(new_y)
            z.equal(new_z)
        
        def booth(x_in):
            x1 = x_in.copy()
            x1.equal(x1 >> 1)
            x3 = x_in.copy()
            x3.equal(x3 >> 3)
            x5 = x_in.copy()
            x5.equal(x5 >> 5)
            x6 = x_in.copy()
            x6.equal(x6 >> 6)
            x8 = x_in.copy()
            x8.equal(x8 >> 8)
            x9 = x_in.copy()
            x9.equal(x9 >> 9)
            x12 = x_in.copy()
            x12.equal(x12 >> 12)
            x13 = x_in.copy()
            x13.equal(x13 >> 13)
            x14 = x_in.copy()
            x14.equal(x14 >> 14)
            x_out = x_in.copy()
            x_out.equal(x_in - x1 + x3 - x5 + x6 - x8 + x9 - x12 + x13 - x14)
            fgfg = Fxp(8.3)
            return x_out
        x.equal(booth(x))
        y.equal(booth(y))

        return(x,y,z)
    
class Cordic_TV_generator:
    
    def __init__(self,
                 name : str,
                 n_vectors : int,
                 n_iter : int, 
                 n_word : int,
                 n_frac : int, 
                 is_signed : bool,
                 mode : str = "rotation"):
        
        self.name = name
        self.n_vectors = n_vectors
        self.cordic = Cordic(mode, n_iter, n_word, n_frac, is_signed)
        self.n_word = n_word
        self.n_frac = n_frac
        self.is_signed = is_signed
        self.mode = mode
        
    @staticmethod
    def vector3_to_txt(x : Fxp, y : Fxp, z: Fxp) -> str:

        vector_string = "{x} {y} {z}".format(x = x.hex(), y = y.hex(), z = z.hex())

        return vector_string

    def generate_files(self, input_vectors : list):
        
        try:
            os.mkdir("..//TV//{name}".format(name=self.name))
        except OSError as error:
            print(error)
            
        with open("..//TV//{name}//input_vectors.txt".format(name=self.name), 'w') as f_in:
            with open("..//TV//{name}//output_vectors.txt".format(name=self.name), 'w') as f_out:
                    for i in tqdm(range(self.n_vectors)):
                        v_in = input_vectors[i]
                        v_out = self.cordic.run(*v_in)


                        in_vector_string = self.vector3_to_txt(*v_in)
                        f_in.write(in_vector_string)
                        f_in.write("\n")

                        out_vector_string = self.vector3_to_txt(*v_out)
                        f_out.write(out_vector_string)
                        f_out.write("\n")

    
    def run(self):
        vectors = []
        for i in range(self.n_vectors):        
            x_in = Fxp(np.random.rand()*2-1,n_word = self.n_word, n_frac = self.n_frac, signed = self.is_signed)
            y_in = Fxp(np.random.rand()*2-1,n_word = self.n_word, n_frac = self.n_frac, signed = self.is_signed)
            
            if self.mode == "vectoring":
                z_in = Fxp(0,n_word = self.n_word, n_frac = self.n_frac, signed = self.is_signed)
            else:
                z_in = Fxp((np.random.rand()*2-1)*np.pi/2,n_word = self.n_word, n_frac = self.n_frac, signed = self.is_signed)
            
            vectors.append((x_in, y_in, z_in))
        self.generate_files(vectors)
