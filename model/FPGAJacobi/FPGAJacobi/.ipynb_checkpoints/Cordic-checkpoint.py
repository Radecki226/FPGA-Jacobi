import numpy as np
from fxpmath import Fxp

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
            new_x.equal(x - d*(y >> i))

            new_y = Fxp(signed=self._is_signed, n_word=self._n_word, n_frac=self._n_frac, rounding = "around")
            new_y.equal(y + d*(x >> i))

            new_z = Fxp(signed=self._is_signed, n_word=self._n_word, n_frac=self._n_frac, rounding = "around")
            new_z.equal(z - d*arctan[i])

            x.equal(new_x)
            y.equal(new_y)
            z.equal(new_z)
        
        x.equal( (x >> 1) + (x >> 3) - (x >> 6) - (x >> 9) - (x >> 13))
        y.equal( (y >> 1) + (y >> 3) - (y >> 6) - (y >> 9) - (y >> 13))

        return(x,y,z)
