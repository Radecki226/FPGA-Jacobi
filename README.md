# FPGA-Jacobi


## Install

```shell
pip install pipenv
```
go to main project directory and run:
```shell
pipenv shell
```
go to model and run:
```shell
pipenv install fpgajacobi --skip-lock
pipenv jupyter lab
```


## Project hierarchy

- `model` : python fxp model of algorithm
- `docs`  : documentation, drawings, etc..
- `rtl`   : register tranfer level code 
- `verif` : systemverilog verification of given circuit
- `soft`  : software necessary to run application

Moreover `jacobi_project.rar` contains packed vivado project and `Metoda_Jacobiego.pdf` contains project report.

## Circuit main schematic

- ![picture alt](docs/Jacobi.svg)
