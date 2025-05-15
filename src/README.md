# HW4 Analog Device Placer
Implement an analog device placer considering symmetry constraints, using the benchmark circuits published in the 2009 IEEE TCAD paper entitled “Analog Placement Based on Symmetry-Island Formulation” by Lin et al.

## How to Compile
In `HW4/src/` directory, enter the following command:
```
$ make
```
An executable file `hw4` will be generated in `HW4/bin/`.

If you want to remove it, please enter the following command:
```
$ make clean
```

## How to Run
In `HW4/bin/` directory, enter the following command:
Format: 
```
$ ./hw4 <input file> <output file>
```

E.g.,
```
$ ./hw4 ../testcase/public1.txt ../output/public1.out
```


