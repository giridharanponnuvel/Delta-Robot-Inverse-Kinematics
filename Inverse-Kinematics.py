#To find Inverse Kinematics Of A Delta Robot
#Reference:
#Paper Name: The Delta Parallel Robot: Kinematics Solutions; Author name: Robert L. Williams II, Ph.D, Mechanical Engineering, Ohio University; Date: October 2016
########################################################################################################################
#Author: Giridharan P
#Date: 16-03-2019
#Credits: Thanks to Asst.Prof Rajeev Lochana G C, Mechanical Department, Amrita School of Engineering, Bangalore
########################################################################################################################
########################################################################################################################
#Note:
#Refer Page 4 & 5, Before you go through this code. Delta robot kinemtatics diagram is given over there.
########################################################################################################################
import numpy as np
import math
import cmath
########################################################################################################################
# Sb,Sp,L variable declaration:
#Refer page:6
try:
    Sb = float(input("Base equilateral triangle side: "))
except ValueError:
    Sb = float(567)
print("Sb:", Sb)

try:
    Sp = float(input("Platform equilateral triangle side: "))
except ValueError:
    Sp = float(76)
print("Sp:", Sp)

try:
    L = float(input("Upper legs length: "))
except ValueError:
    L = float(524)
print("L:", L)

try:
    l = float(input("Lower legs parallelogram length: "))
except ValueError:
    l = float(1244)
print("l:", l)
print()
########################################################################################################################
# x,y,z variable declaration:
# Refer Page: Page 16; x=0,y=0,z=-900 (in mm)
try:
    x = float(input("Enter the 'x' co-ordinate: "))
except ValueError:
    x = float(0)
print("x:", x)

try:
    y = float(input("Enter the 'y' co-ordinate: "))
except ValueError:
    y = float(0)
print("y:", y)

try:
    z = float(input("Enter the 'z' co-ordinate: "))
except ValueError:
    z = float(0)
print("z:", z)
print()
########################################################################################################################
# Wb,Ub,Wp,Up declaration:
#Refer Page: 6
Wb = float(((math.sqrt(3)) / 6) * Sb)
print("Wb:", Wb)

Ub = float(((math.sqrt(3)) / 3) * Sb)
print("Ub:", Ub)

Wp = float(((math.sqrt(3)) / 6) * Sp)
print("Wp:", Wp)

Up = float(((math.sqrt(3)) / 3) * Sp)
print("Up:", Up)
########################################################################################################################
#Refer Page: 11
a = float(Wb - Up)
print("a:", a)

b = float((Sp * 0.5) - (((math.sqrt(3)) * 0.5) * Wb))
print("b:", b)

c = float(Wp - (0.5 * Wb))
print("c:", c)

print()
########################################################################################################################
# Inverse Kinematics:
#Refer Page: 12
E1 = float(2 * L * (y + a))
print("E1:", E1)

F1 = float(2 * z * L)
print("F1:", F1)

G1 = float(
    math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(a, 2) + math.pow(L, 2) + (2 * y * a) - (math.pow(l, 2)))
print("G1:", G1)

E2 = float(-L * (((math.sqrt(3)) * (x + b)) + y + c))
print("E2:", E2)

F2 = float(2 * z * L)
print("F2:", F2)

G2 = float(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (
        2 * ((x * b) + (y * c))) - math.pow(l, 2))
print("G2:", G2)

E3 = float(L * (((math.sqrt(3)) * (x - b)) - y - c))
print("E3:", E3)

F3 = float(2 * z * L)
print("F3:", F3)

G3 = float(math.pow(x, 2) + math.pow(y, 2) + math.pow(z, 2) + math.pow(b, 2) + math.pow(c, 2) + math.pow(L, 2) + (
        2 * (-(x * b) + (y * c))) - math.pow(l, 2))
print("G3:", G3)

print()
########################################################################################################################
#EFG : Pre-Processing  the variables:
#Refer Page: 12
#Quadratic Formula throws few error if we compile all togather hence few operations are pre-processed
E1Sqr = math.pow(E1,2)
F1Sqr = math.pow(F1,2)
G1Sqr = math.pow(G1,2)

E2Sqr = math.pow(E2,2)
F2Sqr = math.pow(F2,2)
G2Sqr = math.pow(G2,2)

E3Sqr = math.pow(E3,2)
F3Sqr = math.pow(F3,2)
G3Sqr = math.pow(G3,2)
########################################################################################################################
EFGsum1 = (E1Sqr + F1Sqr - G1Sqr)
EFGsum2 = (E2Sqr + F2Sqr - G2Sqr)
EFGsum3 = (E3Sqr + F3Sqr - G3Sqr)
########################################################################################################################
if EFGsum1 < 0:
    EFG_1 = -(cmath.sqrt(EFGsum1)).imag
else:
    EFG_1 = math.sqrt(EFGsum1)

if EFGsum2 < 0:
    EFG_2 = -(cmath.sqrt(EFGsum2)).imag
else:
    EFG_2 = math.sqrt(EFGsum2)

if EFGsum3 < 0:
    EFG_3 = -(cmath.sqrt(EFGsum3)).imag
else:
    EFG_3 = math.sqrt(EFGsum3)
print("########################################################################################################################")
print()
# Inverse Kinematics Solution 1:
print("Solution 1:")
T11 = float((-(F1) + EFG_1) / (G1 - E1))
theta11 = 2 * (math.degrees(math.atan(T11)))
print('\u03F4''11:', theta11)

T12 = float((-(F2) + EFG_2) / (G2 - E2))
theta12 = 2 * (math.degrees(math.atan(T12)))
print('\u03F4''12:', theta12)

T13 = float((-(F3) + EFG_3) / (G3 - E3))
theta13 = 2 * (math.degrees(math.atan(T13)))
print('\u03F4''13:', theta13)

print()
########################################################################################################################
# Inverse Kinematics Solution 2:
print("Solution 2:")
T21 = float((-(F1) - EFG_1) / (G1 - E1))
theta21 = 2 * (math.degrees(math.atan(T21)))
print('\u03F4''21:', theta21)

T22 = float((-(F2) - EFG_2) / (G2 - E2))
theta22 = 2 * (math.degrees(math.atan(T22)))
print('\u03F4''22:', theta22)

T23 = float((-(F3) - EFG_3) / (G3 - E3))
theta23 = 2 * (math.degrees(math.atan(T23)))
print('\u03F4''23:', theta23)
########################################################################################################################
#Note:
#You can verify this code by giving the values as mentioned in the paper. Refer Page: 16
########################################################################################################################
########################################################################################################################
