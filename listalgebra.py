#!/usr/bin/env python

from math import sqrt, acos

### Linear algebra on Python lists

# Dot product
def ldot(A,B):
    if len(A) != len(B) or len(A) < 1:
        return 'error'
    return sum(i[0]*i[1] for i in zip(A,B))
    
# Magnitude (Euclidiean norm)
def lnorm(A):
    return sqrt(sum([i**2 for i in A]))
    
# Angle between two vectors
def langle(A,B):
    return acos(ldot(A,B)/(lnorm(A)*lnorm(B)))
    
def ldiff(A,B):
    result = len(A)*[0.0]
    for i in len(B):
        result[i] = A[i] - B[i]
    return result
    
        
# Returns the vector from A TO B, i.e., B-A
def lvector(A,B):
    result = len(A)*[0]
    for i in range(len(A)):
        result[i] = B[i]-A[i] 
    return result 
    
    


