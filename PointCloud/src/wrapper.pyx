from cpython.array cimport array
import Mesh

def call_python_function(list arr):

    result = Mesh.processMesh(arr)
    
    return result