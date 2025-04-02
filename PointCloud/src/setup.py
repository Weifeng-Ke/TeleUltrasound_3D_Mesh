from setuptools import setup, Extension
from Cython.Build import cythonize

extensions = [
    Extension("wrapper", ["wrapper.pyx"]),
]

setup(
    name="CythonWrapper",
    ext_modules=cythonize(extensions),
)