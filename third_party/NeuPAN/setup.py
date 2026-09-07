from setuptools import setup, find_packages

setup(
    name='neupan',
    version='1.3',
    packages=find_packages(),
    install_requires=[
        'cvxpylayers',
        'numpy',
        'scipy<=1.13.0',
        'rich',
        'dill',
        'gctl==1.2',
        'colorama',
        'scikit-learn',
        'pyyaml',
        'torch>=2.1.0',
        'ecos',
    ],
    extras_require={
        'irsim': ['ir-sim>=2.4.0'],
        'all': ['ir-sim>=2.4.0'],
    },
)
