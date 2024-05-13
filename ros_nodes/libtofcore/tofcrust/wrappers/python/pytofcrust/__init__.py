# make sure .pyd or .so files are copied to pytofcrust dir before building wheels
# pytofcrust needs pytofcrust and pytofcore artifacts so it builds properly

from .pytofcrust import *