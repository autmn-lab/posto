'''
Parameters required for the code
'''
import os,sys

'''
Please add the following line in ~/.bashrc
export MNTR_BB_ROOT_DIR = <YOUR PROJECT ROOT>
'''

PROJECT_ROOT = os.environ['MNTR_BB_ROOT_DIR']
sys.path.append(PROJECT_ROOT)

LIB_PATH=PROJECT_ROOT+'/'+'lib/'
SRC_PATH=PROJECT_ROOT+'/'+'src/'
OUTPUT_PATH=PROJECT_ROOT+'/'+'output/'
PICKLE_PATH=PROJECT_ROOT+'/'+'pickles/'
DATA_PATH=PROJECT_ROOT+'/'+'data/'


PICKLE_FLAG=True
REFINE=True

B=100000
c=0.99

VIZ_PER_COVERAGE=20



# Jet
PROBABILITY_LOG=3
DT=0.01
DELTA_STATE=0.002
DELTA_LOG=0.02


'''
# Vander Pol
PROBABILITY_LOG=3
DT=0.01
DELTA_STATE=0.004
DELTA_LOG=0.3
'''