# hbrs_robocup

## Verilook setup
1. Somehow link include and library directory. One way is to add the following
to .bashrc file (replace correct directory):

export CPATH="$CPATH:/opt/Neurotec_Biometric_6_0_SDK/Include"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/Neurotec_Biometric_6_0_SDK/Lib/Linux_x86_64"
