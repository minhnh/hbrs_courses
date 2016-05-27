## Repository for Robocup@Home at HBRS

#### Verilook setup
1. Link include and library directory so GCC can see the SDK. One way is to add the following
to .bashrc file (replace correct directory):

export CPATH="$CPATH:/opt/Neurotec_Biometric_6_0_SDK/Include"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/opt/Neurotec_Biometric_6_0_SDK/Lib/Linux_x86_64"

#### Verilook possible problems
1. Changing ```LICENSE_SERVER``` to something other than ```\local``` may give ```IO Error```
