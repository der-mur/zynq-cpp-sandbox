# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct F:\Documents\GitHub\zynq-cpp-sandbox\2023.2\zybo-z7-20\hw_proj1\vitis_classic\zybo_z7_20_pfm1\platform.tcl
# 
# OR launch xsct and run below command.
# source F:\Documents\GitHub\zynq-cpp-sandbox\2023.2\zybo-z7-20\hw_proj1\vitis_classic\zybo_z7_20_pfm1\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {zybo_z7_20_pfm1}\
-hw {F:\Documents\GitHub\zynq-cpp-sandbox\2023.2\zybo-z7-20\hw_proj1\vitis_classic\hw_proj1_wrapper.xsa}\
-proc {ps7_cortexa9_0} -os {standalone} -out {F:/Documents/GitHub/zynq-cpp-sandbox/2023.2/zybo-z7-20/hw_proj1/vitis_classic}

platform write
platform generate -domains 
platform active {zybo_z7_20_pfm1}
platform generate
platform clean
platform generate
