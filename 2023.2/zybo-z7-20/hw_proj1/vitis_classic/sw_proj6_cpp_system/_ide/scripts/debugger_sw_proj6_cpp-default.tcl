# Usage with Vitis IDE:
# In Vitis IDE create a Single Application Debug launch configuration,
# change the debug type to 'Attach to running target' and provide this 
# tcl script in 'Execute Script' option.
# Path of this script: F:\Documents\GitHub\zynq-cpp-sandbox\2023.2\zybo-z7-20\hw_proj1\vitis_classic\sw_proj6_cpp_system\_ide\scripts\debugger_sw_proj6_cpp-default.tcl
# 
# 
# Usage with xsct:
# To debug using xsct, launch xsct and run below command
# source F:\Documents\GitHub\zynq-cpp-sandbox\2023.2\zybo-z7-20\hw_proj1\vitis_classic\sw_proj6_cpp_system\_ide\scripts\debugger_sw_proj6_cpp-default.tcl
# 
connect -url tcp:127.0.0.1:3121
targets -set -nocase -filter {name =~"APU*"}
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent Zybo Z7 210351A81FE8A" && level==0 && jtag_device_ctx=="jsn-Zybo Z7-210351A81FE8A-23727093-0"}
fpga -file F:/Documents/GitHub/zynq-cpp-sandbox/2023.2/zybo-z7-20/hw_proj1/vitis_classic/sw_proj6_cpp/_ide/bitstream/hw_proj1_wrapper.bit
targets -set -nocase -filter {name =~"APU*"}
loadhw -hw F:/Documents/GitHub/zynq-cpp-sandbox/2023.2/zybo-z7-20/hw_proj1/vitis_classic/zybo_z7_20_pfm1/export/zybo_z7_20_pfm1/hw/hw_proj1_wrapper.xsa -mem-ranges [list {0x40000000 0xbfffffff}] -regs
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*"}
source F:/Documents/GitHub/zynq-cpp-sandbox/2023.2/zybo-z7-20/hw_proj1/vitis_classic/sw_proj6_cpp/_ide/psinit/ps7_init.tcl
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "*A9*#0"}
dow F:/Documents/GitHub/zynq-cpp-sandbox/2023.2/zybo-z7-20/hw_proj1/vitis_classic/sw_proj6_cpp/Debug/sw_proj6_cpp.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "*A9*#0"}
con
