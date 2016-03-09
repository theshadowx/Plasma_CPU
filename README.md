## Description
The Plasma CPU is a small synthesizable 32-bit RISC microprocessor. It is currently running a live web server with an interrupt controller, UART, SRAM or DDR SDRAM controller, and Ethernet controller. The Plasma CPU executes all MIPS I(TM) user mode instructions except unaligned load and store operations (see "Avoiding Limitations" below). 

This "clean room" CPU core is implemented in VHDL with either a two or three-stage pipeline. It is running at 25 MHz on a Xilinx FPGA and also verified on an Altera FPGA.

To know more about Plasma CPU --> [OpenCores](http://opencores.org/project,plasma,overview)
